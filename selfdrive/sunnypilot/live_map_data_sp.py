import json
import numpy as np
import threading
import traceback

from cereal import messaging, custom
from openpilot.common.params_pyx import Params
from openpilot.common.realtime import Ratekeeper, set_core_affinity
from openpilot.selfdrive.navd.helpers import Coordinate
from openpilot.system.swaglog import cloudlog

LOOK_AHEAD_HORIZON_TIME = 15.  # s. Time horizon for look ahead of turn speed sections to provide on liveMapDataSP msg.

_DEBUG = False
_CLOUDLOG_DEBUG = True
ROAD_NAME_TIMEOUT = 30  # secs
DataType = custom.LiveMapDataSP.DataType
R = 6373000.0  # approximate radius of earth in mts
QUERY_RADIUS = 3000  # mts. Radius to use on OSM data queries.
QUERY_RADIUS_OFFLINE = 2250  # mts. Radius to use on offline OSM data queries.


# PFEIFER - MAPD {{
mem_params = Params("/dev/shm/params")
params = Params()
# }} PFEIFER - MAPD


def _debug(msg, log_to_cloud=True):
  if _CLOUDLOG_DEBUG and log_to_cloud:
    cloudlog.debug(msg)
  if _DEBUG:
    print(msg)


class LiveMapSp:
  def __init__(self):
    self.gps_sock = None
    self.last_gps = Coordinate(0.0, 0.0)
    self.last_refresh_loc: Coordinate | None = None
    self.last_query_radius = QUERY_RADIUS
    self.data_type = DataType.default
    self.sm = messaging.SubMaster(['gpsLocation', 'gpsLocationExternal', 'carControl'])
    self.pm = messaging.PubMaster(['liveMapDataSP'])

  def _select_gps_socket(self):
    return 'gpsLocationExternal' if self.sm.rcv_frame['gpsLocationExternal'] > 1 else 'gpsLocation'

  def _get_current_bounding_box(self, radius: float):
    self.last_query_radius = radius
    # Calculate the bounding box coordinates for the bbox containing the circle around location.
    bbox_angle = float(np.degrees(radius / R))

    lat = float(self.last_gps.latitude)
    lon = float(self.last_gps.longitude)

    return {
      "min_lat": lat - bbox_angle,
      "min_lon": lon - bbox_angle,
      "max_lat": lat + bbox_angle,
      "max_lon": lon + bbox_angle,
    }

  def _request_refresh_osm_bounds_data(self):
    self.last_refresh_loc = Coordinate(self.last_gps.latitude, self.last_gps.longitude)
    self.last_query_radius = QUERY_RADIUS
    current_bounding_box = self._get_current_bounding_box(self.last_query_radius)
    mem_params.put("OSMDownloadBounds", json.dumps(current_bounding_box))

  def update_gps(self):
    self.gps_sock = self._select_gps_socket()
    if not self.sm.updated[self.gps_sock] or not self.sm.valid[self.gps_sock]:
      return

    _last_gps = self.sm[self.gps_sock]
    # ignore the message if the fix is invalid
    if _last_gps.flags % 2 == 0:
      return

    self.last_gps = Coordinate(_last_gps.latitude, _last_gps.longitude)
    self.last_gps.annotations['unixTimestampMillis'] = _last_gps.unixTimestampMillis
    self.last_gps.annotations['latitude'] = _last_gps.latitude
    self.last_gps.annotations['longitude'] = _last_gps.longitude
    self.last_gps.annotations['speed'] = _last_gps.speed
    self.last_gps.annotations['bearingDeg'] = _last_gps.bearingDeg
    self.last_gps.annotations['accuracy'] = _last_gps.accuracy if self.gps_sock == 'gpsLocationExternal' else 1
    self.last_gps.annotations['bearingAccuracyDeg'] = _last_gps.bearingAccuracyDeg

    last_gps_postition_for_osm = {
      "latitude": self.last_gps.latitude,
      "longitude": self.last_gps.longitude,
      "bearing": self.last_gps.annotations.get("bearingDeg", 0)
    }

    mem_params.put("LastGPSPosition", json.dumps(last_gps_postition_for_osm))

  @staticmethod
  def get_speed_limit():
    speed_limit = mem_params.get("MapSpeedLimit", encoding='utf8')
    return float(speed_limit) if speed_limit else 0.0

  @staticmethod
  def get_road_name():
    current_road_name = mem_params.get("RoadName", encoding='utf8')
    return current_road_name if current_road_name else ""

  def get_next_speed_limit_section(self):
    next_speed_limit_section_str = mem_params.get("NextMapSpeedLimit", encoding='utf8')
    next_speed_limit_section = json.loads(next_speed_limit_section_str) if next_speed_limit_section_str else {}
    next_speed_limit = next_speed_limit_section.get('speedlimit', 0.0)
    next_speed_limit_latitude = next_speed_limit_section.get('latitude')
    next_speed_limit_longitude = next_speed_limit_section.get('longitude')
    next_speed_limit_distance = 0

    if next_speed_limit_latitude and next_speed_limit_longitude:
      next_speed_limit_coordinates = Coordinate(next_speed_limit_latitude, next_speed_limit_longitude)
      next_speed_limit_distance = self.last_gps.distance_to(next_speed_limit_coordinates)

    return next_speed_limit, next_speed_limit_distance

  def create_live_map_data(self, speed_limit, current_road_name, next_speed_limit, next_speed_limit_distance):
    map_data_msg = messaging.new_message('liveMapDataSP')
    map_data_msg.valid = self.sm.all_alive(service_list=[self.gps_sock]) and self.sm.all_valid(service_list=[self.gps_sock])
    live_map_data = map_data_msg.liveMapDataSP

    if self.last_gps:
      live_map_data.lastGpsTimestamp = self.last_gps.annotations.get('unixTimestampMillis', 0)
      live_map_data.lastGpsLatitude = self.last_gps.annotations.get('latitude', 0)
      live_map_data.lastGpsLongitude = self.last_gps.annotations.get('longitude', 0)
      live_map_data.lastGpsSpeed = self.last_gps.annotations.get('speed', 0)
      live_map_data.lastGpsBearingDeg = self.last_gps.annotations.get('bearingDeg', 0)
      live_map_data.lastGpsAccuracy = self.last_gps.annotations.get('accuracy', 0)
      live_map_data.lastGpsBearingAccuracyDeg = self.last_gps.annotations.get('bearingAccuracyDeg', 0)

    live_map_data.speedLimitValid = bool(speed_limit > 0)
    live_map_data.speedLimit = speed_limit
    live_map_data.speedLimitAheadValid = bool(next_speed_limit > 0)
    live_map_data.speedLimitAhead = float(next_speed_limit)
    live_map_data.speedLimitAheadDistance = float(next_speed_limit_distance)
    live_map_data.currentRoadName = str(current_road_name)
    live_map_data.dataType = self.data_type

    # Former M-TSC implementation
    # live_map_data.turnSpeedLimitValid = bool(turn_speed_limit_section is not None)
    # live_map_data.turnSpeedLimit = float(turn_speed_limit_section.value
    #                                                 if turn_speed_limit_section is not None else 0.0)
    # live_map_data.turnSpeedLimitSign = int(turn_speed_limit_section.curv_sign
    #                                                   if turn_speed_limit_section is not None else 0)
    # live_map_data.turnSpeedLimitEndDistance = float(turn_speed_limit_section.end
    #                                                            if turn_speed_limit_section is not None else 0.0)
    # live_map_data.turnSpeedLimitsAhead = [float(s.value) for s in next_turn_speed_limit_sections]
    # live_map_data.turnSpeedLimitsAheadDistances = [float(s.start) for s in next_turn_speed_limit_sections]
    # live_map_data.turnSpeedLimitsAheadSigns = [float(s.curv_sign) for s in next_turn_speed_limit_sections]
    return map_data_msg

  def publish(self):
    speed_limit = self.get_speed_limit()
    current_road_name = self.get_road_name()
    next_speed_limit, next_speed_limit_distance = self.get_next_speed_limit_section()
    map_data_msg = self.create_live_map_data(speed_limit, current_road_name, next_speed_limit, next_speed_limit_distance)

    self.pm.send('liveMapDataSP', map_data_msg)
    # Replace _debug with actual debug function
    _debug(f'Mapd *****: Publish: \n{map_data_msg}\n********', log_to_cloud=False)

  def update(self):
    self.sm.update()
    self.update_gps()


def excepthook(args):
  _debug(f'MapD: Threading exception:\n{args}')
  traceback.print_exception(args.exc_type, args.exc_value, args.exc_traceback)


def live_map_data_sp_thread():
  try:
    set_core_affinity([0, 1, 2, 3])
  except Exception:
    cloudlog.exception("mapd: failed to set core affinity")
  live_map_sp = LiveMapSp()
  rk = Ratekeeper(1, print_delay_threshold=None)

  while True:
    live_map_sp.update()
    live_map_sp.publish()
    rk.keep_time()


def main():
  threading.excepthook = excepthook
  live_map_data_sp_thread()


if __name__ == "__main__":
  main()

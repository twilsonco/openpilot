#!/usr/bin/env python3
import threading
from traceback import print_exception
import numpy as np
from time import strftime, gmtime
import cereal.messaging as messaging
from collections import defaultdict
from common.params import Params
from common.op_params import opParams
from common.realtime import Ratekeeper
import requests
from selfdrive.mapd.lib.osm import OSM
from selfdrive.mapd.lib.geo import distance_to_points
from selfdrive.mapd.lib.WayCollection import WayCollection
from selfdrive.mapd.config import QUERY_RADIUS, MIN_DISTANCE_FOR_NEW_QUERY, FULL_STOP_MAX_SPEED, LOOK_AHEAD_HORIZON_TIME
from selfdrive.swaglog import cloudlog


_DEBUG = False

WEATHER_BASE_URL="http://api.openweathermap.org/data/2.5/weather?"
WEATHER_DEFAULT_API_KEY = ["128gb5631e9bff6:", "c:2d2d6c8gb5742e"]
def rot_str(s,n):
  return ''.join([chr(ord(i)+n) for i in s])

def _debug(msg):
  if not _DEBUG:
    return
  print(msg)


def excepthook(args):
  _debug(f'MapD: Threading exception:\n{args}')
  print_exception(args.exc_type, args.exc_value, args.exc_traceback)


threading.excepthook = excepthook

class WeatherD():
  def __init__(self):
    self._op_params = opParams(calling_function="mapd.py WeatherD")
    self.api_key = self._op_params.get('MISC_open_weather_map_api_key', force_update=True)
    if self.api_key is not None:
      self.api_key = self.api_key.strip()
    if self.api_key is None or len(self.api_key) < 32:
      self.api_key = rot_str(''.join(WEATHER_DEFAULT_API_KEY), -1)
    if len(self.api_key) == 32:
      cloudlog.info(f"liveWeatherData: using OpenWeatherMap.org api key: {'*' * 24}{self.api_key[-8:]}")
    else:
      cloudlog.info("liveWeatherData: no OpenWeatherMap.org api key provided")
    self.weather = None
    self._lock = threading.RLock()
    self._query_thread = None
    
  def update(self, lat, lon):
    weather = None
    if len(self.api_key) > 30:
      url = f"{WEATHER_BASE_URL}lat={lat}&lon={lon}&appid={self.api_key}"
      response = requests.get(url)
      x = response.json()
      if x["cod"] != "404":
        weather = x
    return weather
        
  def _query_owm_not_blocking(self, lat, lon):
    def query(lat, lon):
      _debug(f'WeatherD: Start query for OWM map data at {(lat,lon)}')
      weather = self.update(lat, lon)
      _debug('WeatherD: Query to OWM finished {}successfully'.format("un" if weather is None else ""))

      cloudlog.info(f"Fetched weather: {weather}")
      # Only issue an update if we received new weather. Otherwise it is most likely a conectivity issue.
      # Use the lock to update weather as it might be being used to update the route.
      _debug('WeatherD: Locking to write results from OWM.')
      with self._lock:
        self.weather = weather
        _debug(f'WeatherD: Updated map data @ {(lat,lon)}')
      _debug('WeatherD: Releasing Lock to write results from OWM')

    # Ignore if we have a query thread already running.
    if self._query_thread is not None and self._query_thread.is_alive():
      return

    self._query_thread = threading.Thread(target=query, 
                                          args=(lat,lon))
    self._query_thread.start()
  
  def publish(self, pm, sm):
    if len(self.api_key) < 32:
      return 1
    if self.weather is None:
      return -5 # tries again in  seconds
    
    def publish_weather():
      w = messaging.new_message('liveWeatherData')
      
      ww = self.weather
      w.liveWeatherData.valid = True
      x = ww["weather"][0]
      w.liveWeatherData.main = x["main"]
      w.liveWeatherData.weatherID = x["id"]
      w.liveWeatherData.description = x["description"]
      w.liveWeatherData.icon = x["icon"]
      x = ww["main"]
      w.liveWeatherData.temperature = x["temp"] - 273.15
      w.liveWeatherData.temperatureFeelsLike = x["feels_like"] - 273.15
      w.liveWeatherData.pressure = x["pressure"]
      w.liveWeatherData.humidity = x["humidity"]
      w.liveWeatherData.visibility = ww["visibility"]
      if "wind" in ww:
        x = ww["wind"]
        w.liveWeatherData.windSpeed = x["speed"]
        w.liveWeatherData.windDirectionDeg = x["deg"]
        w.liveWeatherData.windSpeedGust = x["gust"] if "gust" in x else 0.0
      w.liveWeatherData.cloudsPercent = ww["clouds"]["all"]
      w.liveWeatherData.cityName = ww["name"]
      if "rain" in ww:
        w.liveWeatherData.rain1Hour = ww["rain"]["1h"] if "1h" in ww["rain"] else 0.0
        w.liveWeatherData.rain3Hour = ww["rain"]["3h"] if "3h" in ww["rain"] else 0.0
      if "snow" in ww:
        w.liveWeatherData.snow1Hour = ww["snow"]["1h"] if "1h" in ww["snow"] else 0.0
        w.liveWeatherData.snow3Hour = ww["snow"]["3h"] if "3h" in ww["snow"] else 0.0
      w.liveWeatherData.timeCurrent = ww["dt"]
      w.liveWeatherData.timeZone = ww["timezone"]
      x = ww["sys"]
      w.liveWeatherData.timeSunrise = x["sunrise"]
      w.liveWeatherData.timeSunset = x["sunset"]
      
      pm.send('liveWeatherData', w)
      cloudlog.info(f"WeatherD: Current weather in {w.liveWeatherData.cityName}: {w.liveWeatherData.temperature:0.1f}C and {w.liveWeatherData.description}")
    
    with self._lock:
      publish_weather()
    
    return 1 # used to increment a counter

class MapD():
  def __init__(self):
    self.osm = OSM()
    self.way_collection = None
    self.route = None
    self.last_gps_fix_timestamp = 0
    self.last_gps = None
    self.location_deg = None  # The current location in degrees.
    self.location_rad = None  # The current location in radians as a Numpy array.
    self.bearing_rad = None
    self.accuracy = None  # The current location accuracy in mts. 2 sigma.
    self.gps_speed = 0.
    self.last_fetch_location = None
    self.last_route_update_fix_timestamp = 0
    self.last_publish_fix_timestamp = 0
    self._op_enabled = False
    self._disengaging = False
    self._query_thread = None
    self._lock = threading.RLock()

  def udpate_state(self, sm):
    sock = 'controlsState'
    if not sm.updated[sock] or not sm.valid[sock]:
      return

    controls_state = sm[sock]
    self._disengaging = not controls_state.enabled and self._op_enabled
    self._op_enabled = controls_state.enabled

  def update_gps(self, sm):
    sock = 'gpsLocationExternal'
    if not sm.updated[sock] or not sm.valid[sock]:
      return

    log = sm[sock]
    self.last_gps = log

    # ignore the message if the fix is invalid
    if log.flags % 2 == 0:
      return

    self.last_gps_fix_timestamp = log.timestamp  # Unix TS. Milliseconds since January 1, 1970.
    self.location_rad = np.radians(np.array([log.latitude, log.longitude], dtype=float))
    self.location_deg = (log.latitude, log.longitude)
    self.bearing_rad = np.radians(log.bearingDeg, dtype=float)
    self.gps_speed = log.speed
    # log accuracies are presumably 1 sigma. To work with an accuracy that gives us >95% confidence and working
    # on the assumption that fix location follows a bell distribution we should use 2 * sigma instead.
    self.accuracy = log.accuracy * 2.

    _debug('Mapd: ********* Got GPS fix'
           f'Pos: {self.location_deg} +/- {self.accuracy} mts.\n'
           f'Bearing: {log.bearingDeg} +/- {log.bearingAccuracyDeg * 2.} deg.\n'
           f'timestamp: {strftime("%d-%m-%y %H:%M:%S", gmtime(self.last_gps_fix_timestamp * 1e-3))}'
           f'*******')

  def _query_osm_not_blocking(self):
    def query(osm, location_deg, location_rad, radius):
      _debug(f'Mapd: Start query for OSM map data at {location_deg}')
      lat, lon = location_deg
      ways = osm.fetch_road_ways_around_location(lat, lon, radius)
      _debug(f'Mapd: Query to OSM finished with {len(ways)} ways')

      # Only issue an update if we received some ways. Otherwise it is most likely a conectivity issue.
      # Will retry on next loop.
      if len(ways) > 0:
        new_way_collection = WayCollection(ways, location_rad)

        # Use the lock to update the way_collection as it might be being used to update the route.
        _debug('Mapd: Locking to write results from osm.')
        with self._lock:
          self.way_collection = new_way_collection
          self.last_fetch_location = location_rad
          _debug(f'Mapd: Updated map data @ {location_deg} - got {len(ways)} ways')

        _debug('Mapd: Releasing Lock to write results from osm')

    # Ignore if we have a query thread already running.
    if self._query_thread is not None and self._query_thread.is_alive():
      return

    self._query_thread = threading.Thread(target=query, 
                                          args=(self.osm, 
                                                self.location_deg, 
                                                self.location_rad,
                                                QUERY_RADIUS))
    self._query_thread.start()

  def updated_osm_data(self):
    if self.route is not None:
      distance_to_end = self.route.distance_to_end
      if distance_to_end is not None and distance_to_end >= MIN_DISTANCE_FOR_NEW_QUERY:
        # do not query as long as we have a route with enough distance ahead.
        return

    if self.location_rad is None:
      return

    if self.last_fetch_location is not None:
      distance_since_last = distance_to_points(self.last_fetch_location, np.array([self.location_rad]))[0]
      if distance_since_last < QUERY_RADIUS - MIN_DISTANCE_FOR_NEW_QUERY:
        # do not query if are still not close to the border of previous query area
        return

    self._query_osm_not_blocking()

  def update_route(self):
    def update_proc():
      # Ensure we clear the route on op disengage, this way we can correct possible incorrect map data due
      # to wrongly locating or picking up the wrong route.
      if self._disengaging:
        self.route = None
        _debug('Mapd *****: Clearing Route as system is disengaging. ********')

      if self.way_collection is None or self.location_rad is None or self.bearing_rad is None:
        _debug('Mapd *****: Can not update route. Missing WayCollection, location or bearing ********')
        return

      if self.route is not None and self.last_route_update_fix_timestamp == self.last_gps_fix_timestamp:
        _debug('Mapd *****: Skipping route update. No new fix since last update ********')
        return

      self.last_route_update_fix_timestamp = self.last_gps_fix_timestamp

      # Create the route if not existent or if it was generated by an older way collection
      if self.route is None or self.route.way_collection_id != self.way_collection.id:
        self.route = self.way_collection.get_route(self.location_rad, self.bearing_rad, self.accuracy)
        _debug(f'Mapd *****: Route created: \n{self.route}\n********')
        return

      # Do not attempt to update the route if the car is going close to a full stop, as the bearing can start
      # jumping and creating unnecesary loosing of the route. Since the route update timestamp has been updated
      # a new liveMapData message will be published with the current values (which is desirable)
      if self.gps_speed < FULL_STOP_MAX_SPEED:
        _debug('Mapd *****: Route Not updated as car has Stopped ********')
        return

      self.route.update(self.location_rad, self.bearing_rad, self.accuracy)
      if self.route.located:
        _debug(f'Mapd *****: Route updated: \n{self.route}\n********')
        return

      # if an old route did not mange to locate, attempt to regenerate form way collection.
      self.route = self.way_collection.get_route(self.location_rad, self.bearing_rad, self.accuracy)
      _debug(f'Mapd *****: Failed to update location in route. Regenerated with route: \n{self.route}\n********')

    # We use the lock when updating the route, as it reads `way_collection` which can ben updated by
    # a new query result from the _query_thread.
    _debug('Mapd: Locking to update route.')
    with self._lock:
      update_proc()

    _debug('Mapd: Releasing Lock to update route')

  def publish(self, pm, sm):
    # Ensure we have a route currently located
    if self.route is None or not self.route.located:
      return

    # Ensure we have a route update since last publish
    if self.last_publish_fix_timestamp == self.last_route_update_fix_timestamp:
      return

    self.last_publish_fix_timestamp = self.last_route_update_fix_timestamp

    speed_limit = self.route.current_speed_limit
    next_speed_limit_section = self.route.next_speed_limit_section
    turn_speed_limit_section = self.route.current_curvature_speed_limit_section
    horizon_mts = self.gps_speed * LOOK_AHEAD_HORIZON_TIME
    next_turn_speed_limit_sections = self.route.next_curvature_speed_limit_sections(horizon_mts)
    current_road_name = self.route.current_road_name
    current_road_type = self.route.current_road_type

    map_data_msg = messaging.new_message('liveMapData')
    map_data_msg.valid = sm.all_alive_and_valid(service_list=['gpsLocationExternal'])

    map_data_msg.liveMapData.lastGpsTimestamp = self.last_gps.timestamp
    map_data_msg.liveMapData.speedLimitValid = bool(speed_limit is not None)
    map_data_msg.liveMapData.speedLimit = float(speed_limit if speed_limit is not None else 0.0)
    map_data_msg.liveMapData.speedLimitAheadValid = bool(next_speed_limit_section is not None)
    map_data_msg.liveMapData.speedLimitAhead = float(next_speed_limit_section.value
                                                     if next_speed_limit_section is not None else 0.0)
    map_data_msg.liveMapData.speedLimitAheadDistance = float(next_speed_limit_section.start
                                                             if next_speed_limit_section is not None else 0.0)

    map_data_msg.liveMapData.turnSpeedLimitValid = bool(turn_speed_limit_section is not None)
    map_data_msg.liveMapData.turnSpeedLimit = float(turn_speed_limit_section.value
                                                    if turn_speed_limit_section is not None else 0.0)
    map_data_msg.liveMapData.turnSpeedLimitSign = int(turn_speed_limit_section.curv_sign
                                                      if turn_speed_limit_section is not None else 0)
    map_data_msg.liveMapData.turnSpeedLimitEndDistance = float(turn_speed_limit_section.end
                                                               if turn_speed_limit_section is not None else 0.0)
    map_data_msg.liveMapData.turnSpeedLimitsAhead = [float(s.value) for s in next_turn_speed_limit_sections]
    map_data_msg.liveMapData.turnSpeedLimitsAheadDistances = [float(s.start) for s in next_turn_speed_limit_sections]
    map_data_msg.liveMapData.turnSpeedLimitsAheadSigns = [float(s.curv_sign) for s in next_turn_speed_limit_sections]
    map_data_msg.liveMapData.currentRoadName = str(current_road_name if current_road_name is not None else "")
    map_data_msg.liveMapData.currentRoadType = int(current_road_type if current_road_type is not None else 100)

    pm.send('liveMapData', map_data_msg)
    _debug(f'Mapd *****: Publish: \n{map_data_msg}\n********')


# provides live map data information
def mapd_thread(sm=None, pm=None):
  mapd = MapD()
  weatherd = WeatherD()
  rk = Ratekeeper(1., print_delay_threshold=None)  # Keeps rate at 1 hz

  # *** setup messaging
  if sm is None:
    sm = messaging.SubMaster(['gpsLocationExternal', 'controlsState'])
  if pm is None:
    pm = messaging.PubMaster(['liveMapData', 'liveWeatherData'])
    
  # see if mapd features are enabled
  params = Params()

  slc = params.get_bool("SpeedLimitControl")
  tsc = params.get_bool("TurnSpeedControl")
  enable_mapd = slc or tsc

  weather_iter = 0
  weather_freq = 10
  weather_check = weather_freq * 18
  while True:
    sm.update()
    mapd.udpate_state(sm)
    mapd.update_gps(sm)
    if enable_mapd:
      mapd.updated_osm_data()
      mapd.update_route()
    mapd.publish(pm, sm)
    if weather_iter % weather_freq == 0: # check weather every 3 minutes
      if mapd.location_deg is not None \
        and (weatherd.weather is None or weather_iter % weather_check == 0):
        lat, lon = mapd.location_deg
        weatherd._query_owm_not_blocking(lat, lon)
      weather_iter += weatherd.publish(pm, sm)
    else:
      weather_iter += 1
    rk.keep_time()


def main(sm=None, pm=None):
  mapd_thread(sm, pm)


if __name__ == "__main__":
  main()

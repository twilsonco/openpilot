#!/usr/bin/env python
import argparse
import os
import sys
from common.basedir import BASEDIR
from tools.lib.logreader import MultiLogIterator
from tools.lib.route import Route
from datetime import datetime
import gpxpy.gpx
import json

os.environ['BASEDIR'] = BASEDIR


def get_arg_parser():
  parser = argparse.ArgumentParser(
      description="Create gpx track file from rlog route",
      formatter_class=argparse.ArgumentDefaultsHelpFormatter)

  parser.add_argument("data_dir", nargs='?',
                              help="Path to directory in which log and camera files are located.")
  parser.add_argument("route_name", type=(lambda x: x.replace("#", "|")), nargs="?",
                      help="The route whose messages will be published.")
  parser.add_argument("--out_path", nargs='?', default='',
                      help="Output pickle file path")
  return parser

class GTP:
  valid_list = ["lat", "lon", "elevation", "time", "speed", "satellites"]
  valid_len = 32
  def __init__(self):
    self.d = {k:None for k in self.valid_list}
  
  def is_valid(self):
    return None not in [self.d[k] for k in self.valid_list] and len(self.d) == self.valid_len
  
  def to_gtp(self):
    out = gpxpy.gpx.GPXTrackPoint(latitude=self.d["lat"], 
                                   longitude=self.d["lon"],
                                   elevation=self.d["elevation"],
                                   time=self.d["time"],
                                   speed=self.d["speed"])
    for k in ["satellites", "course", "source"]:
      if self.d[k] is not None:
        setattr(out, k, self.d[k])
    self.d["time"] = self.d["time"].strftime("%m/%d/%Y %H:%M:%S")
    out.description = json.dumps(self.d)
    for k in ["lat", "lon", "speed"]:
      del(self.d[k])
    out.comment = "\n".join([f"{k}: {v}" if type(v) != float else f"{k}: {v:0.2f}" for k,v in sorted(self.d.items(),key=lambda x:x[0])])
    return out

def main(argv):
  args = get_arg_parser().parse_args(sys.argv[1:])
  if not args.data_dir:
    print('Data directory invalid.')
    return

  if not args.route_name:
    # Extract route name from path
    args.route_name = os.path.basename(args.data_dir)
    args.data_dir = os.path.dirname(args.data_dir)

  route = Route(args.route_name, args.data_dir)
  lr = MultiLogIterator(route.log_paths())
  
  gpx = gpxpy.gpx.GPX()
  gpx_track = gpxpy.gpx.GPXTrack()
  gpx.tracks.append(gpx_track)
  gpx_segment = gpxpy.gpx.GPXTrackSegment()
  gpx_track.segments.append(gpx_segment)
  
  gpx_segment.points.append(gpxpy.gpx.GPXTrackPoint(2.1234, 5.1234, elevation=1234))

  try:
    done = False
    i = 0
    gtp = GTP()
    distance = 0.0
    time = 0.0
    avg_speed = 0.0
    while not done:
      msg = next(lr)
      if not msg:
        break
      typ = msg.which()
      if typ == 'carState':
        gtp.d["speed"] = float(msg.carState.vEgo)
        gtp.d["accelLong"] = float(msg.carState.aEgo)
        gtp.d["gas"] = float(msg.carState.gas)
        gtp.d["brake"] = float(msg.carState.brake)
        gtp.d["steerAngle"] = float(msg.carState.steeringAngleDeg)
        gtp.d["steerTorqueEPS"] = float(msg.carState.steeringTorqueEps)
        gtp.d["steerTorque"] = float(msg.carState.steeringTorque)
        gtp.d["blinkerLeft"] = float(msg.carState.leftBlinker)
        gtp.d["blinkerRight"] = float(msg.carState.rightBlinker)
        gtp.d["doorOpen"] = float(msg.carState.doorOpen)
        gtp.d["seatbeltUnlatched"] = float(msg.carState.seatbeltUnlatched)
        gtp.d["blindspotLeft"] = float(msg.carState.leftBlindspot)
        gtp.d["blinkspotRight"] = float(msg.carState.rightBlindspot)
      elif typ == 'controlsState':
        gtp.d["opEnabled"] = float(msg.controlsState.enabled)
        gtp.d["opActive"] = float(msg.controlsState.active)
      elif typ == 'lateralPlan':
        gtp.d["opDesCurvature"] = float(msg.lateralPlan.curvatures[1])
      elif typ == 'longitudinalPlan':
        gtp.d["opDesAccel"] = float(msg.longitudinalPlan.accels[1])
        gtp.d["opDesSpeed"] = float(msg.longitudinalPlan.speeds[1])
      elif typ == 'radarState':
        gtp.d["leadV"] = float(msg.radarState.leadOne.vLead)
        gtp.d["leadA"] = float(msg.radarState.leadOne.aLeadK)
        gtp.d["leadD"] = float(msg.radarState.leadOne.dRel)
      elif typ == 'gpsLocationExternal':
        gtp.d["lat"] = float(msg.gpsLocationExternal.latitude)
        gtp.d["lon"] = float(msg.gpsLocationExternal.longitude)
        gtp.d["elevation"] = float(msg.gpsLocationExternal.altitude)
        gtp.d["time"] = datetime.fromtimestamp(msg.gpsLocationExternal.timestamp / 1000)
        gtp.d["course"] = float(msg.gpsLocationExternal.bearingDeg)
        gtp.d["accuracy"] = float(msg.gpsLocationExternal.accuracy)
        gtp.d["source"] = str(msg.gpsLocationExternal.source)
      elif typ == 'ubloxGnss' and msg.ubloxGnss.which() == 'measurementReport':
        gtp.d["satellites"] = int(msg.ubloxGnss.measurementReport.numMeas)
      elif typ == 'liveLocationKalman' and gtp.d["speed"] is not None:
        gtp.d["accelLat"] = float(msg.liveLocationKalman.angularVelocityCalibrated.value[2] / max(0.01, gtp.d["speed"]))
      elif typ == 'driverMonitoringState':
        gtp.d["driverDistracted"] = bool(msg.driverMonitoringState.isDistracted)
        gtp.d["driverFaceDetected"] = bool(msg.driverMonitoringState.faceDetected)
      
      if gtp.is_valid():
        if i > 0:
          dt = (gtp.d["time"] - gpx_segment.points[-1].time).total_seconds()
          distance += gtp.d["speed"] * dt
          time += dt
          avg_speed = distance/max(0.1, time)
        gtp.d["distance"] = distance
        gtp.d["driveTime"] = time
        gtp.d["avgSpeed"] = avg_speed
        
        gpx_segment.points.append(gtp.to_gtp())
        gtp = GTP()
        i += 1
        if i % 1000 == 0:
          print(f"Added {i} points...")
        
  except StopIteration:
    print(f'Added {i} points')
  out_path = args.out_path
  if out_path == "":
    out_path = os.path.join(args.data_dir, args.route_name).replace('bz2','') + ".gpx"
  if len(gpx_segment.points) > 30:
    gpx_segment.points = gpx_segment.points[10:-10]
  with open(out_path, 'w') as f:
    f.write(gpx.to_xml())


if __name__ == "__main__":
  sys.exit(main(sys.argv[1:]))

#!/usr/bin/env python3
from __future__ import print_function
import math
import numpy as np
from cereal import car
from common.params import Params
from opendbc.can.parser import CANParser
from selfdrive.car.gm.values import DBC, CAR, CanBus
from selfdrive.config import Conversions as CV
from selfdrive.car.interfaces import RadarInterfaceBase

RADAR_HEADER_MSG = 1120
SLOT_1_MSG = RADAR_HEADER_MSG + 1
RADAR_NUM_SLOTS = 20

VOACC_NUM_SLOTS = 12
VOACC_MIN_RANGE = 145. # below this distance, defer to radar
VOACC_RADAR_TRACK_COMBINE_DIST = 10. # [m]
VOACC_RADAR_TRACK_COMBINE_VREL = 4. # [m/s]
VOACC_RADAR_TRACK_COMBINE_DIST_VREL_RATIO = VOACC_RADAR_TRACK_COMBINE_DIST / VOACC_RADAR_TRACK_COMBINE_VREL

# Actually it's 0x47f, but can parser only reports
# messages that are present in DBC
LAST_RADAR_MSG = RADAR_HEADER_MSG + RADAR_NUM_SLOTS

def create_radar_can_parser(car_fingerprint, voacc_enabled):
  if car_fingerprint not in (CAR.VOLT, CAR.VOLT18, CAR.MALIBU, CAR.HOLDEN_ASTRA, CAR.ACADIA, CAR.CADILLAC_ATS, CAR.ESCALADE, CAR.ESCALADE_ESV):
    return None
  
  # radar header and RADAR_NUM_SLOTS tracks, preserving the ordering of the old implementation
  signals = [
    ('FLRRNumValidTargets', 'F_LRR_Obj_Header', 0),
    ('FLRRSnsrBlckd', 'F_LRR_Obj_Header', 0),
    ('FLRRYawRtPlsblityFlt', 'F_LRR_Obj_Header', 0),
    ('FLRRHWFltPrsntInt', 'F_LRR_Obj_Header', 0),
    ('FLRRAntTngFltPrsnt', 'F_LRR_Obj_Header', 0),
    ('FLRRAlgnFltPrsnt', 'F_LRR_Obj_Header', 0),
    ('FLRRSnstvFltPrsntInt', 'F_LRR_Obj_Header', 0)
  ]
  signals += [('TrkRange', f'LRRObject{i:02}', 0.0) for i in range(1, RADAR_NUM_SLOTS+1)]
  signals += [('TrkRangeRate', f'LRRObject{i:02}', 0.0) for i in range(1, RADAR_NUM_SLOTS+1)]
  signals += [('TrkRangeAccel', f'LRRObject{i:02}', 0.0) for i in range(1, RADAR_NUM_SLOTS+1)]
  signals += [('TrkAzimuth', f'LRRObject{i:02}', 0.0) for i in range(1, RADAR_NUM_SLOTS+1)]
  signals += [('TrkObjectID', f'LRRObject{i:02}', 0.0) for i in range(1, RADAR_NUM_SLOTS+1)]
  
  if voacc_enabled:
    # VOACC header and VOACC_NUM_SLOTS tracks
    signals += [
      ('FVisionNumValidTrgts', 'F_Vision_Obj_Header', 0),
      ('FVsnSnsrBlckd', 'F_Vision_Obj_Header', 0),
      ('FrtVsnFld', 'F_Vision_Obj_Header', 0),
      ('FrtVsnUnvlbl', 'F_Vision_Obj_Header', 0),
      ('FrtVsnSrvAlgnInPrcs', 'F_Vision_Obj_Header', 0),
    ]
    signals += [(f'FwdVsnRngTrk{i}Rev', f'F_Vision_Obj_Track_{i}', 0.0) for i in range(1, VOACC_NUM_SLOTS+1)]
    signals += [(f'FwdVsnLongVlctyTrk{i}', f'F_Vision_Obj_Track_{i}_B', 0.0) for i in range(1, VOACC_NUM_SLOTS+1)]
    signals += [(f'FwdVsnLatOfstTrk{i}', f'F_Vision_Obj_Track_{i}_B', 0.0) for i in range(1, VOACC_NUM_SLOTS+1)]
    signals += [(f'FVisionObjectIDTrk{i}', f'F_Vision_Obj_Track_{i}', 0.0) for i in range(1, VOACC_NUM_SLOTS+1)]

  checks = list({(s[1], 14) for s in signals})

  return CANParser(DBC[car_fingerprint]['radar'], signals, checks, CanBus.OBSTACLE)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)


    self.extended_radar_enabled = Params().get_bool("ExtendedRadar")
    self.voacc_enabled = self.extended_radar_enabled and Params().get_bool("LongRangeLeadsEnabled")

    self.rcp = None if CP.radarOffCan else create_radar_can_parser(CP.carFingerprint, self.voacc_enabled)

    self.trigger_msg = LAST_RADAR_MSG
    self.updated_messages = set()
    self.radar_ts = CP.radarTimeStep

  def update(self, can_strings):
    if self.rcp is None:
      return super().update(None)

    vls = self.rcp.update_strings(can_strings)
    self.updated_messages.update(vls)

    if self.trigger_msg not in self.updated_messages:
      return None

    ret = car.RadarData.new_message()
    radar_header = self.rcp.vl[RADAR_HEADER_MSG]
    radar_fault = radar_header['FLRRSnsrBlckd'] or radar_header['FLRRSnstvFltPrsntInt'] or \
      radar_header['FLRRYawRtPlsblityFlt'] or radar_header['FLRRHWFltPrsntInt'] or \
      radar_header['FLRRAntTngFltPrsnt'] or radar_header['FLRRAlgnFltPrsnt']

    # for voacc data, if one or the other faults, it's ok
    if self.voacc_enabled:
      voacc_header = self.rcp.vl['F_Vision_Obj_Header']
      voacc_fault = voacc_header['FVsnSnsrBlckd'] or \
              voacc_header['FrtVsnFld'] or \
              voacc_header['FrtVsnUnvlbl']

    errors = []
    errorStrs = []
    if not self.rcp.can_valid:
      errors.append("canError")
      errorStrs.append("can error")
    if radar_fault and (not self.voacc_enabled or voacc_fault):
      errors.append("fault")
      errorStrs += [k for k in ['FLRRSnsrBlckd','FLRRSnstvFltPrsntInt','FLRRYawRtPlsblityFlt','FLRRHWFltPrsntInt','FLRRAntTngFltPrsnt','FLRRAlgnFltPrsnt'] if radar_header[k]]
      errorStrs += [k for k in ['FVsnSnsrBlckd','FrtVsnFld','FrtVsnUnvlbl'] if voacc_header[k]]
    ret.errors = errors
    ret.errorStrs = errorStrs

    currentTargets = set()
    if not radar_fault:
      num_targets = radar_header['FLRRNumValidTargets']

      # Not all radar messages describe targets,
      # no need to monitor all of the self.rcp.msgs_upd
      for ii in self.updated_messages:
        if ii == RADAR_HEADER_MSG:
          continue

        if num_targets == 0:
          break

        cpt = self.rcp.vl[ii]
        if 'TrkRange' not in cpt:
          continue
        
        # Zero distance means it's an empty target slot
        if cpt['TrkRange'] > 0.0:
          targetId = cpt['TrkObjectID']
          currentTargets.add(targetId)
          if targetId not in self.pts:
            self.pts[targetId] = car.RadarData.RadarPoint.new_message()
            self.pts[targetId].trackId = targetId
          distance = cpt['TrkRange']
          self.pts[targetId].dRel = distance  # from front of car
          # From driver's pov, left is positive
          self.pts[targetId].yRel = math.sin(cpt['TrkAzimuth'] * CV.DEG_TO_RAD) * distance
          self.pts[targetId].vRel = cpt['TrkRangeRate']
          self.pts[targetId].aRel = cpt['TrkRangeAccel']
          self.pts[targetId].yvRel = float('nan')
            
    if self.voacc_enabled and not voacc_fault:
      num_targets = voacc_header['FVisionNumValidTrgts']
      if num_targets > 0:
        for i in range(1,VOACC_NUM_SLOTS+1):
          cpt = self.rcp.vl[f'F_Vision_Obj_Track_{i}']
          cptb = self.rcp.vl[f'F_Vision_Obj_Track_{i}_B']
          distance = cpt[f'FwdVsnRngTrk{i}Rev']
          y_rel = cptb[f'FwdVsnLatOfstTrk{i}']
          v_rel = cptb[f'FwdVsnLongVlctyTrk{i}']
          # Zero distance means it's an empty target slot
          # if close enough to existing radar point, move that point to midpoint with this point
          if len(currentTargets) > 0:
            closestPt = min([self.pts[i] for i in currentTargets], 
                            key=lambda x:np.linalg.norm([x.dRel - distance, x.yRel - y_rel, (x.vRel - v_rel) * VOACC_RADAR_TRACK_COMBINE_DIST_VREL_RATIO]))
          else:
            closestPt = None
            
          if closestPt is not None and np.linalg.norm([closestPt.dRel - distance, closestPt.yRel - y_rel]) <= VOACC_RADAR_TRACK_COMBINE_DIST \
              and np.fabs(closestPt.vRel - v_rel) <= VOACC_RADAR_TRACK_COMBINE_VREL:
            if closestPt.dRel >= VOACC_MIN_RANGE:
              self.pts[closestPt.trackId].dRel = (distance + closestPt.dRel) / 2
              self.pts[closestPt.trackId].yRel = (y_rel + closestPt.yRel) / 2
              self.pts[closestPt.trackId].vRel = (v_rel + closestPt.vRel) / 2
          elif distance >= VOACC_MIN_RANGE or radar_fault:
            targetId = cpt[f'FVisionObjectIDTrk{i}']
            currentTargets.add(targetId)
            if targetId not in self.pts:
              self.pts[targetId] = car.RadarData.RadarPoint.new_message()
              self.pts[targetId].trackId = targetId
            self.pts[targetId].dRel = distance  # from front of car
            # From driver's pov, left is positive
            self.pts[targetId].yRel = y_rel
            self.pts[targetId].vRel = v_rel
            self.pts[targetId].aRel = float('nan')
            self.pts[targetId].yvRel = float('nan')

    for oldTarget in list(self.pts.keys()):
      if oldTarget not in currentTargets:
        del self.pts[oldTarget]

    ret.points = list(self.pts.values())
    self.updated_messages.clear()
    return ret

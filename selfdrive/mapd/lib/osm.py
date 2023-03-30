import overpy
import subprocess
import numpy as np
from cereal import log
from selfdrive.mapd.lib.geo import R
from selfdrive._c_mem import get, update



def create_way(way_id, node_ids, from_way):
  """
  Creates and OSM Way with the given `way_id` and list of `node_ids`, copying attributes and tags from `from_way`
  """
  return overpy.Way(way_id, node_ids=node_ids, attributes={}, result=from_way._result,
                    tags=from_way.tags)


class OSM:
  def __init__(self):
    self.api = overpy.Overpass()
    self._osm_local_db_enabled = get("OsmLocalDb", False)
    # self.api = overpy.Overpass(url='http://3.65.170.21/api/interpreter')

  def _online_query(self, q, area_q):
    print("Query OSM from remote server")
    query = self.api.query(q + area_q)
    areas, ways = query.areas, query.ways
    data_type = "online"
    return areas, ways, data_type

  def fetch_road_ways_around_location(self, lat, lon, radius):
    # Calculate the bounding box coordinates for the bbox containing the circle around location.
    bbox_angle = np.degrees(radius / R)
    # fetch all ways and nodes on this ways in bbox
    bbox_str = f'{str(lat - bbox_angle)},{str(lon - bbox_angle)},{str(lat + bbox_angle)},{str(lon + bbox_angle)}'
    lat_lon = "(%f,%f)" % (lat, lon)
    q = """
        way(""" + bbox_str + """)
          [highway]
          [highway!~"^(footway|path|corridor|bridleway|steps|cycleway|construction|bus_guideway|escape|service|track)$"];
        (._;>;);
        out;"""
    area_q = """is_in""" + lat_lon + """;area._[admin_level~"[24]"];
        convert area ::id = id(), admin_level = t['admin_level'],
        name = t['name'], "ISO3166-1:alpha2" = t['ISO3166-1:alpha2'];out;
        """
    try:
        areas, ways, data_type = self._online_query(q, area_q)
    except Exception as e:
      print(f'Exception while querying OSM:\n{e}')
      areas, ways, data_type = [], [], "error"
      raise e

    return areas, ways, data_type

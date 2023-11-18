import json
import time

from openpilot.common.params_pyx import Params
from openpilot.common.realtime import Ratekeeper

# PFEIFER - MAPD {{
mem_params = Params("/dev/shm/params")
params = Params()
# }} PFEIFER - MAPD


def request_refresh_osm_location_data(nations: [str], states: [str] = None):
  params.put("OsmDownloadedDate", str(time.time()))
  params.put_bool("OsmDbUpdatesCheck", False)

  osm_download_locations = json.dumps({
    "nations": nations,
    "states": states or []
  })

  print(f"Downloading maps for {osm_download_locations}")
  mem_params.put("OSMDownloadLocations", osm_download_locations)


def filter_nations_and_states(nations: [str], states: [str] = None):
  """Filters and prepares nation and state data for OSM map download.

  If the nation is 'US' and a specific state is provided, the nation 'US' is removed from the list.
  If the nation is 'US' and the state is 'All', the 'All' is removed from the list.
  The idea behind these filters is that if a specific state in the US is provided,
  there's no need to download map data for the entire US. Conversely,
  if the state is unspecified (i.e., 'All'), we intend to download map data for the whole US,
  and 'All' isn't a valid state name, so it's removed.

  Parameters:
  nations (list): A list of nations for which the map data is to be downloaded.
  states (list, optional): A list of states for which the map data is to be downloaded. Defaults to None.

  Returns:
  tuple: Two lists. The first list is filtered nations and the second list is filtered states.
  """

  if "US" in nations and states and not any(x.lower() == "all" for x in states):
    # If a specific state in the US is provided, remove 'US' from nations
    nations.remove("US")
  elif "US" in nations and states and any(x.lower() == "all" for x in states):
    # If 'All' is provided as a state (case invariant), remove those instances from states
    states = [x for x in states if x.lower() != "all"]
  elif "US" not in nations and states and any(x.lower() == "all" for x in states):
    states.remove("All")
  return nations, states or []


def update_osm_db():
  # last_downloaded_date = float(params.get('OsmDownloadedDate', encoding='utf-8') or 0.0)
  # if params.get_bool("OsmDbUpdatesCheck") or time.time() - last_downloaded_date >= 604800:  # 7 days * 24 hours/day * 60
  if params.get_bool("OsmDbUpdatesCheck"):
    country = params.get('OsmLocationName', encoding='utf-8')
    state = params.get('OsmStateName', encoding='utf-8') or "All"
    filtered_nations, filtered_states = filter_nations_and_states([country], [state])
    request_refresh_osm_location_data(filtered_nations, filtered_states)

  if not mem_params.get("OSMDownloadBounds"):
    mem_params.put("OSMDownloadBounds", "")

  if not mem_params.get("LastGPSPosition"):
    mem_params.put("LastGPSPosition", "{}")


def main_thread(sm=None, pm=None):
  rk = Ratekeeper(1, print_delay_threshold=None)

  while True:
    update_osm_db()
    rk.keep_time()


def main():
  main_thread()


if __name__ == "__main__":
  main()

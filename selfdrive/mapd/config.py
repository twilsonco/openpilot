# Map query config

QUERY_RADIUS = 9000  # mts. Radius to use on OSM data queries.
MIN_DISTANCE_FOR_NEW_QUERY = 3500  # mts. Minimum distance to query area edge before issuing a new query.
FULL_STOP_MAX_SPEED = 1.  # m/s Max speed for considering car is stopped.
LOOK_AHEAD_HORIZON_TIME = 8.  # s. Time horizon for look ahead of turn speed sections to provide on liveMapData msg.
LANE_WIDTH = 3.7  # Lane width estimate. Used for detecting departures from way.

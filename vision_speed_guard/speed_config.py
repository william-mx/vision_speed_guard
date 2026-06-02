# speed_config.py — edit this file to tune signs and thresholds
# ---------------------------------------------------------------
# Students: all the numbers you'll want to tweak live here.
# The rest of the code (speed_targets.py, speed_guard.py) stays fixed.

from .speed_controllers import Target, StopSign, Vehicle, DriveController

KMH = lambda v: round(v / 3.6, 1)  # km/h → m/s

DEFAULT_SPEED   = 1.1   # m/s — normal cruising speed
ATTENTION_SPEED = 0.8   # m/s — cautious speed near crosswalks etc.

# Min bbox height (px) a sign must reach before we react — acts as a distance gate
DEFAULT_MIN_HEIGHT = 45.0
VEHICLE_MIN_HEIGHT = 150.0   # start slowing down
VEHICLE_MAX_HEIGHT = 300.0   # full stop

# Signs that set a persistent speed limit (lowest active one wins)
SPEED_SIGNS  = ['crosswalk_ahead', 'end_speed_limit', 'speed_limit_5']

# Signs that temporarily override the limit (stop, red light, …)
ACTION_SIGNS = ['no_entry', 'stop_sign', 'traffic_light_red', 'traffic_light_yellow']

TARGETS = {
    'crosswalk_ahead':          Target('crosswalk_ahead',           ATTENTION_SPEED, min_height=DEFAULT_MIN_HEIGHT),
    'end_speed_limit':          Target('end_speed_limit',           DEFAULT_SPEED, min_height=DEFAULT_MIN_HEIGHT),
    'speed_limit_5':            Target('speed_limit_5',             KMH(5), min_height=DEFAULT_MIN_HEIGHT),
    'no_entry':                 Target('no_entry',                  0.0, min_height=DEFAULT_MIN_HEIGHT),
    'traffic_light_red':        Target('traffic_light_red',         0.0, threshold=0.5, len_history=5, min_height=DEFAULT_MIN_HEIGHT),
    'traffic_light_red_yellow': Target('traffic_light_red_yellow',  0.0, threshold=0.5, len_history=5, min_height=DEFAULT_MIN_HEIGHT),
    'traffic_light_yellow':     Target('traffic_light_yellow',      KMH(2), min_height=0.0),
    'stop_sign':                StopSign('stop_sign'),
    'vehicle':                  Vehicle('vehicle',                  min_height=VEHICLE_MIN_HEIGHT, max_height=VEHICLE_MAX_HEIGHT),
}

# Build the controller from config above — this is what speed_guard.py imports.
controller = DriveController(
    targets=TARGETS,
    speed_signs=SPEED_SIGNS,
    action_signs=ACTION_SIGNS,
    default_speed=DEFAULT_SPEED,
)
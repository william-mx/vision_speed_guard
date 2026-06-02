from collections import deque
import numpy as np

# ---------------------------------------------------------------------------
# Speed helpers & config
# ---------------------------------------------------------------------------

KMH = lambda v: round(v / 3.6, 1)   # Convert km/h to m/s

DEFAULT_SPEED   = 1.1   # m/s — normal driving speed
ATTENTION_SPEED = 0.8   # m/s — slow down near hazards (e.g. crosswalk)

# Signs that set a new speed limit (lower bound is kept across frames)
SPEED_SIGNS = ['crosswalk_ahead', 'end_speed_limit', 'speed_limit_5']

# Signs that temporarily override the speed limit (stop, red light, …)
ACTION_SIGNS = ['no_entry', 'stop_sign', 'traffic_light_red', 'traffic_light_yellow']

# ---------------------------------------------------------------------------
# Target classes — one per sign / object type
# ---------------------------------------------------------------------------

class Target:
    """
    Base class for a detectable sign the vehicle reacts to.

    History smoothing: a sign is considered 'visible' only when it appears
    in more than `threshold` fraction of the last `len_history` frames.
    This prevents false positives from a single noisy detection.
    """

    def __init__(self, label, set_speed,
                 threshold=0.3, len_history=10, min_height=50.0):
        self.label      = label
        self.set_speed  = set_speed       # target speed when reacting
        self.threshold  = threshold       # fraction of history needed to be 'visible'
        self.min_height = min_height      # bbox height (px) needed to be 'in range'
        self.history    = deque([False] * len_history, maxlen=len_history)
        self.detection  = None
        self.has_reacted = False

    @property
    def visible(self):
        """True when the sign appears consistently enough to trust."""
        return np.mean(self.history) > self.threshold

    @property
    def in_range(self):
        """True when the sign is close enough to act on (big bbox)."""
        return self.detection is not None and self.detection.height >= self.min_height

    def update(self, detection, current_limit):
        """
        Call every frame with the latest detection (or None if not detected).
        Returns the desired speed, or np.nan if this sign has no opinion.
        """
        self.detection = detection
        self.history.append(detection is not None)

        if detection is None:
            if not self.visible and self.has_reacted:
                print(f"[{self.label}] No longer visible — resetting.")
                self.has_reacted = False
            return np.nan

        if self.visible and self.in_range:
            if not self.has_reacted:
                self.has_reacted = True
                print(f"{detection.seq}: [{self.label}] Reacting! {detection}")
            return self.set_speed

        if not self.visible and self.has_reacted:
            print(f"{detection.seq}: [{self.label}] No longer visible — resetting.")
            self.has_reacted = False

        return np.nan


class StopSign(Target):
    """
    Stops the vehicle and holds for `duration` seconds before resuming.
    Uses a higher detection threshold (0.8) to avoid phantom stops.
    """

    def __init__(self, label, threshold=0.8, min_height=50.0,
                 set_speed=0.0, duration=3.0):
        super().__init__(label, set_speed, threshold=threshold, min_height=min_height)
        self.duration      = duration
        self.start_time_ns = None

    def update(self, detection, current_limit):
        self.detection = detection
        self.history.append(detection is not None)

        # Trigger stop on first confirmed sighting
        if detection is not None and self.visible and self.in_range and not self.has_reacted:
            self.has_reacted   = True
            self.start_time_ns = detection.timestamp
            print(f"{detection.seq}: [{self.label}] Reacting! {detection}")
            return self.set_speed

        # Hold the stop for the required duration
        if self.has_reacted:
            if detection is not None:
                elapsed = (detection.timestamp - self.start_time_ns) / 1e9
                if elapsed <= self.duration:
                    return self.set_speed
                # Duration elapsed — resume
                self.has_reacted = self.start_time_ns = None
                return current_limit

            if not self.visible:
                print(f"[{self.label}] No longer visible — resetting.")
                self.has_reacted = self.start_time_ns = None

        return np.nan


# ---------------------------------------------------------------------------
# Sign registry — maps label → Target instance
# ---------------------------------------------------------------------------

TARGETS = {
    'crosswalk_ahead':      Target('crosswalk_ahead',      set_speed=ATTENTION_SPEED),
    'end_speed_limit':      Target('end_speed_limit',      set_speed=DEFAULT_SPEED),
    'speed_limit_5':        Target('speed_limit_5',        set_speed=KMH(5)),
    'no_entry':             Target('no_entry',             set_speed=0.0),
    'traffic_light_red':    Target('traffic_light_red',    set_speed=0.0,      min_height=130.0),
    'traffic_light_yellow': Target('traffic_light_yellow', set_speed=KMH(2),   min_height=0.0),
    'stop_sign':            StopSign('stop_sign'),
}

# ---------------------------------------------------------------------------
# DriveController — single entry point for the ROS node
# ---------------------------------------------------------------------------

class DriveController:
    """
    Combines speed-sign limits and action-sign overrides into one speed value.

    Usage (each frame):
        speed = controller.update(list_of_detections)
    """

    def __init__(self):
        self.speed_limit = DEFAULT_SPEED   # regulatory limit (updated by speed signs)
        self.drive_limit = np.nan          # transient override (stop, red light, …)

    def update(self, detections):
        """
        detections : list of Detection objects for the current frame.
        Returns    : float — the speed the vehicle should drive at.
        """
        by_label = {d.label: d for d in detections}

        # 1. Speed signs set the regulatory limit (take the lowest active one)
        speed_votes = [TARGETS[l].update(by_label.get(l), self.speed_limit)
                       for l in SPEED_SIGNS]
        if not np.isnan(speed_votes).all():
            self.speed_limit = np.nanmin(speed_votes)

        # 2. Action signs can temporarily override the limit (stop, red light, …)
        action_votes = [TARGETS[l].update(by_label.get(l), self.speed_limit)
                        for l in ACTION_SIGNS]
        self.drive_limit = np.nan if np.isnan(action_votes).all() else np.nanmin(action_votes)

        # 3. Final speed = most restrictive of the two
        return float(np.nanmin([self.speed_limit, self.drive_limit]))
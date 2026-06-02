# speed_targets.py — Target classes and DriveController
# ---------------------------------------------------------------
# Students: read this to understand how each sign type behaves.
# To change thresholds / speeds, edit speed_config.py instead.

from collections import deque
import numpy as np


class Target:
    """
    Base class for any detectable sign the vehicle reacts to.

    History smoothing: we keep a short boolean history of recent frames.
    A sign is 'visible' only when it appears in > `threshold` of them —
    this filters out single-frame false positives.
    """

    def __init__(self, label, set_speed, threshold=0.3, len_history=10, min_height=45.0):
        self.label       = label
        self.set_speed   = set_speed     # speed to command when reacting
        self.threshold   = threshold     # visibility threshold (fraction of history)
        self.min_height  = min_height    # min bbox height (px) to be considered 'in range'
        self.history     = deque([False] * len_history, maxlen=len_history)
        self.detection   = None
        self.has_reacted = False

    @property
    def visible(self):
        """True when the sign appears consistently across recent frames."""
        return np.mean(self.history) > self.threshold

    @property
    def in_range(self):
        """True when the bbox is large enough — i.e. the sign is close."""
        return self.detection is not None and self.detection.height >= self.min_height

    def update(self, detection, current_limit):
        """Called every frame. Returns desired speed, or np.nan = no opinion."""
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
    Stops the vehicle, holds for `duration` seconds, then resumes.
    Uses a stricter threshold (0.8) to avoid phantom stops.
    """

    def __init__(self, label, threshold=0.8, min_height=45.0, set_speed=0.0, duration=3.0):
        super().__init__(label, set_speed, threshold=threshold, min_height=min_height)
        self.duration      = duration
        self.start_time_ns = None

    def update(self, detection, current_limit):
        self.detection = detection
        self.history.append(detection is not None)

        # First confirmed sighting → start the stop timer
        if detection is not None and self.visible and self.in_range and not self.has_reacted:
            self.has_reacted = True
            self.start_time_s = detection.timestamp
            print(f"{detection.seq}: [{self.label}] Reacting! {detection}")
            return self.set_speed

        if self.has_reacted:
            if detection is not None:
                elapsed = detection.timestamp - self.start_time_s
                if elapsed <= self.duration:
                    return self.set_speed          # still holding
                return current_limit               # timer done — resume

            if not self.visible:
                print(f"[{self.label}] No longer visible — resetting.")
                self.has_reacted = self.start_time_s = None

        return np.nan


class Vehicle(Target):
    """
    Leading vehicle — scales speed linearly with bbox height.

    bbox height acts as a proxy for distance:
      height < min_height  →  far away, keep current speed
      min_height … max_height  →  getting closer, slow down proportionally
      height >= max_height  →  too close, stop (speed = 0)
    """

    def __init__(self, label, threshold=0.5, len_history=10,
                 min_height=150.0, max_height=300.0):
        super().__init__(label, set_speed=None, threshold=threshold,
                         len_history=len_history, min_height=min_height)
        self.max_height = max_height

    def update(self, detection, current_limit):
        self.detection = detection
        self.history.append(detection is not None)

        if detection is None or not self.visible:
            return np.nan

        h = detection.height
        if h >= self.max_height:
            speed = 0.0                                                    # stop — too close
        elif h >= self.min_height:
            ratio = (h - self.min_height) / (self.max_height - self.min_height)
            speed = current_limit * (1.0 - ratio)                         # slow down gradually
        else:
            speed = current_limit                                          # far enough — no change

        print(f"[{self.label}] height={h:.0f}px  "
              f"[{self.min_height:.0f}…{self.max_height:.0f}]  speed={speed:.2f}")
        return speed


class DriveController:
    """
    Merges speed-sign limits and action-sign overrides into one speed output.

    Speed signs  — set a persistent regulatory limit (lowest active one wins).
    Action signs — temporarily override that limit (stop, red light, vehicle).

    Usage each frame:
        speed = controller.update(list_of_detections)
    """

    def __init__(self, targets, speed_signs, action_signs, default_speed):
        self.targets      = targets
        self.speed_signs  = speed_signs
        self.action_signs = action_signs
        self.speed_limit  = default_speed   # regulatory limit, updated by speed signs
        self.drive_limit  = np.nan          # transient override

    def update(self, detections):
        by_label = {d.label: d for d in detections}

        # 1. Speed signs → update regulatory limit
        speed_votes = [self.targets[l].update(by_label.get(l), self.speed_limit)
                       for l in self.speed_signs]
        if not np.isnan(speed_votes).all():
            self.speed_limit = np.nanmin(speed_votes)

        # 2. Action signs → compute transient override
        action_votes = [self.targets[l].update(by_label.get(l), self.speed_limit)
                        for l in self.action_signs]
        self.drive_limit = np.nan if np.isnan(action_votes).all() else np.nanmin(action_votes)

        # 3. Final speed = most restrictive of the two
        return float(np.nanmin([self.speed_limit, self.drive_limit]))
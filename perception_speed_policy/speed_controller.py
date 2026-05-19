import numpy as np

KMH = lambda v: round(v / 3.6, 1)  # km/h → m/s
DEFAULT_SPEED   = 1.1  # cruising speed
ATTENTION_SPEED = 0.8  # caution zones (e.g. crosswalks)

class SpeedController:
    def __init__(self):

        # Latched speed limits
        self.speed_limit = DEFAULT_SPEED  # persistent regulatory limit
        self.drive_limit = np.nan         # temporary/reactive limit

    def update(self, detections):

        for d in detections: # list[Detection2DResult]

            # Latch new speed limit when sign is detected
            if d.label == "speed_limit_5" and d.score > 0.0:
                self.speed_limit = KMH(5)
        
        return self.speed_limit
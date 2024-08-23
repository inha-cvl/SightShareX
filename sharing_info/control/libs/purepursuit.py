import numpy as np
import math
import configparser


MPS_TO_KPH = 3.6

class PurePursuit(object):
    def __init__(self):
        self.set_configs()
        self.prev_steer = 0

    def set_configs(self):
        self.lfd_gain = 0.55
        self.min_lfd = 8
        self.max_lfd = 25
        self.wheelbase = 2.72
        self.steer_ratio = 12.9
        self.steer_max = 45
        self.saturation_th = 3

    def execute(self, current_location, state, local_path, current_heading, current_velocity):
        if len(current_location) < 1 or state < 1:
            return 0, [0,0]
        
        lfd = self.lfd_gain * current_velocity * MPS_TO_KPH
        lfd = np.clip(lfd, self.min_lfd, self.max_lfd)

        point = current_location
        route = local_path
        heading = math.radians(current_heading)
        
        steering_angle = 0.
        lh_point = []
        change = False
        for path_point in route:
            diff = path_point - point
            rotated_diff = diff.rotate(-heading)
            if rotated_diff.x > 0:
                dis = rotated_diff.distance()
                if dis >= lfd:
                    theta = rotated_diff.angle
                    steering_angle = np.arctan2(2*self.wheelbase*np.sin(theta), lfd)
                    lh_point = path_point
                    change = True
                    break
        # if not change:
        #     theta = rotated_diff.angle
        #     steering_angle = np.arctan2(2*self.wheelbase*np.sin(theta), lfd)
        #     lh_point = path_point

        steering_angle = math.degrees(steering_angle)
        if current_velocity * MPS_TO_KPH > 30:
            steering_angle = steering_angle * 1.35
        return math.radians(steering_angle), [lh_point.x, lh_point.y]

    def saturate_steering_angle(self, now):
        saturated_steering_angle = now
        diff = abs(self.prev_steer-now)
        if diff > self.saturation_th:
            if now>=0: 
                saturated_steering_angle = self.prev_steer+self.saturation_th
            else: 
                saturated_steering_angle = self.prev_steer-self.saturation_th
        self.prev_steer = saturated_steering_angle
        return saturated_steering_angle


import planning.libs.planning_helper as phelper
import copy
import math

KPH_TO_MPS = 1 / 3.6
MPS_TO_KPH = 3.6

class LocalPathPlanner:
    def __init__(self, map):
        self.MAP = map
        self.setting_values()
    
    def setting_values(self):
        self.phelper = phelper
        self.phelper.lanelets = self.MAP.lanelets
        self.phelper.tiles = self.MAP.tiles
        self.phelper.tile_size = self.MAP.tile_size

        self.local_pose = None
        self.current_velocity = 0.0
        self.current_signal = 0

        self.local_path = None
        self.local_kappa = None
        self.temp_signal = 0
        self.threshold_gap = 2.5

        self.t_reaction_change = 2
        self.minimum_distance = 30
        self.d_lane = 3.5
        self.velocity_range = [0, 80]
        self.theta_range = [15, 20]
        self.L = 4.75
        self.max_path_len = 30

    
    def update_value(self, local_pose, velocity, signal):
        self.local_pose = local_pose
        self.current_velocity = velocity
        self.current_signal = signal
    
    def current_lane_waypoints(self, local_pose): 
        idnidx = self.phelper.lanelet_matching(local_pose)
        if idnidx is None:
            return [], 1
        else:
            l_id, _ = idnidx
            lane_number = self.phelper.lanelets[l_id]['laneNo']
            curr_lane_waypoints = self.phelper.lanelets[l_id]['waypoints']
            curr_lane_waypoints = curr_lane_waypoints[:200] if len(curr_lane_waypoints) > 200 else curr_lane_waypoints
            return curr_lane_waypoints, lane_number
        

    def need_update(self):
        if self.local_path == None:
            return 0, -1
        idx = self.phelper.find_nearest_idx(self.local_path, self.local_pose)
        if self.temp_signal != self.current_signal and self.current_signal != 0 and self.current_signal <= 3:
            self.temp_signal = self.current_signal
            return 2, idx
        threshold = 15#(self.current_velocity*MPS_TO_KPH)*self.threshold_gap
        if len(self.local_path)-idx <= threshold:
            return 1, idx
        else:
            return -1, idx

    def get_change_path(self, sni,  path_len, to=1):
        wps, uni = self.phelper.get_straight_path(sni, path_len)
        c_pt = wps[-1]
        l_id, r_id = self.phelper.get_neighbor(uni[0])
        n_id = l_id if to==1 else r_id
        if n_id != None:
            r = self.MAP.lanelets[n_id]['waypoints']
            u_n = n_id
            u_i = self.phelper.find_nearest_idx(r, c_pt)
            uni = [u_n, u_i]
        else:
            r = wps
        return r, uni
    
    def make_path(self, update_type, local_pos):
        tr0 = []
        if update_type == 0 or update_type == 2 or update_type == 3:
            start_pose = local_pos
        else:
            start_pose = self.local_path[-1]
            idx = self.phelper.find_nearest_idx(self.local_path, local_pos)
            tr0 = self.local_path[idx:]        
                
        idnidx0 = self.phelper.lanelet_matching(start_pose)
        if idnidx0 == None:
            return None
        l_buffer_change = max(0, self.minimum_distance - (self.current_velocity * self.t_reaction_change))
        l_tr1 = self.current_velocity*self.t_reaction_change + l_buffer_change
        tr1, idnidx1 = self.phelper.get_straight_path(idnidx0, l_tr1)
        if self.temp_signal == 0 or self.temp_signal == 3 or update_type == 3 or update_type == 1:
            local_path = tr0 + tr1
        else:
            theta = self.theta_range[1] - ((self.current_velocity-self.velocity_range[0])/(self.velocity_range[1]-self.velocity_range[0])) * (self.theta_range[1]-self.theta_range[0])
            l_tr2 = self.d_lane / math.sin(theta)
            tr2, idnidx2 = self.get_change_path(idnidx1, l_tr2, self.temp_signal)
            l_tr3 = (self.L/2) * self.current_velocity if self.current_velocity > 0 else self.L * 3 
            tr3, _ = self.phelper.get_straight_path(idnidx2, l_tr3)
            local_path = tr0+tr1+tr3

        return local_path

    

    def execute(self):
        if self.local_pose is None or self.local_pose[0] == 'inf':
            return None
        local_path = []
        local_kappa = []
        need_update, idx = self.need_update()
        if need_update != -1:
            local_path = self.make_path(need_update, self.local_pose)
            if local_path == None or len(local_path) <= 0:
                return
            interp_path, local_kappa = self.phelper.interpolate_path(local_path)
            self.local_path = self.phelper.limit_path_length(interp_path, self.max_path_len)
            self.local_kappa = local_kappa
        return self.local_path, self.local_kappa


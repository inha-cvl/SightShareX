

import planning.libs.planning_helper as phelper
import copy

KPH_TO_MPS = 1 / 3.6
MPS_TO_KPH = 3.6

class LocalPathPlanner:
    def __init__(self, map):
        self.MAP = map
        self.setting_values()
    
    def setting_values(self):
        phelper.lanelets = self.MAP.lanelets
        phelper.tiles = self.MAP.tiles
        phelper.tile_size = self.MAP.tile_size

        self.local_pose = None
        self.current_velocity = 0.0
        self.current_signal = 0

        self.precision = 1
        self.local_path = None
        self.temp_signal = 0
        self.threshold_gap = 2.5
        self.x_p = 3
        self.x_c = 50
        self.x2_i = 30
        self.x2_v_th = 9
        self.x3_c = 8
    
    def update_value(self, local_pose, velocity, signal):
        self.local_pose = local_pose
        self.current_velocity = velocity
        self.current_signal = signal

    def need_update(self):
        if self.local_path == None:
            return 0, -1

        idx = phelper.find_nearest_idx(self.local_path, self.local_pose)
        if self.temp_signal != self.current_signal and self.current_signal != 0:
            self.temp_signal = self.current_signal
            return 2, idx
        threshold = (self.current_velocity*MPS_TO_KPH)*self.threshold_gap
        if len(self.local_path)-idx <= threshold:
            return 1, idx
        else:
            return -1, idx

    def get_change_path(self, sni,  path_len, to=1):
        wps, uni = phelper.get_straight_path( sni, path_len)
        c_pt = wps[-1]
        l_id, r_id = phelper.get_neighbor(uni[0])
        n_id = l_id if to==1 else r_id

        if n_id != None:
            r = self.MAP.lanelets[n_id]['waypoints']
            u_n = n_id
            u_i = phelper.find_nearest_idx(r, c_pt)
            uni = [u_n, u_i]
        else:
            r = wps
        return r, uni
    
    def make_path(self, update_type, local_pos):
        r0 = []
        if update_type == 0 or update_type == 2 or update_type == 3:
            start_pose = local_pos
        else:
            start_pose = self.local_path[-1]
            idx = phelper.find_nearest_idx(self.local_path, local_pos)
            r0 = self.local_path[idx:]        
       
        ego_ni = phelper.lanelet_matching(start_pose)
        if ego_ni == None:
            return None
        
        if self.current_velocity < 0.5:
            x1 = self.x_c
        else:
            x1 = self.x_c-20 if self.current_velocity < self.x2_v_th else self.current_velocity * MPS_TO_KPH

        r1, ni1 = phelper.get_straight_path(ego_ni, x1)

        if self.current_signal == 0 or self.current_signal == 4 or update_type == 3 or update_type == 1:
            local_path = r0+r1
        else:
            x2 = self.x2_i if self.current_velocity < self.x2_v_th else self.current_velocity * self.x_p
            _, ni2 = self.get_change_path(ni1, x2, self.current_signal)
            x3 = self.x3_c + self.current_velocity * self.x_p
            r3, _ = phelper.get_straight_path(ni2, x3)
            local_path = r0+r1+r3

        return local_path

    def calc_kappa(self, idx):
        copy_local_path = copy.deepcopy(self.local_path)
        copy_local_path.insert(0, self.local_path[0])
        copy_local_path.append(self.local_path[-1])
        local_kappa = []
        for i, f in enumerate(self.local_path):
            before_after_pts = [copy_local_path[i], copy_local_path[i+2]]
            Rk = phelper.calc_kappa(f, before_after_pts)
            local_kappa.append(Rk)
        return local_kappa

    def execute(self):
        if self.local_pose == None:
            return None
        local_path = []
        need_update, idx = self.need_update()
        if need_update != -1:
            local_path = self.make_path(need_update, self.local_pose)
            if local_path == None or len(local_path) <= 0:
                return
            local_path = phelper.smooth_interpolate(local_path, self.precision)
            self.local_path = phelper.limit_path_length(local_path, 30)

        self.local_kappa = self.calc_kappa(idx)
        return self.local_path, self.local_kappa

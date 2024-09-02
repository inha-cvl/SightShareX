import numpy as np
import math


class ObstacleHandler:
    def __init__(self, phelper):
        self.setting_values(phelper)

    def setting_values(self, phelper):
        self.phelper = phelper
        self.local_pose = None
        self.prev_local_pose = None
        self.current_heading = 0.0

    def update_value(self, local_pose, heading):
        self.local_pose = local_pose
        self.current_heading = heading
        if self.prev_local_pose is None:
            self.prev_local_pose = self.local_pose
    
    def check_dimension(self, dimension):
        if dimension[0] > 2 and dimension[1] > 1.5 :
            return True
        else:
            return False

    def object2enu(self,  obs_pose):
        rad = np.radians(self.current_heading)

        nx = math.cos(rad) * obs_pose[0] - math.sin(rad) * obs_pose[1]
        ny = math.sin(rad) * obs_pose[0] + math.cos(rad) * obs_pose[1]

        obj_x = self.local_pose[0] + nx
        obj_y = self.local_pose[1] + ny

        return obj_x, obj_y
    
    def filtering_by_lane_num(self, lane_num, fred_d):
        if lane_num == 2:
            if -4.15 < fred_d < 1.45:
                return True
            else:
                return False
        elif lane_num == 3:
            if -7.15 < fred_d < 4.15:
                return True
            else:
                return False
        else:
            if -1.45< fred_d < 4.15:
                return True
            else:
                return False

    
    def filtering_by_frenet(self,fred_d):
        if -4.15 < fred_d < 1.5:
            return True
        else:
            return False
            
    def distance(self, x1, y1, x2, y2):
        return np.sqrt((x2-x1)**2+(y2-y1)**2    )

    def object2frenet(self, local_waypoints, obj_enu):
        if len(local_waypoints) > 0:  
            centerline = np.array(local_waypoints)
            point = np.array(obj_enu)

            tangents = np.gradient(centerline, axis=0)
            tangents = tangents / np.linalg.norm(tangents, axis=1)[:, np.newaxis]
            
            normals = np.column_stack([-tangents[:, 1], tangents[:, 0]])
            
            distances = np.linalg.norm(centerline - point, axis=1)
            
            closest_index = np.argmin(distances)
            closest_point = centerline[closest_index]
            tangent = tangents[closest_index]
            normal = normals[closest_index]
            
            vector_to_point = point - closest_point
            d = np.dot(vector_to_point, normal)
            s = np.sum(np.linalg.norm(np.diff(centerline[:closest_index + 1], axis=0), axis=0))
            
            return s, d
        else:
            return None
    
    def refine_heading_by_lane(self, obs_pos):
        idnidx = self.phelper.lanelet_matching(obs_pos)
        if idnidx is not None:
            waypoints = self.phelper.lanelets[idnidx[0]]['waypoints']
            next_idx = idnidx[1]+3 if idnidx[1]+3 < len(waypoints)-4 else len(waypoints)-4
            
            prev_point = waypoints[idnidx[1]]
            next_point = waypoints[next_idx]

            delta_x = next_point[0] - prev_point[0]
            delta_y = next_point[1] - prev_point[1]
            
            heading = math.degrees(math.atan2(delta_y, delta_x))

            return heading
        else:
            return None
import numpy as np
import quaternion


class WaypointList():
    def __init__(self, wp_yaml):
        self.waypoints = []
        for point in wp_yaml["waypoints"]:
            wp = {}
            for key, val in point["point"].items():
                wp[key] = val
            self.append(wp)
        
        self.point_keys = self.waypoints[0].keys()
        self.number_dict = {}
        return
    

    def append(self, waypoint: dict, id=None):
        self.waypoints.append(waypoint)
        if id: self.set_id(len(self.waypoints), id)
        return len(self.waypoints)
    

    def insert(self, num, waypoint: dict, id=None):
        self.waypoints.insert(num-1, waypoint)
        for i, n in self.number_dict.items():
            self.number_dict[i] = n+1 if (n>=num) else n
        if id: self.set_id(num, id)
        return
    

    def remove(self, id):
        num = self.get_num(str(id))
        self.waypoints.pop(num-1)
        self.number_dict.pop(str(id))
        for i, n in self.number_dict.items():
            self.number_dict[i] = n-1 if (n>num) else n
        return
    

    def set_waypoint_val(self, id, key, val):
        self.waypoints[self.get_num(id)-1][key] = val
        return
    

    def delete_waypoint_param(self, id, key):
        return self.waypoints[self.get_num(id)-1].pop(key)


    def get_waypoint(self, id=None, num=None):
        if id: return self.waypoints[self.get_num(str(id)) - 1]
        if num: return self.waypoints[num-1]
        return self.waypoints
    

    def get_id_list(self):
        return self.number_dict.keys()
    

    def set_id(self, num, id):
        self.number_dict[str(id)] = num
        return
    

    def get_num(self, id):
        return self.number_dict[str(id)]
    



class FinishPose():
    def __init__(self, wp_yaml):
        self.header = {}
        for key, val in wp_yaml["finish_pose"]["header"].items():
            self.header[key] = val
        
        self.position = {}
        for key, val in wp_yaml["finish_pose"]["pose"]["position"].items():
            self.position[key] = val
        
        self.orientation = {}
        for key, val in wp_yaml["finish_pose"]["pose"]["orientation"].items():
            self.orientation[key] = val
        
        self.x = self.position["x"]
        self.y = self.position["y"]
        self.yaw = self.get_euler()
        self.id = None
        return
    

    def get_euler(self):
        o = self.orientation
        q = np.quaternion(o["x"], o["y"], o["z"], o["w"])
        return quaternion.as_euler_angles(q)[1]




def get_waypoint_yaml(waypoints: WaypointList, finish_pose: FinishPose):
    s = ["waypoints:" + "\n"]
    for point in waypoints.get_waypoint():
        s.append("- point: {")
        for i, (key, val) in enumerate(point.items()):
            if (i != 0): s.append(", ")
            s.append(key + ": " + str(val).lower())
        s.append("}" + "\n")
    
    s.append("finish_pose:" + "\n")
    seq, stamp, frame = (finish_pose.header["seq"], finish_pose.header["stamp"], finish_pose.header["frame_id"])
    s.append("  header: {" + "seq: {}, stamp: {}, frame_id: {}".format(seq,stamp,frame) + "}" + "\n")
    s.append("  pose:" + "\n")
    x = finish_pose.x
    y = finish_pose.y
    z = finish_pose.position["z"]
    s.append("    position: {" + "x: {}, y: {}, z: {}".format(x, y, z) + "}" + "\n")
    q = quaternion.from_euler_angles([0, 0, finish_pose.yaw])
    s.append("    orientation: {" + "x: {}, y: {}, z: {}, w: {}".format(q.x, q.y, q.z, q.w) + "}")
    return "".join(s)

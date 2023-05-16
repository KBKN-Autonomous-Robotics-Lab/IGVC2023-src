import tkinter as tk
import numpy as np
import math
import ruamel.yaml
from PIL import Image, ImageTk
from pathlib import Path
from .waypointlib import WaypointList, FinishPose



def read_file(path: Path):
    with open(path) as file:   # .yamlを読み込む
        yaml = ruamel.yaml.YAML().load(file)
    return yaml



class MyMap():

    def __init__(self, path: Path, map_yaml):
        self.yaml_path = path.resolve()
        if map_yaml["image"][0] == "/":
            self.img_path = map_yaml["image"]
        else:
            self.img_path = self.yaml_path.with_name(map_yaml["image"]).resolve()
        self.original_img_pil = Image.open(self.img_path).convert("RGBA")
        self.pil_img = self.original_img_pil.copy()
        self.tk_img = ImageTk.PhotoImage(self.pil_img)
        self.origin = map_yaml["origin"]
        self.resolution = map_yaml["resolution"]
        self.map_yaml = map_yaml
        self.mat_affine = np.eye(3)
        self.img_origin = [-self.origin[0]/self.resolution, self.original_img_pil.height+self.origin[1]/self.resolution]
        return
    

    def translate(self, x, y):
        self.affine(x, y, 1)
        return

    
    def scale_at(self, x, y, scale):
        self.affine(-x, -y, 1)
        self.affine(0, 0, scale)
        self.affine(x, y, 1)
        return
    

    def rotate(self, theta, canv_center=None):
        if not canv_center:
            cx, cy = self.original_img_pil.width/2, self.original_img_pil.height/2
            canv_center = np.dot(self.mat_affine, [cx, cy, 1])
        self.translate(-canv_center[0], -canv_center[1])
        mat = [[math.cos(theta), -math.sin(theta), 0],
               [math.sin(theta),  math.cos(theta), 0],
               [0,                0,               1]]
        self.mat_affine = (np.dot(mat, self.mat_affine))
        self.translate(canv_center[0], canv_center[1])
        return

    
    def affine(self, x, y, scale):
        x = float(x)
        y = float(y)
        scale = float(scale)
        mat = [[scale,0,x], [0,scale,y], [0,0,1]]
        self.mat_affine = np.dot(mat, self.mat_affine)
        return


    def get_draw_image(self, canvas_size):
        mat_inv = np.linalg.inv(self.mat_affine)
        self.pil_img = self.original_img_pil.transform(canvas_size,
            Image.Transform.AFFINE, tuple(mat_inv.flatten()), Image.Resampling.NEAREST,
            fillcolor="#0000"
        )
        self.tk_img = ImageTk.PhotoImage(image=self.pil_img)
        return self.tk_img
    

    def get_corners(self):
        # 1 2  画像の四隅の座標を左の順番で返す
        # 4 3
        w, h = self.original_img_pil.width, self.original_img_pil.height
        xy1 = self.mat_affine[:, 2]
        xy2 = np.dot(self.mat_affine, [w, 0, 1])
        xy3 = np.dot(self.mat_affine, [w, h, 1])
        xy4 = np.dot(self.mat_affine, [0, h, 1])
        return xy1[0], xy1[1], xy2[0], xy2[1], xy3[0], xy3[1], xy4[0], xy4[1]
    

    def set_transparency(self, a):
        self.original_img_pil.putalpha(int(a/100*255))
        return
    
    
    def transform(self, img_x, img_y):
        mat = [img_x, img_y, 1]
        tf_mat = np.dot(self.mat_affine, mat)
        return tf_mat[0], tf_mat[1]
    
 
    def inv_transform(self, x, y):
        mat = [x, y, 1]
        inv_affine = np.linalg.inv(self.mat_affine)
        inv = np.dot(inv_affine, mat)
        return inv[0], inv[1]
    

    def image2real(self, img_x, img_y):
        real_x = (img_x - self.img_origin[0]) * self.resolution
        real_y = -(img_y - self.img_origin[1]) * self.resolution
        real_x = round(real_x, 6)
        real_y = round(real_y, 6)
        return real_x, real_y
    

    def real2image(self, real_x, real_y):
        img_x = self.img_origin[0] + real_x / self.resolution
        img_y = self.img_origin[1] - real_y / self.resolution
        return img_x, img_y
    

    def get_rotate_angle(self):
        return math.atan2(self.mat_affine[1,0], self.mat_affine[0,0])





class MapDisplay(tk.Frame):

    def __init__(self, master, theme, **kwargs):
        super().__init__(master, kwargs)
        self.min_width = 400

        self.canvas = tk.Canvas(self, bg="black", highlightthickness=0)
        self.canvas.pack(expand=True, fill=tk.BOTH, padx=5, pady=5)
        self.update()
        self.canvas.bind("<B1-Motion>", self.left_click_move)
        self.canvas.bind("<Button-1>", self.left_click)
        self.canvas.bind("<ButtonRelease-1>", self.left_click_release)
        self.canvas.bind("<Control-Button-1>", lambda event, scale=1.2: self.ctrl_click(event, scale))
        self.canvas.bind("<Control-Button-3>", lambda event, scale=0.8: self.ctrl_click(event, scale))
        self.canvas.bind("<Configure>", self.resize_callback)

        self.canv_w = self.canvas.winfo_width()
        self.canv_h = self.canvas.winfo_height()
        self.base_scale = 1
        self.map_dict = {}
        self.waypoints_dict = {}
        self.theme = theme
        self.old_click_point = None
        self.rotate_center = None
        self.old_map_rot = None
        self.selected_map_key = None
        self.mode = 0
        self.base_map_key = ""
        self.point_rad = 5

        # mode を識別するための定数
        self.Normal = 0
        self.MoveSelected = 1
        self.RotateSelected = 2
        return



    def add_map(self, path: Path, map_yaml, waypoints_yaml, base=False):
        key = self.path2key(path)
        if key in self.map_dict.keys():
            return False
        new_map = MyMap(path, map_yaml)
        img_w = new_map.original_img_pil.width
        img_h = new_map.original_img_pil.height
        scale = 1
        offset_x = 0
        offset_y = 0
        if base:
            if (self.canv_w / self.canv_h) > (img_w / img_h):
                # canvasの方が横長　画像をcanvasの縦に合わせてリサイズ
                scale = self.canv_h / img_h
                offset_x = (self.canv_w - img_w*scale) / 2
            else:
                # canvasの方が縦長　画像をcanvasの横に合わせてリサイズ
                scale = self.canv_w / img_w
                offset_y = (self.canv_h - img_h*scale) / 2
            self.base_map_key = key
            self.base_scale = scale
        else:
            scale = self.base_scale
            base_map: MyMap = self.map_dict[self.base_map_key]
            img_x, img_y = base_map.real2image(new_map.origin[0], new_map.origin[1])
            offset_x, offset_y = base_map.transform(img_x, img_y-img_h)

        new_map.scale_at(0, 0, scale)
        new_map.translate(offset_x, offset_y)
        self.canvas.create_image(0, 0, anchor=tk.NW, tags=key,
            image=new_map.get_draw_image((self.canv_w, self.canv_h))
        )
        self.map_dict[key] = new_map
        self.select_map(path)
        self.plot_origin()
        # waypoints
        self.waypoints_dict[key] = WaypointList(waypoints_yaml)
        self.waypoints_dict[key+"fp"] = FinishPose(waypoints_yaml)
        self.plot_waypoints()
        return True
    

    def plot_origin(self):
        base_map: MyMap = self.map_dict[self.base_map_key]
        canv_origin = base_map.transform(base_map.img_origin[0], base_map.img_origin[1])
        r = self.point_rad
        x1 = canv_origin[0] - r
        y1 = canv_origin[1] - r
        x2 = canv_origin[0] + r + 1
        y2 = canv_origin[1] + r + 1
        if self.canvas.find_withtag("origin"):
            self.canvas.moveto("origin", x1, y1)
        else:
            self.canvas.create_oval(x1, y1, x2, y2, tags="origin", fill='cyan', outline='blue')
        self.canvas.lift("origin")
        return
    

    def plot_waypoints(self):
        for key, wp_list in self.waypoints_dict.items():
            if key[-2:] == "fp":
                finish_pose: FinishPose = self.waypoints_dict[key]
                img_x, img_y = self.map_dict[key[:-2]].real2image(finish_pose.x, finish_pose.y)
                cx, cy = self.map_dict[key[:-2]].transform(img_x, img_y)
                th = finish_pose.yaw - self.map_dict[key[:-2]].get_rotate_angle()
                x0 = cx
                y0 = cy
                x1 = x0 + math.cos(th) * self.point_rad * 3
                y1 = y0 - math.sin(th) * self.point_rad * 3
                if self.waypoints_dict[key].id is not None:
                    self.canvas.delete(self.waypoints_dict[key].id) # movetoだとX,毎回削除,再描画
                self.waypoints_dict[key].id = self.canvas.create_line(x0, y0, x1, y1, tags=key,
                    width=5, arrow=tk.LAST, arrowshape=(4,5,3), fill="#AAF"
                )
                self.canvas.lift(self.waypoints_dict[key].id, key[:-2])
                continue
            
            if len(wp_list.get_id_list()) == 0: # 初めて描画する
                for n, wp in enumerate(wp_list.get_waypoint()):
                    id = self.create_waypoint(key, wp, self.map_dict[key])
                    self.waypoints_dict[key].set_id(n+1, id)
            else:
                for id in wp_list.get_id_list():
                    wp = wp_list.get_waypoint(id)
                    img_x, img_y = self.map_dict[key].real2image(float(wp["x"]), float(wp["y"]))
                    cx, cy = self.map_dict[key].transform(img_x, img_y)
                    x0, y0 = cx-self.point_rad, cy-self.point_rad
                    self.canvas.moveto(id, round(x0), round(y0))
                    self.canvas.lift(id, key)
        return
    

    def create_waypoint(self, key, waypoint: dict, mymap: MyMap):
        img_x, img_y = mymap.real2image(float(waypoint["x"]), float(waypoint["y"]))
        cx, cy = mymap.transform(img_x, img_y)
        r = self.point_rad-1
        x0 = round(cx - r)
        y0 = round(cy - r)
        x1 = round(cx + r + 1)
        y1 = round(cy + r + 1)
        id = self.canvas.create_oval(x0, y0, x1, y1, fill='#F88', outline='#F88', tags=key+"wp")
        return id


    def set_transparency(self, path: Path, alpha):
        key = self.path2key(path)
        self.map_dict[key].set_transparency(alpha)
        self.canvas.itemconfigure(key, image=self.map_dict[key].get_draw_image((self.canv_w, self.canv_h)))



    def select_map(self, path: Path):
        self.selected_map_key = self.path2key(path)
        polygon = self.map_dict[self.selected_map_key].get_corners()
        if self.canvas.find_withtag("selection_frame"):
            self.canvas.coords("selection_frame", polygon)
        else:
            self.canvas.create_polygon(polygon, tags="selection_frame", fill="", outline="#FF0", dash=(5,3), width=5)
        self.canvas.lift(self.selected_map_key)
        self.canvas.lift("selection_frame")
        self.canvas.lift(self.selected_map_key+"wp")
        self.canvas.lift(self.selected_map_key+"fp")
        self.canvas.lift("origin")
        return
    

    def set_vision_state(self, path: Path, vision: bool):
        key = self.path2key(path)
        if vision:
            self.canvas.itemconfigure(key, state=tk.NORMAL)
            self.canvas.itemconfigure(key+"wp", state=tk.NORMAL)
            self.canvas.itemconfigure(key+"fp", state=tk.NORMAL)
        else:
            self.canvas.itemconfigure(key, state=tk.HIDDEN)
            self.canvas.itemconfigure(key+"wp", state=tk.HIDDEN)
            self.canvas.itemconfigure(key+"fp", state=tk.HIDDEN)

    

    def path2key(self, path: Path):
        return path.with_suffix("").name
    

    def get_map(self, path: Path):
        key = self.path2key(path)
        return self.map_dict[key]
    

    def left_click_move(self, event):
        if (len(self.map_dict) == 0): return
        if self.old_click_point is None:
            self.old_click_point = [event.x, event.y]
            return
        delta_x = event.x - self.old_click_point[0]
        delta_y = event.y - self.old_click_point[1]
        self.old_click_point = [event.x, event.y]

        if self.mode == self.RotateSelected:
            # 選択されているマップを回転する
            if self.rotate_center is None:
                self.rotate_center = self.old_click_point
                r = self.point_rad
                x1, y1 = self.rotate_center[0]-r, self.rotate_center[1]-r
                x2, y2 = self.rotate_center[0]+r, self.rotate_center[1]+r
                self.canvas.create_oval(x1, y1, x2, y2, fill="#FFA", outline="#FF0", tags="rotation_center")
                return
            key = self.selected_map_key
            cx, cy = self.rotate_center
            if (abs(event.x-cx) < 20) and (abs(event.y-cy) < 20): return
            map_rot = math.atan2((event.y - cy), (event.x - cx))
            if self.old_map_rot is None:
                self.old_map_rot = map_rot
                return
            theta = map_rot - self.old_map_rot
            self.map_dict[key].rotate(theta, canv_center=self.rotate_center)
            self.canvas.itemconfigure(key, image=self.map_dict[key].get_draw_image((self.canv_w, self.canv_h)))
            if self.canvas.find_withtag("rotation_line"):
                self.canvas.coords("rotation_line", cx, cy, event.x, event.y)
            else:
                self.canvas.create_line(cx, cy, event.x, event.y, tags="rotation_line", width=1, fill="#FAA")
            polygon = polygon = self.map_dict[key].get_corners()
            self.canvas.coords("selection_frame", polygon)
            self.canvas.lift("origin")
            self.plot_waypoints()
            self.old_map_rot = map_rot
            return
        
        elif self.mode == self.MoveSelected:
            # 選択されているマップのみ移動する
            key = self.selected_map_key
            self.map_dict[key].translate(delta_x, delta_y)
            self.canvas.itemconfigure(key, image=self.map_dict[key].get_draw_image((self.canv_w, self.canv_h)))
            self.canvas.move(key+"wp", delta_x, delta_y)
            self.canvas.move(key+"fp", delta_x, delta_y)

        else:
            # 全てのマップを移動する
            for key in self.map_dict.keys():
                self.map_dict[key].translate(delta_x, delta_y)
                self.canvas.itemconfigure(key, image=self.map_dict[key].get_draw_image((self.canv_w, self.canv_h)))
                self.canvas.move(key+"wp", delta_x, delta_y)
                self.canvas.move(key+"fp", delta_x, delta_y)
            self.canvas.move("origin", delta_x, delta_y)
        
        self.canvas.move("selection_frame", delta_x, delta_y)
        self.canvas.lift("origin")
        # self.plot_waypoints()
        return
    

    def ctrl_click(self, event, scale):
        if (len(self.map_dict) == 0): return
        self.base_scale *= scale
        for key in self.map_dict.keys():
            self.map_dict[key].scale_at(event.x, event.y, scale)
            self.canvas.itemconfigure(key, image=self.map_dict[key].get_draw_image((self.canv_w, self.canv_h)))
        
        self.plot_origin()
        self.plot_waypoints()
        polygon = polygon = self.map_dict[self.selected_map_key].get_corners()
        self.canvas.coords("selection_frame", polygon)
        self.canvas.lift("selection_frame")
        self.canvas.lift("origin")
        return
    

    def left_click(self, event):
        if (self.mode != self.RotateSelected): return
        self.old_click_point = [event.x, event.y]
        return
    

    def left_click_release(self, event):
        self.old_click_point = None
        self.old_map_rot = None
        self.rotate_center = None
        if self.canvas.find_withtag("rotation_line"):
            self.canvas.delete("rotation_line")
        if self.canvas.find_withtag("rotation_center"):
            self.canvas.delete("rotation_center")
        return


    def resize_callback(self, event):
        self.canv_w = self.canvas.winfo_width()
        self.canv_h = self.canvas.winfo_height()
        return



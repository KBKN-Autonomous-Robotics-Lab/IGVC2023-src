import tkinter as tk
import numpy as np
import ruamel.yaml
import math
from tkinter import messagebox
from PIL import Image, ImageTk
from pathlib import Path
from .mapdisp import MapDisplay, MyMap
from .waypointlib import FinishPose, WaypointList, get_waypoint_yaml



def read_file(path: Path):
    with open(path) as file:   # .yamlを読み込む
        yaml = ruamel.yaml.YAML().load(file)
    return yaml



class LayerLabel(tk.Label):

    def __init__(self, master, *args, **kwargs):
        super().__init__(master, *args, **kwargs)
        self.map_path = Path()
        self.map_transparency = tk.IntVar()
        self.map_transparency.set(100)
        self.state = "unlocked"

    
    def set_map_path(self, path: Path):
        self.map_path = path



class Tools(tk.Frame):

    def __init__(self, master, theme, **kwargs):
        super().__init__(master, kwargs)
        self.min_width = 200
        width = self.cget("width")

        #### ツールボタン群を配置 ####
        self.btn_frame = tk.Frame(self, height=200, width=width, bg=theme["main"])
        self.btn_frame.pack(expand=False, fill=tk.X)
        ## move ボタン
        icon = Image.open(Path(__file__).parent.parent / Path("icon","move_button.png"))
        icon = icon.resize((30, 30))
        self.move_btn_icon = ImageTk.PhotoImage(image=icon)
        self.move_btn = tk.Label(self.btn_frame, image=self.move_btn_icon, bg=theme["main"])
        self.move_btn.bind("<Enter>", lambda event, btn=self.move_btn: self.btn_entry(event, btn))
        self.move_btn.bind("<Leave>", lambda event, btn=self.move_btn: self.btn_leave(event, btn))
        self.move_btn.bind("<Button-1>", lambda event, btn=self.move_btn, mode="move": self.btn_clicked(event, btn, mode))
        self.move_btn.grid(column=0, row=0)
        ## rotate ボタン
        icon = Image.open(Path(__file__).parent.parent / Path("icon","rotate_button.png"))
        icon = icon.resize((30, 30))
        self.rot_btn_icon = ImageTk.PhotoImage(image=icon)
        self.rot_btn = tk.Label(self.btn_frame, image=self.rot_btn_icon, bg=theme["main"])
        self.rot_btn.bind("<Enter>", lambda event, btn=self.rot_btn: self.btn_entry(event, btn))
        self.rot_btn.bind("<Leave>", lambda event, btn=self.rot_btn: self.btn_leave(event, btn))
        self.rot_btn.bind("<Button-1>", lambda event, btn=self.rot_btn, mode="rotate": self.btn_clicked(event, btn, mode))
        self.rot_btn.grid(column=1, row=0)
        ## lock, unlock ボタン
        icon = Image.open(Path(__file__).parent.parent / Path("icon","locked_button.png"))
        icon = icon.resize((30,30))
        self.locked_icon = ImageTk.PhotoImage(image=icon)
        icon = Image.open(Path(__file__).parent.parent / Path("icon","unlocked_button.png"))
        icon = icon.resize((30,30))
        self.unlocked_icon = ImageTk.PhotoImage(image=icon)
        self.lock_btn = tk.Label(self.btn_frame, image=self.unlocked_icon, bg=theme["main"])
        self.lock_btn.bind("<Enter>", lambda event, btn=self.lock_btn: self.btn_entry(event, btn))
        self.lock_btn.bind("<Leave>", lambda event, btn=self.lock_btn: self.btn_leave(event, btn))
        self.lock_btn.bind("<Button-1>", self.lock_btn_clicked)
        self.lock_btn.grid(column=0, row=1)
        ## 余白
        space = tk.Frame(self, height=100, bg=theme["main"])
        space.pack(expand=False, fill=tk.X)

        #### マップリストを表示 ####
        ## アイコン
        icon = Image.open(Path(__file__).parent.parent / Path("icon","maps.png"))
        icon = icon.resize((20, 20))
        self.layer_icon = ImageTk.PhotoImage(image=icon)
        layer_label = tk.Label(self, bg=theme["bg1"],
            text="  Maps", fg="white", font=("Arial",11,"bold"),
            image=self.layer_icon, compound=tk.LEFT
        )
        layer_label.pack(expand=False, anchor=tk.W, ipadx=3, ipady=2, padx=5)
        ## レイヤーリストを表示するフレームを配置
        self.layers_frame = tk.Frame(self, width=width, bg=theme["bg1"])
        self.layers_frame.pack(expand=True, fill=tk.BOTH, padx=5)
        ## 透明度調節用のスクロールバーを配置
        self.tra_bar = tk.Scale(self.layers_frame, from_=0, to=100, resolution=1, highlightthickness=0,
            label="Transparency", fg="white", font=("Ariel",9,"bold"), bg=theme["bg1"],
            activebackground="grey", borderwidth=2, sliderlength=15, sliderrelief=tk.RAISED,
            orient=tk.HORIZONTAL, showvalue=True, state=tk.DISABLED, command=self.change_transparency
        )
        self.tra_bar.pack(expand=False, fill=tk.X, padx=3, pady=5)
        ## アイコンを読み込む
        icon = Image.open(Path(__file__).parent.parent / Path("icon","visible.png"))
        icon = icon.resize((18, 17))
        self.visible_icon = ImageTk.PhotoImage(image=icon)
        icon = Image.open(Path(__file__).parent.parent / Path("icon","invisible.png"))
        icon = icon.resize((18, 17))
        self.invisible_icon = ImageTk.PhotoImage(image=icon)
        icon = Image.open(Path(__file__).parent.parent / Path("icon","folder.png"))
        icon = icon.resize((20, 14))
        self.folder_icon = ImageTk.PhotoImage(image=icon)
        
        #### マップ画像を表示するフレームを生成 ####
        self.map_disp = MapDisplay(self.master, theme, bg=theme["main"])

        #### 変数
        self.label_list = []
        self.base_map_layer = LayerLabel(self.layers_frame)
        self.selected_layer = LayerLabel(self.layers_frame)
        self.base_waypoints_path = Path()
        self.theme = theme
        return



    def btn_entry(self, event, btn: tk.Label):
        if btn.cget("bg") == "black": return
        btn.configure(bg="#333")
    

    def btn_leave(self, event, btn: tk.Label):
        if btn.cget("bg") == "black": return
        btn.configure(bg=self.theme["main"])
    

    def btn_clicked(self, event, btn: tk.Label, mode):
        if (len(self.label_list) == 0) or (self.selected_layer == self.base_map_layer): return
        if self.selected_layer.state == "locked": return
        if btn.cget("bg") == "black":
            self.map_disp.mode = self.map_disp.Normal
            btn.configure(bg="#333")
            return
        # 既に他のモードだった場合ボタンの状態を切り替える
        if self.map_disp.mode == self.map_disp.MoveSelected: self.move_btn.configure(bg=self.theme["main"])
        elif self.map_disp.mode == self.map_disp.RotateSelected: self.rot_btn.configure(bg=self.theme["main"])
        # MapDisplayにモードを設定
        if mode == "move": self.map_disp.mode = self.map_disp.MoveSelected
        elif mode == "rotate": self.map_disp.mode = self.map_disp.RotateSelected
        btn.configure(bg="black")
        return
    

    def lock_btn_clicked(self, event):
        if (len(self.label_list) == 0) or (self.selected_layer == self.base_map_layer): return
        if self.lock_btn.cget("bg") == "black":
            self.lock_btn.configure(image=self.unlocked_icon, bg=self.theme["main"])
            self.selected_layer.state = "unlocked"
        else:
            self.lock_btn.configure(image=self.locked_icon, bg="black")
            self.selected_layer.state = "locked"
        return


    
    def set_base_map(self, map_path: Path, wp_path: Path):
        map_yaml = read_file(map_path)
        if not ("image" in map_yaml):
            messagebox.showerror(title="Format error", message="Selected map file is unexpected format.")
            return False
        wp_yaml = read_file(wp_path)
        if (not "waypoints" in wp_yaml) or (not "finish_pose" in wp_yaml):
            messagebox.showerror(title="Format error", message="Selected waypoints file is unexpected format.")
            return False
        try:
            self.map_disp.add_map(map_path, map_yaml, wp_yaml, base=True)
        except (FileNotFoundError, FileExistsError):
            messagebox.showerror(title="Image file is not found", message="\""+map_yaml["image"]+"\"  is not found.")
            return False
        self.append_layer(map_path, base=True)
        self.base_map_layer.state = "locked"
        self.lock_btn.configure(image=self.locked_icon, bg="black")
        self.base_waypoints_path = wp_path
        return True

    
    def add_map(self, path: Path, wp_path: Path):
        map_yaml = read_file(path)
        if not ("image" in map_yaml):
            messagebox.showerror(title="Format error", message="Selected map file is unexpected format.")
            return False
        wp_yaml = read_file(wp_path)
        if (not "waypoints" in wp_yaml) or (not "finish_pose" in wp_yaml):
            messagebox.showerror(title="Format error", message="Selected waypoints file is unexpected format.")
            return False
        try:
            if self.map_disp.add_map(path, map_yaml, wp_yaml):
                self.append_layer(path)
            else:
                i = 2
                path2 = path.with_name(path.with_suffix("").name + "-" + str(i))
                while(not self.map_disp.add_map(path2, map_yaml, wp_yaml)):
                    i += 1
                    path2 = path.with_name(path.with_suffix("").name + "-" + str(i))
                self.append_layer(path2)
                path = path2
        except (FileNotFoundError, FileExistsError):
            messagebox.showerror(title="Image file is not found", message="\""+map_yaml["image"]+"\"  is not found.")
            return False
        self.selected_layer.map_transparency.set(70)
        self.map_disp.set_transparency(path, 70)
        self.tra_bar.configure(variable=self.selected_layer.map_transparency)
        return True
    

    def set_multimaps(self, dirpath: Path, wp_path: Path):
        all_wp_yaml = read_file(wp_path)
        if (not "waypoints" in all_wp_yaml) or (not "finish_pose" in all_wp_yaml):
            messagebox.showerror(title="Format error", message="Selected waypoints file is unexpected format.")
            return False
        map_idx = 0
        wp_idx = 0
        while(True):
            map_path = dirpath / Path("map{}.yaml".format(map_idx))
            if (not map_path.exists()) or (wp_idx >= len(all_wp_yaml["waypoints"])): break
            map_yaml = read_file(map_path)
            wp_yaml = {"waypoints":[]}
            while(True):
                if wp_idx >= len(all_wp_yaml["waypoints"]):
                    wp_yaml["finish_pose"] = all_wp_yaml["finish_pose"]
                    break
                point = all_wp_yaml["waypoints"][wp_idx]
                if not "change_map" in point["point"].keys():
                    wp_yaml["waypoints"].append(point)
                    wp_idx += 1
                else:
                    finish_pose = point["point"]
                    wp_yaml["finish_pose"] = {
                        "header":{"seq":0, "stamp":0, "frame_id":"map"},
                        "pose":{
                            "position":{"x":finish_pose["x"], "y":finish_pose["y"], "z":finish_pose["z"]},
                            "orientation":{"x":0, "y":0, "z":0, "w":1}
                        }
                    }
                    break
            if map_idx==0:
                self.map_disp.add_map(map_path, map_yaml, wp_yaml, base=True)
                self.append_layer(map_path, base=True)
                self.base_map_layer.state = "locked"
                self.lock_btn.configure(image=self.locked_icon, bg="black")
                self.base_waypoints_path = wp_path
            else:
                self.map_disp.add_map(map_path, map_yaml, wp_yaml)
                self.append_layer(map_path)
                self.selected_layer.map_transparency.set(70)
                self.map_disp.set_transparency(map_path, 70)
                self.tra_bar.configure(variable=self.selected_layer.map_transparency)
            if "change_map" in point["point"].keys(): map_idx = int(point["point"]["change_map"])
            wp_idx += 1

        return True

    
    def append_layer(self, path: Path, base=None):
        name = " " + path.with_suffix("").name
        font = ("Arial",10)
        if base:
            name += " ( base )"
            font = ("Arial", 10, "bold")
        label = LayerLabel(self.layers_frame, bg=self.theme["main"], bd=0,
            text=name, fg="white", font=font, anchor=tk.W, padx=10, pady=2,
            image=self.visible_icon, compound=tk.LEFT
        )
        label.set_map_path(path)
        label.bind("<Button-1>", lambda event, label=label: self.layerlabel_clicked(event, label))
        if self.label_list:
            label.pack(expand=False, side=tk.TOP, fill=tk.X, padx=2, pady=2, ipady=3, before=self.label_list[-1])
        else:
            label.pack(expand=False, side=tk.TOP, fill=tk.X, padx=2, pady=2, ipady=3)

        if self.selected_layer is not None:
            self.selected_layer.configure(bg=self.theme["bg1"])
        self.selected_layer = label
        self.tra_bar.configure(variable=self.selected_layer.map_transparency, state=tk.NORMAL)
        self.lock_btn.configure(image=self.unlocked_icon, bg=self.theme["main"])
        self.label_list.append(label)
        if base: self.base_map_layer = label
        return
    

    def layerlabel_clicked(self, event, label: LayerLabel):
        px = label.cget("padx")
        imw = self.visible_icon.width()
        if (px-3 < event.x) and (event.x < px+imw+3):
            # 可視アイコンがクリックされたとき
            fg = label.cget("fg")
            if fg == "white":
                label.configure(image=self.invisible_icon, fg="grey")
                self.map_disp.set_vision_state(label.map_path, vision=False)
            else:
                label.configure(image=self.visible_icon, fg="white")
                self.map_disp.set_vision_state(label.map_path, vision=True)
        
        if label != self.selected_layer:
            self.selected_layer.configure(bg=self.theme["bg1"])
            label.configure(bg=self.theme["main"])
            self.selected_layer = label
            self.tra_bar.configure(variable=self.selected_layer.map_transparency)
            self.map_disp.select_map(self.selected_layer.map_path)
            for btn in [self.move_btn, self.rot_btn]:
                if btn.cget("bg") == "black":
                    self.map_disp.mode = self.map_disp.Normal
                    btn.configure(bg=self.theme["main"])
                    break
            if self.selected_layer.state == "locked":
                self.lock_btn.configure(image=self.locked_icon, bg="black")
            else:
                self.lock_btn.configure(image=self.unlocked_icon, bg=self.theme["main"])
        return


    def change_transparency(self, event):
        alpha = self.selected_layer.map_transparency.get()
        self.map_disp.set_transparency(self.selected_layer.map_path, alpha)
        return
    

    def get_map_lists(self):
        base_map: MyMap = self.map_disp.map_dict[self.map_disp.base_map_key]
        base_yaml = base_map.map_yaml
        img_list = [base_map.original_img_pil]
        yaml_list = [base_yaml]
        for label in self.label_list[1:]:
            mymap: MyMap = self.map_disp.get_map(label.map_path)
            # image
            img = mymap.original_img_pil.convert("L")
            theta = math.degrees(-mymap.get_rotate_angle())
            img = img.rotate(theta, expand=True, fillcolor=205)
            img_list.append(img)
            # yaml
            yaml = mymap.map_yaml
            corners = np.array(mymap.get_corners())
            left = corners[[0,2,4,6]].min()
            lower = corners[[1,3,5,7]].max()
            img_x, img_y = base_map.inv_transform(left, lower)
            real_x, real_y = base_map.image2real(img_x, img_y)
            yaml["origin"] = [real_x, real_y, 0.0]
            yaml_list.append(yaml)
        return img_list, yaml_list
    

    def get_waypoints_yaml(self):
        # ベースのウェイポイントをコピー
        waypoints = WaypointList(self.map_disp.waypoints_dict[self.map_disp.base_map_key].waypoints_yaml)
        # ベースのfinish poseをウェイポイントに変換
        finish_pose: FinishPose = self.map_disp.waypoints_dict[self.map_disp.base_map_key + "fp"]
        # 個別のウェイポイントをリスト化
        map_idx = 1
        wp_yaml_list = [get_waypoint_yaml(waypoints, finish_pose)]
        wpc = waypoints.get_waypoint(num=1).copy()
        wpc["x"] = finish_pose.x
        wpc["y"] = finish_pose.y
        wpc["stop"] = "true"
        wpc["change_map"] = map_idx
        map_idx += 1
        waypoints.append(wpc)
        # 残りのウェイポイントを追加していく
        base_map: MyMap = self.map_disp.map_dict[self.map_disp.base_map_key]
        name = "waypoints-" + base_map.yaml_path.with_suffix("").name
        for label in self.label_list[1:]:
            mymap: MyMap = self.map_disp.get_map(label.map_path)
            name += "-" + mymap.yaml_path.with_suffix("").name
            # 2つ目以降の地図のもともとの原点のキャンバス上の位置を取得
            origin = mymap.transform(mymap.img_origin[0], mymap.img_origin[1])
            # ベースのマップから見た原点の座標を計算
            img_x, img_y = base_map.inv_transform(origin[0], origin[1])
            real_x, real_y = base_map.image2real(img_x, img_y)
            # 地図の回転角
            theta = -mymap.get_rotate_angle()
            # 2つ目以降の地図に対応するウェイポイントの座標を、ベースの地図の座標系に変換する同次変換行列
            mat = [[math.cos(theta), -math.sin(theta), real_x],
                   [math.sin(theta),  math.cos(theta), real_y],
                   [0,                0,               1     ]]
            wp_list: WaypointList = self.map_disp.waypoints_dict[self.map_disp.path2key(label.map_path)]
            wp_list_copy = WaypointList(wp_list.waypoints_yaml)
            for i, wp in enumerate(wp_list.get_waypoint()):
                tf_xy = np.dot(mat, [wp["x"], wp["y"], 1]) # ベースの座標系に変換
                wpc = wp.copy()
                wpc["x"] = round(tf_xy[0], 6)
                wpc["y"] = round(tf_xy[1], 6)
                waypoints.append(wpc)
                wp_list_copy.waypoints[i] = wpc
            # 2つ目以降の地図のfinish poseも変換
            finish_pose = FinishPose(wp_list.waypoints_yaml)
            tf_xy = np.dot(mat, [finish_pose.x, finish_pose.y, 1])
            finish_pose.x = tf_xy[0]
            finish_pose.y = tf_xy[1]
            finish_pose.yaw = finish_pose.yaw + theta
            wp_yaml_list.append(get_waypoint_yaml(wp_list_copy, finish_pose))
            # 最後以外のfinish poseはウェイポイントとして追加
            if label != self.label_list[-1]:
                wpc = waypoints.get_waypoint(num=1).copy()
                wpc["x"] = finish_pose.x
                wpc["y"] = finish_pose.y
                wpc["stop"] = "true"
                wpc["change_map"] = map_idx
                map_idx += 1
                waypoints.append(wpc)
        return get_waypoint_yaml(waypoints, finish_pose), wp_yaml_list
    

    def get_merged_map(self):
        # 1 2  画像の四隅のキャンバス上の座標を取得
        # 4 3
        base_map: MyMap = self.map_disp.map_dict[self.map_disp.base_map_key]
        corners = base_map.get_corners() # x1, y1, x2, y2, x3, y3, x4, y4
        img_corners = np.array(corners)
        # ベースの地図画像を基準として画像上の座標に変換
        for i in range(0,8,2):
            img_corners[i:i+2] = base_map.inv_transform(corners[i], corners[i+1])
        for label in self.label_list[1:]:
            mymap: MyMap = self.map_disp.get_map(label.map_path)
            corners = np.array(mymap.get_corners())
            for i in range(0,8,2):
                corners[i:i+2] = base_map.inv_transform(corners[i], corners[i+1])
            img_corners = np.vstack((img_corners, corners))
        # 合成する地図画像の中で最も外側にある座標を取得
        left = img_corners[:, [0, 2, 4, 6]].min()
        upper = img_corners[:, [1, 3, 5, 7]].min()
        right = img_corners[:, [0, 2, 4, 6]].max()
        lower = img_corners[:, [1, 3, 5, 7]].max()
        # 合成後の画像を準備
        offset = 2  # 結合時の画像サイズの不揃いを解消
        img_size = (int(round(lower-upper)+offset), int(round(right-left)+offset)) #(y, x)
        unknown = np.full(img_size, 205, dtype=np.uint8) # 未確定領域
        obstacles = np.full(img_size, 255, dtype=np.uint8) # 障害物が0, それ以外が255
        free_area = np.full(img_size, 0, dtype=np.uint8) # 走行可能領域が255, それ以外が0
        # ベースの画像を合成
        img = np.array(base_map.original_img_pil.convert("L"), dtype=np.uint8)
        x, y = int(round(img_corners[0,0]-left)), int(round(img_corners[0,1]-upper))
        h, w = img.shape
        obstacles[y:y+h, x:x+w] = obstacles[y:y+h, x:x+w] & np.where(img>100, 255, 0)
        free_area[y:y+h, x:x+w] = free_area[y:y+h, x:x+w] | np.where(img<230, 0, 255)
        name = "merged-" + base_map.yaml_path.with_suffix("").name
        # その他の画像も合成
        for i in range(1, len(self.label_list)):
            label = self.label_list[i]
            mymap: MyMap = self.map_disp.get_map(label.map_path)
            img = mymap.original_img_pil.convert("L")
            img = img.rotate(-math.degrees(mymap.get_rotate_angle()),
                resample=Image.Resampling.NEAREST, expand=True, fillcolor=205)
            img = np.array(img, dtype=np.uint8)
            x = int(round(img_corners[i, [0, 2, 4, 6]].min() - left))
            y = int(round(img_corners[i, [1, 3, 5, 7]].min() - upper))
            h, w = img.shape
            obstacles[y:y+h, x:x+w] = obstacles[y:y+h, x:x+w] & np.where(img>100, 255, 0)
            free_area[y:y+h, x:x+w] = free_area[y:y+h, x:x+w] | np.where(img<230, 0, 255)
            name += "-" + Path(label.map_path).with_suffix("").name
        # 未確定領域、走行可能領域、障害物を合成し、画像に変換
        merged_img = (unknown | free_area) & obstacles
        merged_img = merged_img[:-offset, :-offset]  # offset分を削除
        merged_img = Image.fromarray(merged_img, "L")
        # 原点から合成後の画像の左下までの距離
        origin = [left-base_map.img_origin[0], -lower+base_map.img_origin[1], 0]
        origin[0] = origin[0] * base_map.resolution
        origin[1] = origin[1] * base_map.resolution
        merged_yaml = {
            "image":"./" + base_map.yaml_path.with_name(name+".pgm").name,
            "resolution":base_map.resolution,
            "origin":origin,
            "negate":base_map.map_yaml["negate"],
            "occupied_thresh":base_map.map_yaml["occupied_thresh"],
            "free_thresh":base_map.map_yaml["free_thresh"]
        }
        return merged_img, merged_yaml
    

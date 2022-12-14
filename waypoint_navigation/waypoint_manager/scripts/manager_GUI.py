import tkinter as tk
import tkinter.filedialog
import math
import ruamel.yaml
import gc
import itertools
from pathlib import Path
from tkinter import messagebox
from PIL import Image, ImageTk
from lib.mymaplib import MyMap
from lib.waypointlib import WaypointList, FinishPose, get_waypoint_yaml




class Application(tk.Frame):

    def __init__(self, master):
        super().__init__(master)   # スーパークラスのコンストラクタを実行
        self.master.title("Waypoints Manager")

        #### 画面上部のメニューを作成 ####
        self.menu_bar = tk.Menu(self)   # メニューバーを配置
        self.file_menu = tk.Menu(self.menu_bar, tearoff=tk.OFF)   # バーに追加するメニューを作成
        self.open_menu = tk.Menu(self.file_menu, tearoff=tk.OFF)   # Openメニュー
        self.open_menu.add_command(label="Map",       command=self.menu_open_map)
        self.open_menu.add_command(label="Waypoints", command=self.menu_open_waypoints, state=tk.DISABLED)
        self.file_menu.add_cascade(label="Open", menu=self.open_menu)
        self.file_menu.add_command(label="Save", command=self.menu_save,   #コールバック関数を設定
            accelerator="Ctrl+S", state=tk.DISABLED
        )
        self.file_menu.add_command(label="Save As", command=self.menu_saveas, 
            accelerator="Ctrl+Shift+S", state=tk.DISABLED
        )
        self.file_menu.add_separator()
        self.file_menu.add_command(label="Exit", command=self.menu_exit, accelerator="Ctrl+Q")
        self.menu_bar.add_cascade(label="File", menu=self.file_menu)   # Fileメニューとしてバーに追加
        self.bind_all("<Control-s>", self.menu_save)   #キーボードショートカットを設定
        self.bind_all("<Control-Shift-S>", self.menu_saveas)
        self.bind_all("<Control-q>", self.menu_exit)

        self.edit_menu = tk.Menu(self.menu_bar, tearoff=tk.OFF)
        self.edit_menu.add_command(label="Set Finish Pose", command=self.menu_set_finishpose)
        self.edit_menu.add_command(label="From To", command=self.menu_from_to)
        self.menu_bar.add_cascade(label="Edit", menu=self.edit_menu)

        self.show_menu = tk.Menu(self.menu_bar, tearoff=tk.OFF)
        self.fp_menu = tk.Menu(self.show_menu, tearoff=tk.OFF)
        self.fp_menu.add_command(label="Set model", command=self.menu_set_footprint)
        self.show_fp = tk.BooleanVar()
        self.fp_menu.add_checkbutton(label="Show footprint", command=self.menu_show_footprint, 
            variable=self.show_fp, accelerator="Ctrl+F"
        )
        self.show_menu.add_cascade(label="Footprint", menu=self.fp_menu)
        self.menu_bar.add_cascade(label="View", menu=self.show_menu)
        self.bind_all("<Control-f>", self.menu_show_footprint)

        self.master.configure(menu=self.menu_bar)   # 大元に作成したメニューバーを設定

        #### 画面上部に、システムからのメッセージを表示するラベルを配置 ####
        self.msg_label = tk.Label(self.master, text="Please open map file ", anchor=tk.E, font=("Consolas",13))
        self.msg_label.pack(fill=tk.X, padx=5)

        #### 画面下部に、カーソルの座標やピクセル情報を表示するステータスバーを表示 ####
        self.status_bar = tk.Frame(self.master)
        self.mouse_position = tk.Label(self.status_bar, relief=tk.SUNKEN,
            text=" (x, y) = ", anchor=tk.W, font=("Consolas", 15)
        )
        self.mouse_position.pack(side=tk.LEFT, padx=3)
        self.waypoint_num = tk.Label(self.status_bar, relief=tk.SUNKEN,
            text=" Waypoint No. -----", anchor=tk.W, font=("Consolas", 15)
        )
        self.waypoint_num.pack(side=tk.RIGHT, padx=3)
        self.status_bar.pack(side=tk.BOTTOM, fill=tk.X)

        #### 右クリックしたときに表示するポップアップメニューを作成 ####
        self.popup_menu = tk.Menu(self, tearoff=tk.OFF)
        self.popup_menu.add_command(label="Add waypoint here", command=self.add_waypoint_here, state=tk.DISABLED)
        self.right_click_coord = None   # 右クリックしたときの座標を保持する変数

        #### canvasを配置、サイズを取得 ####
        self.canvas = tk.Canvas(self.master, bg="#034")   # 画像を描画するcanvas
        self.canvas.pack(expand=True, fill=tk.BOTH)   # canvasを配置
        self.update()   # 情報の更新をする(canvasのサイズなどの情報が更新される)
        self.canv_w = self.canvas.winfo_width()   # canvasの幅を取得
        self.canv_h = self.canvas.winfo_height()   # canvasの高さを取得

        #### イベントに対するコールバックを設定 ####
        self.master.bind("<Motion>", self.mouse_move)
        self.master.bind("<MouseWheel>", self.mouse_wheel)
        self.master.bind("<B1-Motion>", self.left_click_move)
        self.master.bind("<Button-1>", self.left_click)
        self.master.bind("<ButtonRelease-1>", self.left_click_release)
        self.master.bind("<Button-3>", self.right_click)
        self.master.bind("<Control-Button-1>", self.ctrl_left_click)
        self.master.bind("<Control-Button-3>", self.ctrl_right_click)
        self.master.bind("<Configure>", self.window_resize_callback)

        #### アイコン ####
        icon = Image.open(Path(__file__).parent / Path("icons","new_param_btn.png"))
        icon = icon.resize((30, 30))
        self.new_param_icon = ImageTk.PhotoImage(image=icon)
        icon = Image.open(Path(__file__).parent / Path("icons","delete_param_btn.png"))
        icon = icon.resize((30, 30))
        self.del_param_icon = ImageTk.PhotoImage(image=icon)

        #### その他必要になる変数,オブジェクト ####
        self.mymap: MyMap = None
        self.waypoints: WaypointList = None
        self.finish_pose: FinishPose = None
        self.waypoints_filepath: Path = None
        self.editing_waypoint_id = None   # 編集中のウェイポイントを示す図形のオブジェクトID
        self.moving_waypoint = False   # ウェイポイントをmoveで動かしている最中かどうか
        self.setting_finish_pose = 0 # finish pose のセット中かどうか
        self.old_click_point = None   # 最後にカーソルのあった座標を保持
        self.wp_info_win: tk.Toplevel = None   # ウェイポイント情報を表示するウィンドウ
        self.point_rad = 10   # 画像上に示すポイントの半径ピクセル
        self.footprint = [[0.25, 0.4], [0.25, -0.4], [-0.65, -0.4], [-0.65, 0.4]]
        self.trajectory = []
        return



    """
    ###############################
        メニューコールバック関数
    ###############################
    """
    """
    +++++ File -> Open -> Map +++++
    """
    def menu_open_map(self):
        map_path = tkinter.filedialog.askopenfilename(
            parent=self.master,
            title="Select map yaml file",
            initialdir=str(Path(".")),
            filetypes=[("YAML", ".yaml")]
        )
        if not map_path: return

        with open(map_path) as file:   # .yamlを読み込む
            map_yaml = ruamel.yaml.YAML().load(file)
        if  not "image" in map_yaml.keys():
            messagebox.showerror(title="Format error", message="Selected map file is unexpected format.")
            return
        try:
            del self.mymap
            self.mymap = MyMap(Path(map_path), map_yaml)
        except FileNotFoundError:
            messagebox.showerror(title="Image file is not found", message="\""+map_yaml["image"]+"\"  is not found.")
            self.mymap = None
            return
        self.message("Read map file " + map_path)
        self.canvas.delete("all")
        self.trajectory = []
        if self.waypoints is not None: self.waypoints.number_dict = {}
        ## キャンバスサイズに合わせて画像を表示
        scale = 1
        offset_x = 0
        offset_y = 0
        if (self.canv_w / self.canv_h) > (self.mymap.width() / self.mymap.height()):
            # canvasの方が横長　画像をcanvasの縦に合わせてリサイズ
            scale = self.canv_h / self.mymap.height()
            offset_x = (self.canv_w - self.mymap.width()*scale) / 2
        else:
            # canvasの方が縦長　画像をcanvasの横に合わせてリサイズ
            scale = self.canv_w / self.mymap.width()
            offset_y = (self.canv_h - self.mymap.height()*scale) / 2
        
        self.mymap.scale_at(0, 0, scale)
        self.mymap.translate(offset_x, offset_y)
        self.draw_image()   # 画像を描画
        self.plot_origin()   # 原点を示す円を描画
        if self.waypoints_filepath is not None:
            self.master.title(self.waypoints_filepath.name + " - " + Path(map_path).name)
        else:
            self.master.title(Path(map_path).name)
        self.open_menu.entryconfigure("Waypoints", state=tk.NORMAL)
        if self.waypoints is None:
            self.message("Please open waypoints file ")
            self.menu_open_waypoints()
        else:
            self.plot_waypoints()
            self.draw_trajectory()
        gc.collect()
        return
    

    """
    +++++ File -> Open -> Waypionts +++++
    """
    def menu_open_waypoints(self):
        if (self.waypoints is not None) and (self.master.title()[0] == "*"):
            yn = messagebox.askyesno("Confirm", "Do you want to save changes to " + str(self.waypoints_filepath))
            if yn: self.save_waypoints(str(self.waypoints_filepath)) # Yes
            self.master.title(str(self.master.title()).replace("* " + self.waypoints_filepath.name + " - ", ""))
        elif (self.waypoints is not None):
            self.master.title(str(self.master.title()).replace(self.waypoints_filepath.name + " - ", ""))

        filepath = tkinter.filedialog.askopenfilename(
            parent=self.master,
            title="Select waypoints yaml file",
            initialdir=str(Path(".")),
            filetypes=[("YAML", ".yaml")]
        )
        if not filepath: return
        with open(filepath) as file:
            wp_yaml = ruamel.yaml.YAML().load(file)
        if (not "waypoints" in wp_yaml.keys()) or (not "finish_pose" in wp_yaml.keys()):
            messagebox.showerror(title="Format error", message="Selected waypoints file is unexpected format.")
            return
        
        self.canvas.delete("all")
        self.trajectory = []
        self.draw_image()
        self.plot_origin()
        if (self.wp_info_win is not None) and (self.wp_info_win.winfo_exists()):
            self.wp_info_win.destroy()
            
        del self.waypoints
        self.waypoints = WaypointList(wp_yaml)
        self.finish_pose = FinishPose(wp_yaml)
        self.waypoints_filepath = Path(filepath)
        self.plot_waypoints()
        self.draw_trajectory()
        self.master.title(self.waypoints_filepath.name + " - " + self.master.title())
        self.message("Read waypoints file " + filepath)
        self.file_menu.entryconfigure("Save", state=tk.NORMAL)
        self.file_menu.entryconfigure("Save As", state=tk.NORMAL)
        self.popup_menu.entryconfigure("Add waypoint here", state=tk.NORMAL)
        gc.collect()
        return
    

    """
    +++++ File -> Save +++++
    """
    def menu_save(self, event=None):
        if not self.waypoints: return
        self.save_waypoints(str(self.waypoints_filepath))
        self.message("Saved changes!")
        title = self.master.title()
        if title[0] == "*": self.master.title(str(title).replace("* ", ""))
        return


    """
    +++++  File -> Save As +++++
    """
    def menu_saveas(self, event=None):
        if not self.waypoints: return
        new_filepath = tkinter.filedialog.asksaveasfilename(
            parent=self.master,
            title="Save As",
            initialdir=str(Path('..','..','waypoint_nav','param')),
            filetypes=[("YAML", ".yaml")],
            defaultextension=".yaml"
        )
        if len(new_filepath) == 0: return # cancel
        self.save_waypoints(new_filepath)
        current_title = self.master.title()
        old_filename = self.waypoints_filepath.name
        self.waypoints_filepath = Path(new_filepath)
        self.master.title(current_title.replace(old_filename, self.waypoints_filepath.name))
        self.message("Save As" + "\"" + str(new_filepath) + "\"")
        title = self.master.title()
        if title[0] == "*": self.master.title(str(title).replace("* ", ""))
        return


    """
    +++++  File -> Exit +++++
    """
    def menu_exit(self, event=None):
        title = self.master.title()
        if title[0] != "*":
            self.master.destroy()
            return
        res = messagebox.askyesnocancel(title="Unsaved changes", message="Do you want to save changes before close?")
        if res is None: return # cancel
        if res == True: self.menu_save()
        self.master.destroy()


    """
    +++++  現在のウェイポイント情報を指定のファイルに書き込む +++++
    """
    def save_waypoints(self, path):
        with open(path, 'w') as f:
            f.write(get_waypoint_yaml(self.waypoints, self.finish_pose))
        return
    

    """
    +++++  Edit -> Set Finish Pose +++++
    """
    def menu_set_finishpose(self, event=None):
        if not self.waypoints: return
        self.message("Click any point to set finsih pose. If you want to cancel, right click anywhere.")
        self.setting_finish_pose = 1
        return
    

    """
    +++++  Edit -> From To +++++
    """
    def menu_from_to(self, event=None):
        if not self.waypoints: return
        sub_win = tk.Toplevel()
        sub_win.title("Set parameter from some number's waypoint to any number's one")
        sub_win.attributes('-topmost', True)  # サブウィンドウを最前面で固定
        #
        font = ("Consolas", 12)
        frame1 = tk.Frame(sub_win)
        from_label = tk.Label(frame1, text="From No.", font=font)
        from_label.grid(row=0, column=0)
        from_txt_box = tk.Entry(frame1, width=4, font=font)
        from_txt_box.grid(row=0, column=1)
        to_label = tk.Label(frame1, text="to No.", font=font)
        to_label.grid(row=0, column=2)
        to_txt_box = tk.Entry(frame1, width=4, font=font)
        to_txt_box.grid(row=0, column=3)
        frame1.pack(padx=10, pady=10)
        #
        frame2 = tk.Frame(sub_win)
        name_label = tk.Label(frame2, text="Parameter", font=font)
        name_label.grid(row=0, column=0)
        value_label=tk.Label(frame2, text="Value", font=font)
        value_label.grid(row=0, column=1)
        name_txt_box = tk.Entry(frame2, width=20, font=font)
        name_txt_box.grid(row=1, column=0, padx=20)
        value_txt_box = tk.Entry(frame2, width=20, font=font)
        value_txt_box.grid(row=1, column=1, padx=20)
        frame2.pack(padx=10, pady=10)
        #
        def set_btn_callback():
            from_num = from_txt_box.get()
            to_num = to_txt_box.get()
            if (len(from_num) == 0) or (len(to_num) == 0):
                messagebox.showwarning(title="Warning", message="Enter waypoint numbers.")
                return
            from_num = int(from_num)
            to_num = int(to_num)
            if (from_num < 1) or (to_num > len(self.waypoints.get_waypoint())):
                messagebox.showerror(title="Error", message="Entered number is out of range of the number of waypoints")
                return
            param = name_txt_box.get()
            value = value_txt_box.get()
            if (len(param) == 0) or (len(value) == 0):
                messagebox.showwarning(title="Warning", message="Enter parameter name and value")
                return
            for n in range(from_num, to_num+1):
                self.waypoints.waypoints[n-1][param] = value
            sub_win.destroy()
            self.update_title()
            self.plot_waypoints()
            self.draw_trajectory()
            return
        frame3 = tk.Frame(sub_win)
        set_btn = tk.Button(frame3, text="Set", width=5, command=set_btn_callback)
        set_btn.pack(side=tk.RIGHT, padx=50, pady=10)
        cancel_btn = tk.Button(frame3, text="Cancel", width=8, command=sub_win.destroy)
        cancel_btn.pack(side=tk.LEFT, padx=50, pady=10)
        frame3.pack(padx=10, pady=20, expand=True, fill=tk.X)
        return


    """
    +++++  View -> Footprint -> Set +++++
    """
    def menu_set_footprint(self, event=None):
        sub_win = tk.Toplevel()
        sub_win.title("Set footprint model")
        sub_win.protocol("WM_DELETE_WINDOW")
        sub_win.attributes('-topmost', True)  # サブウィンドウを最前面で固定
        # テキストボックス
        footprint = str(self.footprint)
        txt_box = tk.Entry(sub_win, width=len(footprint)+10, font=("Consolas",12))
        txt_box.insert(tk.END, footprint)
        txt_box.pack(padx=10, pady=20)
        # Set ボタンコールバック
        def callback():
            try:
                self.footprint = eval(txt_box.get())
                if self.show_fp.get():
                    self.show_fp.set(False)
                    self.menu_show_footprint()
                self.show_fp.set(True)
                self.menu_show_footprint()
                sub_win.destroy()
            except SyntaxError:
                sub_win.title("Unexpectd format")
            return
        # Set ボタン
        set_btn = tk.Button(sub_win, text="Set", width=5, height=1, command=callback)
        set_btn.pack(pady=10)
        sub_win.update()
        w = sub_win.winfo_width()
        h = sub_win.winfo_height()
        x = int((self.canv_w - w) / 2)
        y = int((self.canv_h - h) / 2)
        geometry = "{}x{}+{}+{}".format(w, h, x, y)
        sub_win.geometry(geometry)
        return
    

    """
    +++++  View -> Footprint -> Show +++++
    """
    def menu_show_footprint(self, event=None):
        if not self.waypoints: return
        if event: self.show_fp.set(not self.show_fp.get())
        # create polygon as footprint
        def create_footprint(x, y, nx, ny):
            polygon = []
            th = math.atan2((ny - y), (nx -x))
            for xy in self.footprint:
                X = xy[0]*math.cos(th) - xy[1]*math.sin(th) + x
                Y = xy[0]*math.sin(th) + xy[1]*math.cos(th) + y
                cx, cy = self.real2canvas(float(X), float(Y))
                polygon.append([cx, cy])
            id = self.canvas.create_polygon(polygon, fill="", outline="green", tags="footprint")
            self.canvas.lift("footprint", "map_image")
            return
        ##
        if (self.show_fp.get()):
            for i in range(len(self.waypoints.get_waypoint()) - 1):
                wp = self.waypoints.get_waypoint(num=i+1)
                next_wp = self.waypoints.get_waypoint(num=i+2)
                create_footprint(float(wp["x"]), float(wp["y"]), float(next_wp["x"]), float(next_wp["y"]))
            # last waypoint
            create_footprint(float(next_wp["x"]), float(next_wp["y"]), self.finish_pose.x, self.finish_pose.y)
        else:
            self.canvas.delete("footprint")
        return



    """
    #################################
        原点とウェイポイントの表示
    #################################
    """
    """
    +++++ 地図上の原点に円を描画 +++++
    """
    def plot_origin(self):
        cx, cy = self.mymap.transform(self.mymap.img_origin[0], self.mymap.img_origin[1])
        r = self.point_rad   # 円の半径（ピクセル）
        x0 = cx - r
        y0 = cy - r
        x1 = cx + r + 1
        y1 = cy + r + 1
        if self.canvas.find_withtag("origin"):
            self.canvas.moveto("origin", x0, y0)
            self.trajectory[0] = [cx, cy]
        else:
            self.canvas.create_oval(x0, y0, x1, y1, tags="origin", fill='cyan', outline='blue')
            self.trajectory.append([cx, cy])
        return
    

    """
    +++++ 地図上にウェイポイントとフィニッシュポーズを描画 +++++
    """
    def plot_waypoints(self, id=None):
        if not self.waypoints: return
        # 引数にidが指定された場合、そのポイントのみを再描画して終了
        if id:
            wp = self.waypoints.get_waypoint(id)
            cx, cy = self.real2canvas(float(wp["x"]), float(wp["y"]))
            x0 = cx - self.point_rad
            y0 = cy - self.point_rad
            self.canvas.moveto(id, x0, y0)
            self.trajectory[self.waypoints.get_num(id)] = [cx, cy]
            return
        
        if len(self.waypoints.get_id_list()) == 0:
            # 初めて描画する場合
            for n, wp in enumerate(self.waypoints.get_waypoint()):
                id = self.create_waypoint(wp)
                self.waypoints.set_id(num=n+1, id=id)
        else:
            # 既に描画されている場合
            for id in self.waypoints.get_id_list():
                wp = self.waypoints.get_waypoint(id)
                cx, cy = self.real2canvas(float(wp["x"]), float(wp["y"]))
                x0 = cx - self.point_rad
                y0 = cy - self.point_rad
                self.canvas.moveto(id, x0, y0)
                self.trajectory[self.waypoints.get_num(id)] = [cx, cy]  # trajectory
        # Finish poseを描画
        cx, cy = self.real2canvas(self.finish_pose.x, self.finish_pose.y)
        x0 = cx
        y0 = cy
        x1 = x0 + math.cos(self.finish_pose.yaw) * self.point_rad * 3
        y1 = y0 - math.sin(self.finish_pose.yaw) * self.point_rad * 3
        if self.finish_pose.id is not None:
            # movetoだと上手くいかないので、毎回削除、再描画
            self.canvas.delete(self.finish_pose.id)
        self.finish_pose.id = self.canvas.create_line(x0, y0, x1, y1, tags="finish_pose",
                width=10, arrow=tk.LAST, arrowshape=(12,15,9), fill="#AAF"
        )
        # trajectory
        if self.canvas.find_withtag("trajectory"):
            self.trajectory[-1] = [cx, cy]
        else:
            self.trajectory.append([cx, cy])
        return
    

    """
    +++++ ウェイポイントをつないだ軌道を描画する +++++
    """
    def draw_trajectory(self):
        if self.canvas.find_withtag("trajectory"):
            self.canvas.coords("trajectory", list(itertools.chain.from_iterable(self.trajectory)))
        else:
            self.canvas.create_line(list(itertools.chain.from_iterable(self.trajectory)),
                    fill="#DF2", width=2, tags="trajectory")
            self.canvas.lift("trajectory", "map_image")
        return


    """
    +++++ キャンバスに新たなウェイポイントを描画する +++++
    """
    def create_waypoint(self, waypoint: dict):
        img_x, img_y = self.mymap.real2image(float(waypoint["x"]), float(waypoint["y"]))
        cx, cy = self.real2canvas(float(waypoint["x"]), float(waypoint["y"]))
        x0 = (cx - self.point_rad)
        y0 = (cy - self.point_rad)
        x1 = (cx + self.point_rad + 1)
        y1 = (cy + self.point_rad + 1)
        if (img_x < 0) or (img_y < 0) or (img_x > self.mymap.width()) or (img_y > self.mymap.height()):
            id = self.canvas.create_oval(x0, y0, x1, y1, fill='#FEE', outline='#FAA', activefill='#F88', tags="waypoints")
            self.canvas.tag_bind(id, "<Enter>", lambda event, wp_id=id: self.waypoint_enter(event, wp_id))
            self.canvas.tag_bind(id, "<Leave>", self.waypoint_leave)
        else:
            id = self.canvas.create_oval(x0, y0, x1, y1, fill='#FDD', outline='red', activefill='red', tags="waypoints")
            self.canvas.tag_bind(id, "<Button-1>", lambda event, wp_id=id: self.waypoint_clicked(event, wp_id))
            self.canvas.tag_bind(id, "<Enter>", lambda event, wp_id=id: self.waypoint_enter(event, wp_id))
            self.canvas.tag_bind(id, "<Leave>", self.waypoint_leave)
            self.canvas.tag_bind(id, "<B1-Motion>", self.waypoint_click_move)
        self.trajectory.append([cx, cy]) #trajectory
        return id
    

    """
    +++++ ウェイポイントが左クリックされたときのコールバック +++++
    """
    def waypoint_clicked(self, event, wp_id):
        if wp_id != self.editing_waypoint_id:   # 編集中のウェイポイントを切り替え
            self.canvas.itemconfig(self.editing_waypoint_id, fill='#FDD')
            self.editing_waypoint_id = wp_id
            self.canvas.itemconfig(wp_id, fill='red')
            self.moving_waypoint = False
            self.disp_waypoint_info(wp_id)
            self.message("Show selected waypoint information")
        return
    

    """
    +++++ ウェイポイントを左クリックしながら動かしたときのコールバック +++++
    """
    def waypoint_click_move(self, event):
        if not self.moving_waypoint: return
        if self.old_click_point is None:
            self.old_click_point = [event.x, event.y]
            return
        delta_x = event.x-self.old_click_point[0]
        delta_y = event.y-self.old_click_point[1]
        self.canvas.move(self.editing_waypoint_id, delta_x, delta_y)
        box = self.canvas.bbox(self.editing_waypoint_id)
        cx = (box[2] + box[0]) / 2  # ウィンドウ上の座標
        cy = (box[3] + box[1]) / 2
        img_x, img_y = self.mymap.inv_transform(cx, cy)
        # マップ画像上の座標を、実際の座標に変換
        x, y = self.mymap.image2real(img_x, img_y)
        # 編集中のウェイポイント情報を更新
        self.waypoints.set_waypoint_val(self.editing_waypoint_id, "x", x)
        self.waypoints.set_waypoint_val(self.editing_waypoint_id, "y", y)
        # 表示中のウェイポイント情報を更新
        txt_box: tk.Entry = self.wp_info_win.grid_slaves(column=1, row=0)[0]
        txt_box.delete(0, tk.END)
        txt_box.insert(tk.END, x)
        txt_box = self.wp_info_win.grid_slaves(column=1, row=1)[0]
        txt_box.delete(0, tk.END)
        txt_box.insert(tk.END, y)
        self.old_click_point = [event.x, event.y]
        self.update_title()
        # trajectory
        self.trajectory[self.waypoints.get_num(self.editing_waypoint_id)] = [cx, cy]
        self.draw_trajectory()
        return


    """
    +++++ ウェイポイントを示す円にカーソルが入ったときと出たときのコールバック +++++
    """
    def waypoint_enter(self, event, wp_id):
        wp_num = self.waypoints.get_num(wp_id)
        self.waypoint_num["text"] = " Waypoint No. {} " .format(str(wp_num))

    def waypoint_leave(self, event):
        self.waypoint_num["text"] = " Waypoint No. ----- "



    """
    ######################################
        別窓でのウェイポイントの情報表示
    ######################################
    """
    """
    +++++ ウェイポイントが左クリックされたとき、別窓で情報を表示する関数 +++++
    """
    def disp_waypoint_info(self, id):
        point: dict = self.waypoints.get_waypoint(id=id)
        if (self.wp_info_win is not None) and (self.wp_info_win.winfo_exists()):
            # 既にウィンドウが表示されている場合、一度削除
            self.wp_info_win.destroy()
        # ウィンドウが表示されてない場合、初期化
        self.wp_info_win = tk.Toplevel()
        self.wp_info_win.lower()
        self.wp_info_win.protocol("WM_DELETE_WINDOW", self.close_wp_info)
        # ウェイポイントファイルのキーを取得し、ラベルとテキストボックスを配置
        label_width = max([len(key) for key in point.keys()]) + 2
        for i, key in enumerate(point.keys()):
            key_label = tk.Label(self.wp_info_win, text=key+":", width=label_width, font=("Consolas",15), anchor=tk.E)
            key_label.grid(column=0, row=i, padx=2, pady=5)
            txt_box = tk.Entry(self.wp_info_win, width=20, font=("Consolas", 15))
            txt_box.insert(tk.END, str(point[key]).lower())
            txt_box.grid(column=1, row=i, padx=2, pady=2, ipady=3, sticky=tk.EW)
            del_btn = tk.Button(self.wp_info_win, image=self.del_param_icon, relief=tk.FLAT)
            del_btn["command"] = lambda name=key, val=str(point[key]).lower(): self.del_param_btn_callback(name, val)
            del_btn.grid(column=2, row=i, padx=5, pady=5)
        self.wp_info_win.grid_columnconfigure(1, weight=1)
        # New parameter
        new_param_btn = tk.Button(self.wp_info_win, image=self.new_param_icon, relief=tk.FLAT)
        new_param_btn["command"] = self.new_param_btn_callback
        new_param_btn.grid(column=0, columnspan=3, row=self.wp_info_win.grid_size()[1], pady=10)
        # Apply, Move(Drag & Drop), remove ボタン
        canv = tk.Canvas(self.wp_info_win)
        canv.grid(column=0, columnspan=3, row=self.wp_info_win.grid_size()[1], sticky=tk.EW, pady=5)
        apply_btn = tk.Button(canv, text="Apply", width=5, height=1, bg="#FDD",
                    command=self.apply_btn_callback)
        apply_btn.pack(side=tk.RIGHT, anchor=tk.SE, padx=5, pady=5)
        self.wp_info_win.bind('<KeyPress>', self.apply_btn_callback)
        move_btn = tk.Button(canv, text="Move", width=5, height=1, bg="#EEE")
        move_btn["command"] = lambda obj=move_btn: self.move_btn_callback(move_btn)
        move_btn.pack(side=tk.RIGHT, anchor=tk.SE, padx=5, pady=5)
        remove_btn = tk.Button(canv, text="Remove", width=7, height=1, bg="#F00",
                    command=self.remove_btn_callback)
        remove_btn.pack(side=tk.LEFT, anchor=tk.SE, padx=5, pady=5)
        # 位置とサイズを設定
        self.wp_info_win.update()
        w = self.wp_info_win.winfo_width()
        h = self.wp_info_win.winfo_height()
        x = self.canvas.winfo_x() + self.canv_w - w -10
        y = self.canvas.winfo_y() + self.canv_h - h -10
        self.wp_info_win.lift()
        self.wp_info_win.attributes('-topmost', True)  # サブウィンドウを最前面で固定
        self.wp_info_win.geometry("+{}+{}".format(x, y))
        self.wp_info_win.title("Waypoint " + str(self.waypoints.get_num(id)))   # タイトルを設定
        return


    """
    +++++ Applyボタンを押したときのコールバック +++++
    """
    def apply_btn_callback(self, event=None):
        if (event is not None) and (event.keysym != "Return"): return
        point = self.waypoints.get_waypoint(id=self.editing_waypoint_id)
        for i, key in enumerate(point.keys()):
            txt_box = self.wp_info_win.grid_slaves(column=1, row=i)[0]
            val = txt_box.get()
            if (str(point[key]) == val): continue
            self.update_title()
            self.message("Apply changes of waypoint parameters")
            self.waypoints.set_waypoint_val(self.editing_waypoint_id, key, val)
            if ((key == "x") or (key == "y")):
                self.plot_waypoints(self.editing_waypoint_id)
                self.draw_trajectory()
        return


    """
    +++++ ドラッグ&ドロップボタン(Moveボタン)を押したときのコールバック +++++
    """
    def move_btn_callback(self, obj=None):
        if obj is None: return
        btn = obj
        # 押された状態とそうでない状態を切り替える
        if btn["relief"] == tk.RAISED:
            btn["relief"] = tk.SUNKEN
            btn["bg"] = "#AAA"
            self.moving_waypoint = True
            self.message("Drag & Drop to move waypoint")
        elif btn["relief"] == tk.SUNKEN:
            btn["relief"] = tk.RAISED
            btn["bg"] = "#EEE"
            self.moving_waypoint = False
            self.message("Show selected waypoint information")
        return


    """
    +++++ removeボタンを押したときのコールバック +++++
    """
    def remove_btn_callback(self):
        yn = messagebox.askyesno("Delete parameter", message="Are you sure you want to remove this waypoint?")
        if (yn == True):
            self.trajectory.pop(self.waypoints.get_num(self.editing_waypoint_id))
            self.waypoints.remove(self.editing_waypoint_id)
            self.canvas.delete(self.editing_waypoint_id)  # ウェイポイントを示す円を削除
            self.close_wp_info()
            self.message("Removed waypoint")
            self.update_title()
            self.draw_trajectory()
        return
    

    """
    +++++ New Parameter ボタンを押したときのコールバック +++++
    """
    def new_param_btn_callback(self):
        # Sub window
        sub_win = tk.Toplevel()
        sub_win.title("Add new parameter")
        sub_win.attributes('-topmost', True)
        font = ("Consolas",15)
        # Message label
        label = tk.Label(sub_win, text="Name : Value", font=font, anchor=tk.CENTER, width=20)
        label.grid(row=0, column=0, columnspan=3, padx=10, pady=10)
        # Text box
        name_entry = tk.Entry(sub_win, font=font, width=15)
        name_entry.grid(column=0, row=1, padx=10, pady=5)
        colon = tk.Label(sub_win, text=":", font=font, anchor=tk.CENTER, width=2)
        colon.grid(row=1, column=1)
        value_entry = tk.Entry(sub_win, font=font, width=15)
        value_entry.grid(column=2, row=1, padx=10, pady=5)
        # Buttons
        def add_param():
            name = name_entry.get()
            value = value_entry.get()
            self.waypoints.set_waypoint_val(self.editing_waypoint_id, key=name, val=value)
            sub_win.destroy()
            self.wp_info_win.destroy()
            self.disp_waypoint_info(self.editing_waypoint_id)
            self.update_title()
            gc.collect()
            return
        add_btn = tk.Button(sub_win, text="Add", width=5, height=1, font=font, anchor=tk.CENTER, bg="#AFA")
        add_btn["command"] = add_param
        add_btn.grid(column=2, row=2, pady=20)
        cancel_btn = tk.Button(sub_win, text="Cancel", width=7, height=1, font=font, anchor=tk.CENTER)
        cancel_btn["command"] = sub_win.destroy
        cancel_btn.grid(column=0, row=2, pady=20)
        # Window position
        sub_win.update()
        w = sub_win.winfo_width() + 10
        h = sub_win.winfo_height()
        x = int((self.canv_w - w) / 2)
        y = int((self.canv_h - h) / 2)
        sub_win.geometry("{}x{}+{}+{}".format(w, h, x, y))
        return
    

    def del_param_btn_callback(self, name, val):
        msg = "Are you sure you want to delete this parameter?\n\n"
        msg += str(name) + ": " + str(val)
        yn = messagebox.askyesno("Delete parameter", message=msg)
        if (yn == True):
            self.waypoints.delete_waypoint_param(self.editing_waypoint_id, name)
            self.wp_info_win.destroy()
            self.disp_waypoint_info(self.editing_waypoint_id)
            self.update_title()
        return


    """
    +++++ ウェイポイント情報を表示するサブウィンドウを閉じたときのコールバック +++++
    """
    def close_wp_info(self):
        self.canvas.itemconfig(self.editing_waypoint_id, fill='#FDD')
        self.editing_waypoint_id = None
        self.moving_waypoint = False
        self.wp_info_win.destroy()
        gc.collect()
        return



    """
    ###########################
        ウェイポイントの追加
    ###########################
    """
    """
    +++++ 右クリックしてポップアップメニューのadd waypoint hereをクリックしたときのコールバック関数 +++++
    """
    def add_waypoint_here(self):
        if (self.wp_info_win is not None) and (self.wp_info_win.winfo_exists()):
            self.close_wp_info()
        img_x, img_y = self.right_click_coord
        if self.mymap.pil_img.getpixel((img_x, img_y))[0] == 0:
            messagebox.showwarning(title="Warning", message="There is obstacles")
            return
        # 何番目のウェイポイントの次に追加するか入力させる
        add_wp_win = tk.Toplevel()
        add_wp_win.title("Add waypoint")
        add_wp_win.protocol("WM_DELETE_WINDOW")
        add_wp_win.attributes('-topmost', True)  # サブウィンドウを最前面で固定
        msg = tk.Label(add_wp_win, text="Add new waypoint at no. ", font=("Consolas",15), anchor=tk.E)
        msg.grid(column=0, row=0, padx=10, pady=10, sticky=tk.EW)
        txt_box = tk.Entry(add_wp_win, width=4, font=("Consolas",15))
        txt_box.grid(column=1, row=0, pady=10, sticky=tk.W)
        # Add ボタン
        add_btn = tk.Button(add_wp_win, text="Add", width=5, height=1,
            command=lambda num_box=txt_box, win=add_wp_win: self.add_btn_callback(num_box, win)
        )
        add_btn.grid(column=0, columnspan=2, row=1, padx=10, pady=10)
        add_wp_win.update()
        w = add_wp_win.winfo_width() + 10
        h = add_wp_win.winfo_height()
        x = int((self.canv_w - w) / 2)
        y = int((self.canv_h - h) / 2)
        geometry = "{}x{}+{}+{}".format(w, h, x, y)
        add_wp_win.geometry(geometry)
        self.message("Add waypoint")
        return
    

    """
    +++++ add waypoint hereをクリックして開いた別窓のAddボタンのコールバック +++++
    """
    def add_btn_callback(self, num_box: tk.Entry, win: tk.Toplevel):
        num = num_box.get()
        if (num == ""):
            win.attributes('-topmost', False)
            messagebox.showwarning(title="Warning", message="The number has not been entered.")
            win.attributes('-topmost', True)
            return
        num = int(num)
        if (num < 1) or (num > len(self.waypoints.waypoints)+1):
            win.attributes('-topmost', False)
            messagebox.showwarning(title="Warning", message="The number is out of range.")
            win.attributes('-topmost', True)
            return
        win.destroy()
        img_xy = self.right_click_coord
        # ウェイポイント座標を計算
        x, y = self.mymap.image2real(img_xy[0], img_xy[1])
        # ウェイポイントを追加
        point = {}
        for key in self.waypoints.point_keys:
            if (key=="x"): point[key] = x
            elif (key=="y"): point[key] = y
            elif (key=="z"): point[key] = 0.0
            else: point[key] = ""
        id = self.create_waypoint(point)
        cx,cy = self.trajectory.pop(-1)
        self.trajectory[num:num] = [[cx, cy]]
        self.waypoints.insert(num, point, id=id)
        self.plot_waypoints(id=id)
        self.draw_trajectory()
        self.editing_waypoint_id = id
        self.canvas.itemconfig(id, fill='red')
        self.disp_waypoint_info(id)
        self.update_title()
        return
    


    """
    ###############################
        イベントコールバック関数
    ###############################
    """
    """
    +++++ キャンバス内でマウスを動かしたとき +++++
    """
    def mouse_move(self, event):
        if not self.mymap: return

        if self.setting_finish_pose == 2:
            x0, y0, _, _ = self.canvas.coords("set_finish_pose")
            x, y = event.x, event.y
            theta = math.atan2((-y+y0), (x-x0))
            x1 = x0 + math.cos(theta) * self.point_rad * 3
            y1 = y0 - math.sin(theta) * self.point_rad * 3
            self.canvas.delete("set_finish_pose")
            self.canvas.create_line(x0, y0, x1, y1, tags="set_finish_pose",
                width=10, arrow=tk.LAST, arrowshape=(12,15,9), fill="#F88"
            )
        
        img_x, img_y = self.mymap.inv_transform(event.x, event.y)
        if (img_x < 0) or (img_y < 0) or (img_x > self.mymap.width()) or (img_y > self.mymap.height()):
            self.mouse_position["text"] = " Out of map "
            return
        x, y = self.mymap.image2real(img_x, img_y)
        self.mouse_position["text"] = " ( x, y ) = ( {},  {} ) ".format(x, y)
        return
    

    """
    +++++ 左クリックされたとき +++++
    """
    def left_click(self, event):
        self.popup_menu.unpost()   # 右クリックで出るポップアップメニューを非表示

        if self.setting_finish_pose == 1:
            x0, y0 = event.x, event.y
            img_x, img_y = self.mymap.inv_transform(x0, y0)
            if (img_x < 0) or (img_y < 0) or (img_x > self.mymap.width()) or (img_y > self.mymap.height()):
                retry = messagebox.askretrycancel(title="Cannot set finish pose here", message="The point is out of the map.")
                if not retry: self.setting_finish_pose = 0
                return
            x1 = x0 + math.cos(self.finish_pose.yaw) * self.point_rad * 3
            y1 = y0 - math.sin(self.finish_pose.yaw) * self.point_rad * 3
            self.canvas.create_line(x0, y0, x1, y1, tags="set_finish_pose",
                width=10, arrow=tk.LAST, arrowshape=(12,15,9), fill="#F88"
            )
            self.setting_finish_pose = 2
            return
        
        if self.setting_finish_pose == 2:
            x0, y0, _, _ = self.canvas.coords("set_finish_pose")
            x, y = event.x, event.y
            theta = math.atan2((-y+y0), (x-x0))
            x1 = x0 + math.cos(theta) * self.point_rad * 3
            y1 = y0 - math.sin(theta) * self.point_rad * 3
            self.canvas.delete("set_finish_pose")
            self.canvas.delete(self.finish_pose.id)
            self.finish_pose.id = self.canvas.create_line(x0, y0, x1, y1, tags="finish_pose",
                width=10, arrow=tk.LAST, arrowshape=(12,15,9), fill="#AAF"
            )
            self.trajectory[-1] = [x0, y0]
            self.draw_trajectory()
            img_x, img_y = self.mymap.inv_transform(x0, y0)
            real_x, real_y = self.mymap.image2real(img_x, img_y)
            self.finish_pose.x = real_x
            self.finish_pose.y = real_y
            self.finish_pose.yaw = theta
            self.setting_finish_pose = 0
            self.message("New finish pose is set.")
            self.update_title()
        return
    

    """
    +++++ マウスを左クリックしながらドラッグしたとき +++++
    """
    def left_click_move(self, event):
        if not self.mymap: return
        if self.moving_waypoint: return
        if self.setting_finish_pose != 0: return
        if self.old_click_point is None:
            self.old_click_point = [event.x, event.y]
            return
        # カーソルの移動量を計算
        delta_x = event.x - self.old_click_point[0]
        delta_y = event.y - self.old_click_point[1]
        # ウェイポイント移動モードでないとき、地図を平行移動
        self.mymap.translate(delta_x, delta_y)
        self.draw_image()
        # origin, waypoints finish_pose を平行移動
        self.canvas.move("origin", delta_x, delta_y)
        if self.waypoints:
            self.canvas.move("waypoints", delta_x, delta_y)
            self.canvas.move("finish_pose", delta_x, delta_y)
            self.canvas.move("trajectory", delta_x, delta_y)
            self.canvas.lift("trajectory", "map_image")
            self.canvas.move("footprint", delta_x, delta_y)
            for i in range(0, len(self.trajectory)):
                self.trajectory[i][0] = self.trajectory[i][0] + delta_x
                self.trajectory[i][1] = self.trajectory[i][1] + delta_y
        self.old_click_point = [event.x, event.y]
        return
    

    """
    +++++ マウスの左クリックを離したとき +++++
    """
    def left_click_release(self, event):
        self.old_click_point = None
        return
    

    """
    +++++ 右クリックしたとき +++++
    """
    def right_click(self, event):
        if not self.mymap: return
        if self.setting_finish_pose == 1:
            self.setting_finish_pose = 0
            self.message("Canceled.")
            return
        if self.setting_finish_pose == 2:
            self.canvas.delete("set_finish_pose")
            self.setting_finish_pose = 0
            self.message("Canceled.")
            return
        # クリックした座標の近くにあるオブジェクトを取得
        clicked_obj = self.canvas.find_enclosed(event.x-20, event.y-20, event.x+20, event.y+20)
        if clicked_obj:   # 何かオブジェクトがクリックされていた場合
            return
        # クリックされた座標 => 元画像の座標 の変換
        img_x, img_y = self.mymap.inv_transform(event.x, event.y)
        # 変換後の元画像の座標がサイズから外れている場合（地図画像の外をクリックしている）
        if (img_x < 0) or (img_y < 0) or (img_x > self.mymap.width()) or (img_y > self.mymap.height()):
            return
        self.popup_menu.post(event.x_root, event.y_root)   # メニューをポップアップ
        self.right_click_coord = [img_x, img_y]   # クリックされた元画像上の座標を変数に格納
        return
    

    """
    +++++ Ctrl押しながら左クリックしたとき +++++
    """
    def ctrl_left_click(self, event):
        if not self.mymap: return
        if self.setting_finish_pose != 0: return
        scale = 1.2
        self.mymap.scale_at(event.x, event.y, scale)
        self.draw_image()
        self.plot_origin()
        self.plot_waypoints()
        self.canvas.scale("footprint", event.x, event.y, scale, scale)
        self.canvas.scale("trajectory", event.x, event.y, scale, scale)
        self.message("Zoom In")
        return


    """
    +++++ Ctrl押しながら右クリックしたとき +++++
    """
    def ctrl_right_click(self, event):
        if not self.mymap: return
        if self.setting_finish_pose != 0: return
        scale = 0.8
        self.mymap.scale_at(event.x, event.y, scale)
        self.draw_image()
        self.plot_origin()
        self.plot_waypoints()
        self.canvas.scale("footprint", event.x, event.y, scale, scale)
        self.canvas.scale("trajectory", event.x, event.y, scale, scale)
        self.message("Zoom Out")
        return
    

    """
    +++++ マウスホイールを回転したとき（タッチパッドをドラッグしたとき） +++++
    """
    def mouse_wheel(self, event):
        if not self.mymap: return
        if event.delta > 0:
            scale = 1.1   # 上に回転（タッチパッドなら下にドラッグ）=> 拡大
        else:
            scale = 0.9   # 下に回転（タッチパッドなら上にドラッグ）=> 縮小
        self.mymap.scale_at(event.x, event.y, scale)
        self.draw_image()
        self.plot_origin()
        self.plot_waypoints()
        self.canvas.scale("footprint", event.x, event.y, scale, scale)
        self.canvas.scale("trajectory", event.x, event.y, scale, scale)
        return


    """
    +++++ ウィンドウサイズが変更されたとき、情報を更新する +++++
    """
    def window_resize_callback(self, event):
        cw = self.canvas.winfo_width()
        ch = self.canvas.winfo_height()
        if (self.canv_w != cw) or (self.canv_h != ch):
            self.canv_w = cw
            self.canv_h = ch
            self.draw_image()
        return
    



    """
    ##############################################
        地図画像の表示、メッセージ表示、座標変換
    ##############################################
    """
    """
    +++++ 元画像をaffne変換して描画 +++++
    """
    def draw_image(self):
        if not self.mymap: return
        if not self.canvas.find_withtag("map_image"):   # 初めて画像を描画するとき
            self.canvas.create_image(0, 0, anchor='nw', tags="map_image",
                image=self.mymap.get_draw_image((self.canv_w, self.canv_h))
            )
        else:
            # 既に描画された画像を差し替える
            self.canvas.itemconfig("map_image", image=self.mymap.get_draw_image((self.canv_w, self.canv_h)))
        self.canvas.tag_lower("map_image")
        return
    

    """
    +++++ 画面上部にメッセージを表示する +++++
    """
    def message(self, msg):
        if not isinstance(msg, str):
            msg = str(msg)
        self.msg_label["text"] = str(msg)


    """
    +++++ 実際の座標をキャンバス上の座標に変換 +++++
    """
    def real2canvas(self, x, y):
        img_x, img_y = self.mymap.real2image(x,y)
        cx, cy = self.mymap.transform(img_x, img_y)
        return round(cx), round(cy)
    

    def update_title(self):
        title = self.master.title()
        if title[0] != "*": 
            self.master.title("* "+title)
    





if __name__ == "__main__":
    root = tk.Tk()   # 大元になるウィンドウ
    w, h = root.winfo_screenwidth()-10, root.winfo_screenheight()-100
    root.geometry("%dx%d+0+0" % (w, h))
    app = Application(master=root)   # tk.Frameを継承したApplicationクラスのインスタンス
    root.protocol("WM_DELETE_WINDOW", app.menu_exit)
    try:
        app.mainloop()
    except KeyboardInterrupt as e:
        app.menu_exit()

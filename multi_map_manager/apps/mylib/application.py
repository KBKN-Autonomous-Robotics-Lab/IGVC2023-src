import tkinter as tk
import tkinter.filedialog
from pathlib import Path
from .tools import Tools



class Application(tk.Frame):

    def __init__(self, master):
        super().__init__(master)
        self.theme = {"main":  "#444",
                      "bg1":   "#222"}

        #### 画面上部のメニューバーを作成 ####
        self.menu_bar = tk.Menu(self)
        ## Fileメニューの作成
        self.file_menu = tk.Menu(self.menu_bar, tearoff=tk.OFF,
            bg=self.theme["main"], fg="white", activebackground="gray", activeborderwidth=5
        )
        ## Fileメニュー内のOpenメニューを作成
        self.open_menu = tk.Menu(self.file_menu, tearoff=tk.OFF,
            bg=self.theme["main"], fg="white", activebackground="gray", activeborderwidth=5,
            disabledforeground="black"
        )
        self.open_menu.add_command(label="Base map",       command=self.menu_open_base)
        self.open_menu.add_command(label="Additional map", command=self.menu_open_addtion, state="disabled")
        self.open_menu.add_command(label="Multimaps dir",  command=self.menu_open_multi)
        self.file_menu.add_cascade(label="Open", menu=self.open_menu)
        ## Fileメニューその他
        self.file_menu.add_command(label="Export", command=self.menu_export, accelerator="Ctrl+E")
        self.file_menu.add_separator()
        self.file_menu.add_command(label="Exit",    command=self.menu_exit,   accelerator="Ctrl+Q")
        ## キーボードショートカットを設定
        self.bind_all("<Control-e>",       self.menu_export)
        self.bind_all("<Control-q>",       self.menu_exit)
        ## 大元に作成したメニューバーを設定
        self.menu_bar.add_cascade(label=" File ", menu=self.file_menu) # Fileメニューとしてバーに追加
        self.master.configure(menu=self.menu_bar)

        #### 仕切り線で大きさを変更できるウィンドウ ####
        paned_window = tk.PanedWindow(self.master, sashwidth=3, bg="gray", relief=tk.RAISED)
        paned_window.pack(expand=True, fill=tk.BOTH)
        
        #### ツールボタン群とレイヤーリストを表示するフレーム ####
        self.tools = Tools(paned_window, self.theme, width=300, bg=self.theme["main"])
        paned_window.add(self.tools, minsize=self.tools.min_width, padx=2, pady=2)
        ## マップ画像を表示するフレームを追加
        paned_window.add(self.tools.map_disp, minsize=self.tools.map_disp.min_width, padx=2, pady=2)
        return


    """
    ++++++++++ File menu functions ++++++++++
    """
    def menu_open_base(self, event=None):
        map_path = self.open_yaml(title="Select base map yaml file")
        if not map_path: return
        waypoints_path = self.open_yaml(title="Select waypoints file for the map")
        if not waypoints_path: return

        suc = self.tools.set_base_map(Path(map_path).resolve(), Path(waypoints_path).resolve())
        if not suc: return
        self.open_menu.entryconfigure("Base map", state="disabled")
        self.open_menu.entryconfigure("Multimaps dir", state="disabled")
        self.open_menu.entryconfigure("Additional map", state="normal")
        return
    

    def menu_open_addtion(self, event=None):
        map_path = self.open_yaml(title="Select additional map yaml file")
        if not map_path: return
        waypoints_path = self.open_yaml(title="Select waypoints file for the map")
        if not waypoints_path: return
        self.tools.add_map(Path(map_path).resolve(), Path(waypoints_path).resolve())
        return
    

    def menu_open_multi(self, event=None):
        dirpath = tkinter.filedialog.askdirectory(
            parent=self.master,
            title="Select multi maps directory",
            initialdir=str(Path("."))
        )
        if not dirpath: return
        dirpath = Path(dirpath)
        if not (dirpath / Path("map0.yaml")).exists(): return
        waypoints_path = self.open_yaml(title="Select waypoints file for the map")
        if not waypoints_path: return
        suc = self.tools.set_multimaps(dirpath, Path(waypoints_path).resolve())
        if not suc: return
        self.open_menu.entryconfigure("Base map", state="disabled")
        self.open_menu.entryconfigure("Multimaps dir", state="disabled")
        self.open_menu.entryconfigure("Additional map", state="normal")
        return
    

    def open_yaml(self, title):
        filepath = tkinter.filedialog.askopenfilename(
            parent=self.master,
            title=title,
            initialdir=str(Path(".")),
            filetypes=[("YAML", ".yaml")]
        )
        return filepath
    

    def menu_export(self, event=None):
        if (len(self.tools.label_list) < 2): return
        ## サブウィンドウを作成
        win = tk.Toplevel()
        win.geometry("700x200+50+50")
        win.minsize(width=500, height=200)
        win.attributes('-topmost', True)
        win.title("Export")
        font = ("Arial", 12)

        ### ファルダパスを参照するダイアログを開く関数（ボタンコールバック） ###
        def ref_btn_callback_d(entry: tk.Entry, init_dir):
            dirpath = tkinter.filedialog.askdirectory(
                parent=win,
                title="Directory path to export maps",
                initialdir=init_dir
            )
            if not dirpath: return
            entry.delete(0, tk.END)
            entry.insert(tk.END, str(dirpath))
        
        ## サブウィンドウにフォルダ入力エントリーを作成
        frame = tk.Frame(win)
        frame.pack(expand=False, fill=tk.X, padx=5, pady=10)
        label = tk.Label(frame, text="Directory :", anchor=tk.W, font=font)
        label.grid(column=0, row=0, padx=3, pady=2, sticky=tk.EW)
        ref_btn = tk.Button(frame, image=self.tools.folder_icon)
        ref_btn.grid(column=1, row=1, sticky=tk.E, padx=5)
        tbox = tk.Entry(frame, font=font)
        tbox.grid(column=0, row=1, padx=20, pady=2, sticky=tk.EW)
        init_dir = str(Path(".").resolve())
        tbox.insert(tk.END, init_dir)
        ref_btn["command"] = lambda entry=tbox, init_dir=init_dir: ref_btn_callback_d(entry, init_dir)
        frame.grid_columnconfigure(0, weight=1)

        ##　マルチマップyaml、結合したウェイポイント、合成した地図画像と情報yamlを取得
        map_img_list, map_yaml_list = self.tools.get_map_lists()
        wp_yaml, _ = self.tools.get_waypoints_yaml()
        merged_img_pil, merged_yaml = self.tools.get_merged_map()

        ### それぞれをファイルに書き込む関数（ボタンコールバック） ###
        def export_btn_callback():
            # multi maps
            dir_path = Path(tbox.get()).resolve()
            if not dir_path.exists(): dir_path.mkdir()
            for i, img in enumerate(map_img_list):
                img_path = dir_path / Path("map{}.pgm".format(i))
                img.save(str(img_path.resolve()))
                map_yaml_list[i]["image"] = "./"+str(img_path.name)
                str_yaml = ""
                line = "\n"
                for key, val in map_yaml_list[i].items():
                    str_yaml += "{}: {}".format(key, val) + line
                with open(str(img_path.with_suffix(".yaml")), 'w') as f:
                    f.write(str_yaml)

            # merged waypoints
            wp_filepath = dir_path / Path("waypoints.yaml")
            with open(wp_filepath, 'w') as f:
                f.write(wp_yaml)
            
            # merged map (yaml, pgm)
            yaml_path = dir_path / Path("merged_map.yaml")
            img_path = yaml_path.with_suffix(".pgm")
            merged_yaml["image"] = "./" + str(img_path.name)
            str_yaml = ""
            line = "\n"
            for key, val in merged_yaml.items():
                str_yaml += "{}: {}".format(key, val) + line
            with open(str(yaml_path), 'w') as f:
                f.write(str_yaml)
            merged_img_pil.save(str(img_path))
            win.destroy()
            return

        ## エクスポートボタン、キャンセルボタン
        export_btn = tk.Button(win, text="Export", font=font)
        export_btn["command"] = export_btn_callback
        export_btn.pack(side=tk.RIGHT, padx=50, pady=20)
        cancel_btn = tk.Button(win, text="Cancel", font=font)
        cancel_btn["command"] = win.destroy
        cancel_btn.pack(side=tk.LEFT, padx=50, pady=20)
        return
    

    def menu_exit(self, event=None):
        self.master.destroy()


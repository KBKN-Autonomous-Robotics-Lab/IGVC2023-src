import tkinter as tk
import tkinter.filedialog
import ruamel.yaml
from pathlib import Path
from mylib.mapdisp import MyMap
from PIL import Image, ImageDraw


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
        self.menu_bar.add_cascade(label=" File ", menu=self.file_menu)
        ## File -> Open
        self.file_menu.add_command(label="Open", command=self.menu_open, accelerator="Ctrl+O")
        ## File -> Save As
        self.file_menu.add_command(label="Save As", command=self.menu_saveas, accelerator="Ctrl+Shift+S")
        ## File -> Exit メニュー
        self.file_menu.add_separator() # 仕切り
        self.file_menu.add_command(label="Exit", command=self.menu_exit, accelerator="Ctrl+Q")
        ## Edit メニューの作成
        self.edit_menu = tk.Menu(self.menu_bar, tearoff=tk.OFF,
            bg=self.theme["main"], fg="white", activebackground="gray", activeborderwidth=5
        )
        self.menu_bar.add_cascade(label=" Edit ", menu=self.edit_menu)
        ## Edit -> Trim
        self.edit_menu.add_command(label="Trim", command=self.menu_trim, accelerator="Ctrl+T")

        #### メニューバーを設定 ####
        self.master.configure(menu=self.menu_bar)

        #### キーバインド ####
        self.bind_all("<Control-t>", self.menu_trim)
        self.bind_all("<Control-Shift-S>", self.menu_saveas)
        self.bind_all("<Control-o>", self.menu_open)
        self.bind_all("<Control-q>", self.menu_exit)

        #### マップを表示するキャンバス ####
        self.canvas = tk.Canvas(self.master, bg="black", highlightthickness=0)
        self.canvas.pack(expand=True, fill=tk.BOTH, padx=5, pady=5)
        self.update()
        self.canv_w = self.canvas.winfo_width()
        self.canv_h = self.canvas.winfo_height()

        #### マウスイベントコールバック ####
        self.canvas.bind("<B1-Motion>",        self.left_click_move)
        self.canvas.bind("<Button-1>",         self.left_click)
        self.canvas.bind("<ButtonRelease-1>",  self.left_click_release)
        self.canvas.bind("<Control-Button-1>", lambda event, scale=1.1: self.ctrl_click(event, scale))
        self.canvas.bind("<Control-Button-3>", lambda event, scale=0.9: self.ctrl_click(event, scale))
        self.canvas.bind("<Configure>",        self.resize_callback)

        #### 変数 ####
        self.mymap = None
        self.origin_img = []
        self.trim_range = []
        self.trimming_mode = False
        return
    


    """
    ++++++++++ File menu functions ++++++++++
    """
    def menu_open(self, event=None):
        filepath = tkinter.filedialog.askopenfilename(
            parent=self.master,
            title="Select map yaml file",
            initialdir=str(Path(".")),
            filetypes=[("YAML", ".yaml")]
        )
        if not filepath: return
        with open(filepath) as file:   # .yamlを読み込む
            map_yaml = ruamel.yaml.YAML().load(file)
        
        self.mymap = MyMap(Path(filepath).resolve(), map_yaml)
        img_w = self.mymap.original_img_pil.width
        img_h = self.mymap.original_img_pil.height
        scale = 1
        offset_x = 0
        offset_y = 0
        if (self.canv_w / self.canv_h) > (img_w / img_h):
            # canvasの方が横長　画像をcanvasの縦に合わせてリサイズ
            scale = (self.canv_h) / img_h
            offset_x = (self.canv_w - img_w*scale) / 2
        else:
            # canvasの方が縦長　画像をcanvasの横に合わせてリサイズ
            scale = (self.canv_w) / img_w
            offset_y = (self.canv_h - img_h*scale) / 2

        self.mymap.scale_at(0, 0, scale)
        self.mymap.translate(offset_x, offset_y)
        self.canvas.create_image(0, 0, anchor=tk.NW, tags="map",
            image=self.mymap.get_draw_image((self.canv_w, self.canv_h))
        )
        self.trim_range = [0, 0, img_w, img_h]  # [x1, y1, x2, y2] 画像における左上のxy, 右下のxy
        origin = self.mymap.origin
        resolution = self.mymap.resolution
        self.origin_img = [-origin[0]/resolution, origin[1]/resolution+self.mymap.original_img_pil.height]
        self.plot_origin()
        self.master.title(str(filepath))
        return
    

    def menu_saveas(self, event=None):
        new_filepath = tkinter.filedialog.asksaveasfilename(
            parent=self.master,
            title="Save map YAML and PGM files.",
            initialdir=str(self.mymap.yaml_path),
            filetypes=[("PGM YAML", ".pgm .yaml")],
        )
        if len(new_filepath) == 0: return
        # origin
        origin = [(self.trim_range[0]-self.origin_img[0])*self.mymap.resolution,
                  (-self.trim_range[3]+self.origin_img[1])*self.mymap.resolution,
                  0.0]
        # Write map yaml file
        yaml_path = Path(new_filepath).with_suffix(".yaml")
        line = "\n"
        yaml = ["image: ./" + yaml_path.with_suffix(".pgm").name + line]
        yaml.append("resolution: " + str(self.mymap.resolution) + line)
        yaml.append("origin: " + str(origin) + line)
        yaml.append("negate: " + str(self.mymap.map_yaml["negate"]) + line)
        yaml.append("occupied_thresh: " + str(self.mymap.map_yaml["occupied_thresh"]) + line)
        yaml.append("free_thresh: " + str(self.mymap.map_yaml["free_thresh"]) + line)
        with open(yaml_path.resolve(), 'w') as f:
            f.write("".join(yaml))
        # Trim and save map image
        trimmed_img = self.mymap.original_img_pil.crop(tuple(self.trim_range))
        trimmed_img.save(str(yaml_path.with_suffix(".pgm")))
        return
    
    
    def menu_exit(self, event=None):
        self.master.destroy()
    


    """
    ++++++++++ Edit menu functions ++++++++++
    """
    def menu_trim(self, event=None):
        if not self.mymap: return
        if self.trimming_mode:
            self.canvas.delete("upper", "right", "lower", "left", "ul", "ur", "lr", "ll")
            self.trimming_mode = False
            self.set_alpha(0)
            return
        self.trimming_mode = True
        self.set_alpha(128)
        r = 10
        ul_x, ul_y = self.mymap.transform(self.trim_range[0], self.trim_range[1])
        lr_x, lr_y = self.mymap.transform(self.trim_range[2], self.trim_range[3])
        cxy = [ul_x, ul_y, lr_x, ul_y, lr_x, lr_y, ul_x, lr_y]
        # トリミング範囲の上下左右に直線を描画
        for tag, x1, y1, x2, y2 in [("upper",0,1,2,3), ("right",2,3,4,5), ("lower",4,5,6,7), ("left",6,7,0,1)]:
            self.canvas.create_line(cxy[x1], cxy[y1], cxy[x2], cxy[y2], tags=tag, fill="#FFF", width=2)
        #　四隅にマーカーを描画
        for tag, xidx, yidx in [("ul",0,1), ("ur",2,3), ("lr",4,5), ("ll",6,7)]:
            cx, cy = cxy[xidx], cxy[yidx]
            self.canvas.create_oval(cx-r, cy-r, cx+r, cy+r, tags=tag, fill="#BBB", outline="#FFF", activefill="#FFF")
            self.canvas.tag_bind(tag, "<B1-Motion>", lambda event, tag=tag: self.move_trim_range(event, tag))
        return
    

    def move_trim_range(self, event, tag):
        if self.old_click_point is None:
            self.old_click_point = [event.x, event.y]
            return
        delta_x = event.x - self.old_click_point[0]
        delta_y = event.y - self.old_click_point[1]
        self.old_click_point = [event.x, event.y]
        # 四隅の円の移動
        self.canvas.move(tag, delta_x, delta_y)
        if tag == "ul":
            self.canvas.move("ur", 0, delta_y)
            self.canvas.move("ll", delta_x, 0)
        elif tag == "ur":
            self.canvas.move("ul", 0, delta_y)
            self.canvas.move("lr", delta_x, 0)
        elif tag == "lr":
            self.canvas.move("ll", 0, delta_y)
            self.canvas.move("ur", delta_x, 0)
        else: #  "ll"
            self.canvas.move("lr", 0, delta_y)
            self.canvas.move("ul", delta_x, 0)
        # 左上のマーカーの座標を取得
        ul = self.canvas.bbox("ul")
        ul_x = (ul[0] + ul[2]) / 2
        ul_y = (ul[1] + ul[3]) / 2
        # 右下の座標
        lr = self.canvas.bbox("lr")
        lr_x = (lr[0] + lr[2]) / 2
        lr_y = (lr[1] + lr[3]) / 2
        # マーカー間の線を移動
        self.canvas.coords("upper", ul_x, ul_y, lr_x, ul_y)
        self.canvas.coords("right", lr_x, ul_y, lr_x, lr_y)
        self.canvas.coords("lower", lr_x, lr_y, ul_x, lr_y)
        self.canvas.coords("left" , ul_x, lr_y, ul_x, ul_y)
        # 画像上の位置に変換し、トリミングの範囲を更新
        img_ul_x, img_ul_y = self.mymap.inv_transform(ul_x, ul_y)
        img_lr_x, img_lr_y = self.mymap.inv_transform(lr_x, lr_y)
        img_ul_x = max(img_ul_x, 0)
        img_ul_y = max(img_ul_y, 0)
        img_lr_x = min(img_lr_x, self.mymap.original_img_pil.width)
        img_lr_y = min(img_lr_y, self.mymap.original_img_pil.height)
        self.trim_range = [img_ul_x, img_ul_y, img_lr_x, img_lr_y]
        # 画像を更新して描画
        self.set_alpha(128)
        return
    

    def set_alpha(self, a: int):
        im_a = Image.new("L", self.mymap.original_img_pil.size, a)
        draw = ImageDraw.Draw(im_a)
        draw.rectangle(tuple(self.trim_range), fill=255)
        self.mymap.original_img_pil.putalpha(im_a)
        self.canvas.itemconfigure("map", image=self.mymap.get_draw_image((self.canv_w, self.canv_h)))
        return
    


    """
    ++++++++++ Plot and move point marker representing origin ++++++++++
    """
    def plot_origin(self):
        canv_origin = self.mymap.transform(self.origin_img[0], self.origin_img[1])
        r = 5
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
    


    """
    ++++++++++ Mouse event callback functions ++++++++++
    """
    def left_click_move(self, event):
        if (not self.mymap) or (self.trimming_mode) : return
        if self.old_click_point is None:
            self.old_click_point = [event.x, event.y]
            return
        delta_x = event.x - self.old_click_point[0]
        delta_y = event.y - self.old_click_point[1]
        self.old_click_point = [event.x, event.y]
        self.mymap.translate(delta_x, delta_y)
        self.canvas.itemconfigure("map", image=self.mymap.get_draw_image((self.canv_w, self.canv_h)))
        self.canvas.move("origin", delta_x, delta_y)
        return
    

    def ctrl_click(self, event, scale):
        if (not self.mymap) or (self.trimming_mode): return
        self.mymap.scale_at(event.x, event.y, scale)
        self.canvas.itemconfigure("map", image=self.mymap.get_draw_image((self.canv_w, self.canv_h)))
        self.plot_origin()
        return
    

    def left_click(self, event):
        self.old_click_point = [event.x, event.y]
        return
    

    def left_click_release(self, event):
        self.old_click_point = None
        return


    def resize_callback(self, event):
        self.canv_w = self.canvas.winfo_width()
        self.canv_h = self.canvas.winfo_height()
        return




if __name__ == "__main__":
    root = tk.Tk()
    w, h = root.winfo_screenwidth()-10, root.winfo_screenheight()-200
    root.geometry("%dx%d+0+0" % (w, h))
    app = Application(root)
    app.mainloop()
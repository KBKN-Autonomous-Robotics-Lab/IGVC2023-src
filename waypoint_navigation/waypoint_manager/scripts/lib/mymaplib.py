import numpy as np
from PIL import Image, ImageTk
from pathlib import Path


class MyMap():

    def __init__(self, path: Path, map_yaml):
        if map_yaml["image"][0] == "/":
            img_path = map_yaml["image"]
        else:
            img_path = path.with_name(map_yaml["image"]).resolve()
        self.pil_img = Image.open(img_path).convert("RGBA")
        self.tk_img = ImageTk.PhotoImage(self.pil_img)
        self.origin = map_yaml["origin"]
        self.resolution = map_yaml["resolution"]
        self.img_origin = [-self.origin[0]/self.resolution, self.origin[1]/self.resolution+self.height()]
        self.mat_affine = np.eye(3)
        return
    

    def translate(self, x, y):
        self.affine_tf(x, y, 1)
        return

    
    def scale_at(self, x, y, scale):
        self.affine_tf(-x, -y, 1)
        self.affine_tf(0, 0, scale)
        self.affine_tf(x, y, 1)
        return

    
    def affine_tf(self, x, y, scale):
        x = float(x)
        y = float(y)
        scale = float(scale)
        mat = np.array([[scale,0,x], [0,scale,y], [0,0,1]])
        self.mat_affine = np.dot(mat, self.mat_affine)
        return


    def get_draw_image(self, canvas_size):
        mat_inv = np.linalg.inv(self.mat_affine)
        img = self.pil_img.transform(canvas_size,
            Image.Transform.AFFINE, tuple(mat_inv.flatten()),
            Image.Resampling.NEAREST,
            fillcolor="#0000"
        )
        self.tk_img = ImageTk.PhotoImage(image=img)
        return self.tk_img
    

    def real2image(self, real_x, real_y):
        img_x = self.img_origin[0] + real_x / self.resolution
        img_y = self.img_origin[1] - real_y / self.resolution
        return img_x, img_y
    

    def image2real(self, img_x, img_y):
        real_x = (img_x - self.img_origin[0]) * self.resolution
        real_y = -(img_y - self.img_origin[1]) * self.resolution
        real_x = round(real_x, 6)
        real_y = round(real_y, 6)
        return real_x, real_y
    

    def transform(self, img_x, img_y):
        mat = [img_x, img_y, 1]
        tf_mat = np.dot(self.mat_affine, mat)
        return tf_mat[0], tf_mat[1]
    

    def inv_transform(self, x, y):
        mat = [x, y, 1]
        inv_affine = np.linalg.inv(self.mat_affine)
        inv = np.dot(inv_affine, mat)
        return inv[0], inv[1]

        
    def width(self):
        return self.pil_img.width
    
    def height(self):
        return self.pil_img.height
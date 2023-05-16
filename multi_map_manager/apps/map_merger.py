import tkinter as tk
from mylib.application import Application


if __name__ == "__main__":
    root = tk.Tk()
    w, h = root.winfo_screenwidth()-10, root.winfo_screenheight()-100
    root.geometry("%dx%d+0+0" % (w, h))
    root.title("Multi Map & Waypoints Merger")
    app = Application(root)
    app.mainloop()
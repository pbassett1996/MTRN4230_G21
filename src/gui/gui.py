import Tkinter
from Tkinter import *

def setVars():
    global colour
    global shape
    colour = col.get()
    shape = shp.get()

def start():
    print("start")

def stop():
    print("stop")

if __name__ == "__main__":
    root = Tk()
    root.title("MTRN4230 G21")
    root.geometry("500x300")
    root.configure(background='white smoke')
    
    col = StringVar()
    col.set("all")
    shp = StringVar()
    shp.set("all")
    
    title = Text(root)
    title.insert(INSERT,"MTRN4230 G21")
    
    titLable = Label(root, text="MTRN4230 G21",bg='white smoke')
    titLable.place(x=150, y=10)
    titLable.config(font=("Space", 20))
    
    #start button
    startButton = Button(
        root,
        text="START",
        command=start,
        width=10,
        height=2,
        fg="white",
        bg="green4",
        borderwidth=5
    )
    startButton.place(x=50,y=100)
    
    #stop button
    stopButton = Button(
        root,
        text="STOP",
        command=start,
        width=10,height=2,
        fg="white",
        bg="red3",
        borderwidth=5
    )
    stopButton.place(x=50,y=200)
    
    #colour drop down menu
    colorDrop = OptionMenu(root, col,'red', 'blue', 'green', 'yellow', 'all')
    colorDrop.config(fg='white', bg='slate gray', width=10, borderwidth=5)
    colorDrop["menu"].config(fg='white', bg='slate gray')
    colorDrop.place(x=200,y=100)
    
    colLable = Label(root, text="Colour")
    colLable.place(x=235, y=80)
    
    #shape drop down menu
    colorDrop = OptionMenu(root, shp,'Cub', 'Rect Box', 'Cylinder', 'all')
    colorDrop.config(fg='white', bg='slate gray', width=10, borderwidth=5)
    colorDrop["menu"].config(fg='white', bg='slate gray')
    colorDrop.place(x=350,y=100)
    
    shapeLable = Label(root, text="Shape")
    shapeLable.place(x=390, y=80)
    
    #slider
    quantity = Scale(
        root,
        from_=0,
        to=10,
        length=250,
        tickinterval=1,
        orient=HORIZONTAL,
        width=20
    )
    quantity.place(x=200,y=200)
    
    root.mainloop()


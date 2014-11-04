#!/usr/bin/python

from Tkinter import *

class App:

    def __init__(self, master):
        frame = Frame(master)
        frame.pack()

        self.width = 500;
        self.height = 500;

        # Create the window
        self.area = Canvas(frame, bg = "black", width = self.width, height = self.height)
        self.area.pack();
        
        self.timeButton = Button(frame, text = "Next Time Step", command = self.timeInc)
        self.timeButton.pack(side = LEFT)

        self.quitButton = Button(frame, text = "Quit", command = frame.quit)
        self.quitButton.pack(side = LEFT)

        # Create the locations    
        self.locationCount = 9
        self.width / self.locationCount    
        self.area.create_rectangle(100, 100, 300, 200, outline = "yellow")

        # Create the boids
        cx = 100
        cy = 100
        unit = 50;
        self.boid = self.area.create_polygon(cx - unit, cy - unit, cx, cy + unit, cx + unit, cy - unit, cx, cy - (unit/2), fill = "red", outline = "white");
        self.area.itemconfig(self.boid, tags = ("b1"))


    def timeInc(self):
        # Can use move method to move object
        self.area.move("b1", 100, 100)
        print "Time incremented"

root = Tk()

app = App(root)

root.wm_title("Boid Simulation")
root.mainloop()
root.destroy() # optional; see description below

## A root widget (window)
#root = tk.Tk()
#w = tk.Label(root, text="Hello, world!")

## pack() tells the label widget to fit the given text and be visible
#w.pack()

## Makes the window visible
#root.mainloop()

if __name__ == '__main__':
    print "yolo";

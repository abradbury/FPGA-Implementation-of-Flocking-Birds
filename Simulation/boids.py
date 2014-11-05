#!/usr/bin/python

from Tkinter import *
import math

class Boid:

    def __init__(self, canvas):
        # Create the boids

        self.bearing = math.pi
        self.centreX = 100
        self.centreY = 100
        self.chicken = 50;
        self.canvas = canvas

        self.x0 = self.centreX - self.chicken
        self.y0 = self.centreY - self.chicken
        self.x1 = self.centreX
        self.y1 = self.centreY + self.chicken
        self.x2 = self.centreX + self.chicken
        self.y2 = self.centreY - self.chicken
        self.x3 = self.centreX
        self.y3 = self.centreY - (self.chicken/2)

        self.boid = self.canvas.create_polygon(self.x0, self.y0, self.x1, self.y1, self.x2, 
            self.y2, self.x3, self.y3, fill = "red", outline = "white");
        self.canvas.itemconfig(self.boid, tags = ("b1"))

        self.boidCircle = self.canvas.create_oval(self.centreX - 5, self.centreY - 5, 
            self.centreX + 5, self.centreY + 5);

    # Rotation definition based on an answer from StackOverflow: http://stackoverflow.com/a/3409039
    def rotate(self, degrees):
        # Convert to radians for trig functions
        radians = degrees * math.pi / 180
        
        self.bearing -= radians

        # if self.bearing < 0:
        #     self.bearing += (math.pi)

        # if abs(self.bearing) > math.pi:
        #     self.bearing = abs(self.bearing) - math.pi
        # else:
        #     self.bearing = abs(radians)

        # if self.bearing < 0:
        #     self.bearing = -self.bearing

        #self.bearing = (self.bearing / math.pi)
        #self.bearing = 2 * math.pi * round(float(self.bearing)/(2 * math.pi))

        def _rot(x, y):
            # Note: the rotation is done in the opposite fashion from for a right-handed coordinate 
            #   system due to the left-handedness of computer coordinates
            x -= self.centreX
            y -= self.centreY
            _x = x * math.cos(radians) + y * math.sin(radians)
            _y = -x * math.sin(radians) + y * math.cos(radians)
            return _x + self.centreX, _y + self.centreY

        self.x0, self.y0 = _rot(self.x0, self.y0)
        self.x1, self.y1 = _rot(self.x1, self.y1)
        self.x2, self.y2 = _rot(self.x2, self.y2)
        self.x3, self.y3 = _rot(self.x3, self.y3)

        self.canvas.coords(self.boid, self.x0, self.y0, self.x1, self.y1, self.x2, self.y2, self.x3, self.y3)


class Location:

    def __init__(self):
        # Create the locations    
        self.locationCount = 9
        self.width / self.locationCount    
        self.canvas.create_rectangle(100, 100, 300, 200, outline = "yellow")


class Simulation:

    def __init__(self):
        self.root = Tk()
        self.root.wm_title("Boid Simulation")

        frame = Frame(self.root)
        frame.pack()

        self.width = 500;
        self.height = 500;

        # Create the window
        self.canvas = Canvas(frame, bg = "black", width = self.width, height = self.height)
        self.canvas.pack();
        
        self.timeButton = Button(frame, text = "Next Time Step", command = self.timeInc)
        self.timeButton.pack(side = LEFT)

        self.quitButton = Button(frame, text = "Quit", command = frame.quit)
        self.quitButton.pack(side = LEFT)



        self.boid = Boid(self.canvas)
        print self.boid.bearing

        self.root.mainloop()


    def timeInc(self):
        # Can use move method to move object
        #self.canvas.move("b1", 100, 100)
        self.boid.rotate(90)
        print self.boid.bearing



boidSimulation = Simulation()

if __name__ == '__main__':
    print "yolo";

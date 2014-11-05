#!/usr/bin/python

from Tkinter import *       # Used to draw shapes for the simulation
import numpy as np          # Used in various mathematical operations
import random               # Used to randomly position the boids on initialisation

class Boid:

    def __init__(self, canvas, initPosition):
        # Create the boids
        self.bearing = np.pi
        self.step = 10
        self.canvas = canvas
        self.initPosition = initPosition

        # Define the boid polygon points
        self.x0 = self.initPosition[0] - self.step
        self.y0 = self.initPosition[1] - self.step
        self.x1 = self.initPosition[0]
        self.y1 = self.initPosition[1] + self.step
        self.x2 = self.initPosition[0] + self.step
        self.y2 = self.initPosition[1] - self.step
        self.x3 = self.initPosition[0]
        self.y3 = self.initPosition[1] - (self.step/2)

        # Draw the boid
        self.boid = self.canvas.create_polygon(self.x0, self.y0, self.x1, self.y1, self.x2, 
            self.y2, self.x3, self.y3, fill = "red", outline = "white")
        self.canvas.itemconfig(self.boid, tags = ("b1"))

        # Draw a circle at the centre of the boid
        # self.boidCircle = self.canvas.create_oval(self.initPosition[0] - 5, self.initPosition[1] - 5, 
        #     self.initPosition[0] + 5, self.initPosition[1] + 5);


    # Rotation definition based on an answer from StackOverflow: http://stackoverflow.com/a/3409039
    def rotate(self, degrees):
        # Convert to radians for trig functions
        radians = degrees * np.pi / 180
        self.bearing -= radians

        def _rot(x, y):
            # Note: the rotation is done in the opposite fashion from for a right-handed coordinate 
            #   system due to the left-handedness of computer coordinates
            x -= self.centreX
            y -= self.centreY
            _x = x * np.cos(radians) + y * np.sin(radians)
            _y = -x * np.sin(radians) + y * np.cos(radians)
            return _x + self.centreX, _y + self.centreY

        self.x0, self.y0 = _rot(self.x0, self.y0)
        self.x1, self.y1 = _rot(self.x1, self.y1)
        self.x2, self.y2 = _rot(self.x2, self.y2)
        self.x3, self.y3 = _rot(self.x3, self.y3)

        self.canvas.coords(self.boid, self.x0, self.y0, self.x1, self.y1, self.x2, self.y2, self.x3, self.y3)


class Location:

    def __init__(self, canvas, locationID, locationCoords, initialBoidCount):
        # Draw the location bounds
        canvas.create_rectangle(locationCoords[0], locationCoords[1], locationCoords[2], locationCoords[3], outline = "yellow")

        # Draw the location's boids
        for i in range (0, initialBoidCount):
            # Randomly position the boid on initialisation
            self.randomX = random.randint(locationCoords[0], locationCoords[2]);
            self.randomY = random.randint(locationCoords[1], locationCoords[3]);
            self.initialPosition = np.array([self.randomX, self.randomY])
            Boid(canvas, self.initialPosition)

        print "Created location " + str(locationID) + " with " + str(initialBoidCount) + " boids"





class Simulation:

    def __init__(self):
        self.root = Tk()
        self.root.wm_title("Boid Simulation")

        frame = Frame(self.root)
        frame.pack()

        self.width = 500
        self.height = 500
        self.boidCount = 90

        # Create the window
        self.canvas = Canvas(frame, bg = "black", width = 500, height = self.height)
        self.canvas.pack();
        
        # Create the buttons
        self.timeButton = Button(frame, text = "Next Time Step", command = self.timeInc)
        self.timeButton.pack(side = LEFT)
        self.quitButton = Button(frame, text = "Quit", command = frame.quit)
        self.quitButton.pack(side = LEFT)

        # Needed so that the canvas sizes can be used later
        self.root.update()

        # Create the locations
        self.allowedLocationCounts = np.array([1, 2, 4, 9, 16, 25, 36])
        self.locationCount = self.allowedLocationCounts[3]
        self.initialBoidCount = self.boidCount / self.locationCount
        
        self.locationSize = round(self.canvas.winfo_width() / np.sqrt(self.locationCount))

        self.locationCoords = np.array([0, 0, 0, 0])
        for i in range(0, self.locationCount):
            self.locationCoords[0] = (i % 3) * self.locationSize
            self.locationCoords[1] = int(np.floor(i / 3)) * self.locationSize
            self.locationCoords[2] = self.locationCoords[0] + self.locationSize
            self.locationCoords[3] = self.locationCoords[1] + self.locationSize

            print "[" + str(self.locationCoords[0]) + "," + str(self.locationCoords[1]) + "]"

            Location(self.canvas, i + 1, self.locationCoords, self.initialBoidCount)


        # Start everything going
        self.root.mainloop()


    def timeInc(self):
        Boid(self.canvas)



boidSimulation = Simulation()

if __name__ == '__main__':
    print "yolo";

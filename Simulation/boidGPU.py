#!/usr/bin/python
# -*- coding: utf-8 -*-


from Tkinter import *               # Used to draw shapes for the simulation
import numpy as np                  # Used in various mathematical operations


# 
class BoidGPU:

    def __init__(self, _simulation):
        # Link back to simulation
        self.simulation = _simulation

        # Make the system configuration list available
        self.config = self.simulation.config

        # Create the window
        self.root = Tk()
        self.root.wm_title("Boid Simulation")

        frame = Frame(self.root)
        frame.pack()
        
        self.canvas = Canvas(frame, bg = "black", width = self.config['width'], height = self.config['height'])
        self.canvas.pack();
        
        # Create the buttons
        self.timeButton = Button(frame, text = "Next Time Step", command = self.simulation.nextStepButton)
        self.timeButton.pack(side = LEFT)
        self.pauseButton = Button(frame, text = "Begin", command = self.simulation.pause)
        self.pauseButton.pack(side = LEFT)
        self.graphButton = Button(frame, text = "Update Graphs", command = self.simulation.updateGraphs)
        self.graphButton.pack(side = LEFT)
        self.quitButton = Button(frame, text = "Quit", command = frame.quit)
        self.quitButton.pack(side = LEFT)

        self.counterLabel = Label(frame, text = 0, width = 6)
        self.counterLabel.pack(side = RIGHT)

        # Needed so that the canvas sizes can be used later
        self.root.update()


    ################################################################################################
    ## User Input Functions ----------------------------------------------------------------------##
    ################################################################################################

    def updateTimeStepLabel(self, newTimeStep):
        self.counterLabel.config(text = newTimeStep)


    def togglePauseButton(self, resume):
        if resume:
            self.pauseButton.config(text = "Pause")
        else:
            self.pauseButton.config(text = "Resume")


    ################################################################################################
    ## Setup Functions ---------------------------------------------------------------------------##
    ################################################################################################

    def initialiseSimulation(self):
        self.bearing = np.pi
        self.step = 10


    # Creates a boidCPU object and draws on the screen
    def drawBoidCPU(self, coords, locID, colour):
        self.canvas.create_rectangle(coords[0], coords[1], coords[2], coords[3], outline = colour, 
            tags = "L" + str(locID))

    # Redraws the bounds of the boidCPU
    def updateBoidCPU(self, locID, coords):
        self.canvas.coords("L" + str(locID), coords[0], coords[1], coords[2], coords[3])




    def getWindowSize(self):
        return [self.canvas.winfo_width(), self.canvas.winfo_height()]

    # Temporary method
    def getCanvas(self):
        return self.canvas

    def beginMainLoop(self):
        # Start everything going
        self.root.mainloop()


    ################################################################################################
    ## Boid Update Functions ---------------------------------------------------------------------##
    ################################################################################################

    def createBoid(self, position, velocity, colour, outlineColour, boidWidth, boidID):
        points = self.calcBoidPoints(position)

        # Draw the boid
        self.canvas.create_polygon(points[0], points[1], points[2], points[3], points[4], points[5], points[6], points[7], fill = colour, outline = 
            outlineColour, width = boidWidth, tags = ("B" + str(boidID)))

        self.rotateBoid(boidID, velocity, position, points)


    def nextSimulationStep(self, milliseconds):
        self.canvas.after(milliseconds, self.simulation.simulationStep)


    # Calculate the posisitions of the boid polygon points
    def calcBoidPoints(self, position):
        stepSize = 10
        points = []

        points.append(position[0] - stepSize)
        points.append(position[1] - stepSize)
        points.append(position[0])
        points.append(position[1] + stepSize)
        points.append(position[0] + stepSize)
        points.append(position[1] - stepSize)
        points.append(position[0])
        points.append(position[1] - (stepSize/2))

        return points


    # Move the boid on the screen based on the new velocity and position
    def updateBoid(self, position, velocity, colour, boidID):
        points = self.calcBoidPoints(position)

        self.canvas.coords("B" + str(boidID), points[0], points[1], points[2], points[3], points[4], points[5], points[6], points[7])
        self.canvas.itemconfig("B" + str(boidID), fill = colour) 

        self.rotateBoid(boidID, velocity, position, points)

        # Debugging method - follow a specific boid
        if self.config['trackBoid'] and (boidID == self.config['boidToTrack']):  
            self.followBoid(position, boidID)


    # Rotate the specified boid based on its velocity / orientation
    # Rotation definition based on an answer from StackOverflow: http://stackoverflow.com/a/3409039
    def rotateBoid(self, boidID, velocity, position, points):
        radians = np.arctan2(velocity[0], velocity[1])

        # Convert to radians for trig functions
        # radians = degrees * np.pi / 180
        self.bearing -= radians
        self.bearing = self.bearing % (2 * np.pi)

        def _rot(x, y):
            # Note: the rotation is done in the opposite fashion from for a right-handed coordinate 
            #   system due to the left-handedness of computer coordinates
            x -= position[0]
            y -= position[1]
            _x = x * np.cos(radians) + y * np.sin(radians)
            _y = -x * np.sin(radians) + y * np.cos(radians)
            return _x + position[0], _y + position[1]

        points[0], points[1] = _rot(points[0], points[1])
        points[2], points[3] = _rot(points[2], points[3])
        points[4], points[5] = _rot(points[4], points[5])
        points[6], points[7] = _rot(points[6], points[7])

        self.canvas.coords("B" + str(boidID), points[0], points[1], points[2], points[3], points[4], points[5], points[6], points[7])


    # Follows a boid as it moves around the area. The boid has its vision circle shown and is 
    # coloured blue. Any neighbouring boid is coloured green. 
    def followBoid(self, position, boidID):
        self.canvas.delete("boidCircle")
        self.canvas.itemconfig("B" + str(boidID), fill = "blue")

        self.canvas.create_oval(position[0] - self.config['VISION_RADIUS'], 
            position[1] - self.config['VISION_RADIUS'], position[0] + self.config['VISION_RADIUS'], 
            position[1] + self.config['VISION_RADIUS'], outline = "yellow", tags = "boidCircle")


    # Highlights or de-highlights a specific boid based on the given boidID
    def highlightBoid(self, on, boidID):
        if on:
            self.canvas.itemconfig("B" + str(boidID), fill = "green")
        else:
            self.canvas.itemconfig("B" + str(boidID), fill = "red")


    # Draws or updates a line on the screen, depending on if the tag can be found
    def drawLine(self, startPoints, endPoints, tag):
        handle = self.canvas.find_withtag(tag)

        if handle:
            self.canvas.coords(tag, startPoints[0], startPoints[1], endPoints[0], endPoints[1])
        else:
            self.canvas.create_line([startPoints[0], startPoints[1], endPoints[0], endPoints[1]], fill = "red", tags = tag)


    def drawBoidCPUGrid(self, boidCPUCoords, segmentWidth, segmentHieght):
        for i in range(segmentWidth):
            x = boidCPUCoords[0] + (i * self.config['stepSize'])
            self.canvas.create_line([x, boidCPUCoords[1], x, boidCPUCoords[3]], fill = "green", tags = "gridLines")

        for i in range(segmentHieght):
            y = boidCPUCoords[1] + (i * self.config['stepSize'])
            self.canvas.create_line([boidCPUCoords[0], y, boidCPUCoords[2], y], fill = "green", tags = "gridLines")

    def removeBoidCPUGrid(self):
        self.canvas.delete("gridLines")



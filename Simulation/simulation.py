#!/usr/bin/python


from location import Location       # Import the Location class
from boid import Boid               # Import the Boid class

import logging                      # Used to handle the textual output
from Tkinter import *               # Used to draw shapes for the simulation
import numpy as np                  # Used in various mathematical operations
import matplotlib.pyplot as plt     # Used to plot the graphs
import time                         # Used to time stuff


# The main application class. Sets up the simulation area and the number of locations that it is to 
# be divided into. 
#
# Note that the simulation begins with a user pressing the pauseButton (initially labelled 'Begin'). 
# Because the pause flag is initally set to true, in the action function the simulation is 'resumed'.
class Simulation:

    def __init__(self):
        # Setup the simulation parameters
        self.width = 700
        self.height = 700
        self.boidCount = 90
        self.pauseSimulation = True
        self.timeStepCounter = 0

        # Setup logging
        self.setupLogging()

        # Create the window
        self.root = Tk()
        self.root.wm_title("Boid Simulation")

        frame = Frame(self.root)
        frame.pack()
        
        self.canvas = Canvas(frame, bg = "black", width = self.width, height = self.height)
        self.canvas.pack();
        
        # Create the buttons
        self.timeButton = Button(frame, text = "Next Time Step", command = self.nextStepButton)
        self.timeButton.pack(side = LEFT)
        self.pauseButton = Button(frame, text = "Begin", command = self.pause)
        self.pauseButton.pack(side = LEFT)
        self.graphButton = Button(frame, text = "Update Graphs", command = self.updateGraphs)
        self.graphButton.pack(side = LEFT)
        self.quitButton = Button(frame, text = "Quit", command = frame.quit)
        self.quitButton.pack(side = LEFT)

        self.counterLabel = Label(frame, text = self.timeStepCounter, width = 6)
        self.counterLabel.pack(side = RIGHT)

        # Needed so that the canvas sizes can be used later
        self.root.update()

        # Create the locations
        self.allowedLocationCounts = np.array([1, 2, 4, 9, 16, 25, 36])
        self.locationCount = self.allowedLocationCounts[3]
        self.initialBoidCount = self.boidCount / self.locationCount
        self.locationSize = round(self.canvas.winfo_width() / np.sqrt(self.locationCount))

        self.locations = []
        self.calcTimings = []
        self.drawTimings = []

        self.locationColours = ["red", "blue", "green", "yellow", "white", "magenta", "dark grey", "cyan", "dark green"]

        self.locationCoords = np.array([0, 0, 0, 0])
        for i in range(0, self.locationCount):
            self.locationCoords[0] = (i % 3) * self.locationSize
            self.locationCoords[1] = int(np.floor(i / 3)) * self.locationSize
            self.locationCoords[2] = self.locationCoords[0] + self.locationSize
            self.locationCoords[3] = self.locationCoords[1] + self.locationSize

            loc = Location(self.canvas, self, i + 1, self.locationCoords, self.initialBoidCount, self.locationColours[i])
            self.locations.append(loc)

        self.logger.info("- Press the 'Begin' button to start the simulation")

        # Setup the graphs
        self.setupGraphs()

        # Start everything going
        self.root.mainloop()


    # Setup logging
    def setupLogging(self):   
        self.logger = logging.getLogger('boidSimulation')
        self.logger.setLevel(logging.WARNING)

        self.ch = logging.StreamHandler()
        self.ch.setLevel(logging.WARNING)

        self.formatter = logging.Formatter("[%(levelname)8s] --- %(message)s")
        self.ch.setFormatter(self.formatter)
        self.logger.addHandler(self.ch)


    # Create the graphs
    def setupGraphs(self):
        self.yData = []

        self.lines = []
        self.lines2 = []

        self.axes = []
        self.axes2 = []

        for i in range(0, self.locationCount):
            # Create the subplots and sync the axes
            if i != 0:
                axis = plt.subplot(3, 3, i+1, sharex = self.axes[0], sharey = self.axes[0])
            else:
                axis = plt.subplot(3, 3, i+1)

            axis.set_title("Location %d" % (i + 1))
            axis.grid(True)
            
            # Setup the first y axis
            for tl in axis.get_yticklabels():
                tl.set_color('b')
    
            # Setup the second y axis
            axis2 = axis.twinx()
            for tl in axis2.get_yticklabels():
                tl.set_color('r')

            # Plot each line, returns a list of lines - the comma is needed
            # If the x data set is not specified, it is assumed to be the number 
            # of element contained in the y data set - which is fine here.
            line1, = axis.plot(self.yData, color = 'b')
            line2, = axis2.plot(self.yData, color = 'r')
            
            # Customise the suplot grid so that axes don't overlap
            if (i % 3) != 0:
                plt.setp(axis.get_yticklabels(), visible = False)
            else:
                axis.set_ylabel("Computation time, seconds", color = 'b')

            if (i % 3) != (3 - 1):
                plt.setp(axis2.get_yticklabels(), visible = False)
            else:
                axis2.set_ylabel("Number of boids", color = 'r')

            if int(np.floor(i / 3)) != 2:
                plt.setp(axis.get_xticklabels(), visible = False)
            else:
                axis.set_xlabel("Number of timesteps")

            # Add the lines to the line lists
            self.lines.append(line1)
            self.lines2.append(line2)

            # Add the axes to the axes lists
            self.axes.append(axis)
            self.axes2.append(axis2)

        # Show the subplots in interactive mode (doesn't block)
        plt.ion()
        plt.show()


    # For each location, get the graph lines and set the data to the current data  
    # plus the new data. Then reformat both axes to accommodate the new data.
    def updateGraphs(self):

        for i in range(0, self.locationCount):
            self.lines[i].set_xdata(self.locations[i].xData)
            self.lines[i].set_ydata(self.locations[i].yData)

            self.lines2[i].set_xdata(self.locations[i].xData)
            self.lines2[i].set_ydata(self.locations[i].y2Data)

            self.axes[i].relim()
            self.axes[i].autoscale_view()

            self.axes2[i].relim()
            self.axes2[i].autoscale_view()

            plt.draw()


    def simulationStep(self):
        if self.pauseSimulation == False:
            for i in range(0, self.locationCount):
                self.logger.debug("Calculating next boid positions for location " + 
                    str(self.locations[i].locationID) + "...")

                # Calculate the next boid positions and time this                
                self.startTime = time.clock()
                self.locations[i].update(False)
                self.endTime = time.clock()

                # Store the timing information for later plotting
                self.locations[i].xData.append(self.timeStepCounter)
                self.locations[i].yData.append(self.endTime - self.startTime)

            for i in range(0, self.locationCount):
                self.logger.debug("Moving boids to calculated positions for location " + 
                    str(self.locations[i].locationID) + "...")
    
                # Update the canvas with the new boid positions
                self.locations[i].update(True)

                # Store the number of boids for later plotting
                self.locations[i].y2Data.append(self.locations[i].boidCount)

            self.logger.info("Location boid counts: " + " ".join(str(loc.boidCount) 
                for loc in self.locations))

            # Update the counter label
            self.timeStepCounter += 1
            self.counterLabel.config(text = self.timeStepCounter)

            # Call self after 10ms
            self.canvas.after(10, self.simulationStep)

    
    # Manual timestep increment
    def nextStepButton(self):
        self.pauseSimulation = False
        self.simulationStep()
        self.pauseSimulation = True


    # Sets a flag that is used to pause and resume the simulation 
    def pause(self):
        if self.pauseSimulation == False:
            self.pauseSimulation = True
            self.pauseButton.config(text = "Resume")
        else:
            self.pauseSimulation = False
            self.pauseButton.config(text = "Pause")
            self.simulationStep()


    # Get the neighbouring locations of the specified location. Currently, this simply returns a 
    # hard-coded list of neighbours tailored to the asking location. Ideally, the neighbours would 
    # be calculated in a programmatic way.
    def getNeighbouringLocations(self, locationID):
        if locationID == 1:
            self.neighbouringLocations = [0, 0, 0, 2, 5, 4, 0, 0]
            # self.neighbouringLocations = [2, 3, 4, 5, 6, 7, 8, 9]
        elif locationID == 2:
            self.neighbouringLocations = [0, 0, 0, 3, 6, 5, 4, 1]
            # self.neighbouringLocations = [1, 3, 4, 5, 6, 7, 8, 9]
        elif locationID == 3:
            self.neighbouringLocations = [0, 0, 0, 0, 0, 6, 5, 2]
            # self.neighbouringLocations = [2, 1, 4, 5, 6, 7, 8, 9]
        elif locationID == 4:
            self.neighbouringLocations = [0, 1, 2, 5, 8, 7, 0, 0]
            # self.neighbouringLocations = [2, 3, 1, 5, 6, 7, 8, 9]
        elif locationID == 5:
            self.neighbouringLocations = [1, 2, 3, 6, 9, 8, 7, 4]
            # self.neighbouringLocations = [2, 3, 4, 1, 6, 7, 8, 9]
        elif locationID == 6:
            self.neighbouringLocations = [2, 3, 0, 0, 0, 9, 8, 5]
            # self.neighbouringLocations = [2, 3, 4, 5, 1, 7, 8, 9]
        elif locationID == 7:
            self.neighbouringLocations = [0, 4, 5, 8, 0, 0, 0, 0]
            # self.neighbouringLocations = [2, 3, 4, 5, 6, 1, 8, 9]
        elif locationID == 8:
            self.neighbouringLocations = [4, 5, 6, 9, 0, 0, 0, 7]
            # self.neighbouringLocations = [2, 3, 4, 5, 6, 7, 1, 9]
        elif locationID == 9:
            self.neighbouringLocations = [5, 6, 0, 0, 0, 0, 0, 8]
            # self.neighbouringLocations = [2, 3, 4, 5, 6, 7, 8, 1]

        return self.neighbouringLocations


    # Return a list of the boids for a specified location
    def getLocationBoids(self, locationID):
        return self.locations[locationID - 1].getBoids()


    # Transfer a boid from one location to another
    def transferBoid(self, boid, toID, fromID):
        self.locations[toID - 1].acceptBoid(boid, fromID)


if __name__ == '__main__':
    # Start everything off
    boidSimulation = Simulation()

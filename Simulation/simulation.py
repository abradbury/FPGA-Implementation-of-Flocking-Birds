#!/usr/bin/python
# -*- coding: utf-8 -*-


from location import Location       # Import the Location class
from boid import Boid               # Import the Boid class

import logging                      # Used to handle the textual output
from Tkinter import *               # Used to draw shapes for the simulation
import numpy as np                  # Used in various mathematical operations
import matplotlib.pyplot as plt     # Used to plot the graphs
import time                         # Used to time stuff


## MUST DOs ========================================================================================
# FIXME: Investigate arithmetic warning on rule calculation (causes boid to disappear from GUI)
# FIXME: Boids don't seem to be repelling each other that much, they are on top of one another
# FIXME: Obstacle avoidance does not seem to work well

# TODO: Modify code to handle different load balancing protocols
# TODO: Implement load balancing algorithm 1
#       - Locations should know if they are overloaded
#       - They should signal the controller with a requested boundary step change
# TODO: Implement load balancing algorithm 2

## MAY DOs =========================================================================================
# TODO: Add keybinding to capture return key and simulate button press
# TODO: Add acceleration to smooth movement (especially around borders)
# TODO: Calculate a locations neighbours programmatically - rather than hardcoding


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

        # The maximum amount of boids a location should have at any one time
        self.BOID_THRESHOLD = 30

        self.loadBalance = False

        # Define debugging flags
        self.colourCode = False
        self.trackBoid = True

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

        self.locationColours = ["red", "blue", "green", "yellow", "white", "magenta", "dark grey", 
            "cyan", "dark green"]

        self.locationCoords = np.array([0, 0, 0, 0])
        for i in range(0, self.locationCount):
            self.locationCoords[0] = (i % 3) * self.locationSize
            self.locationCoords[1] = int(np.floor(i / 3)) * self.locationSize
            self.locationCoords[2] = self.locationCoords[0] + self.locationSize
            self.locationCoords[3] = self.locationCoords[1] + self.locationSize

            # Define the location's position in the grid of locations [row, col]
            self.locationGridPos = [int(np.floor(i / 3)), (i % 3)]

            loc = Location(self.canvas, self, i + 1, self.locationCoords, self.initialBoidCount, 
                self.locationColours[i], self.locationGridPos)
            self.locations.append(loc)

        # Draw the dynamic lines to test the load balancing
        if self.loadBalance: 
            self.drawDynamicLines()

        # Setup variables to keep track of how many locations exceed the boid threshold
        self.violationCount = 0
        self.violationList = []

        self.logger.info("- Press the 'Begin' button to start the simulation")

        # Setup the graphs
        self.setupGraphs()
        self.setupSummaryGraph()

        # Start everything going
        self.root.mainloop()


    # Setup logging
    def setupLogging(self):   
        self.logger = logging.getLogger('boidSimulation')
        self.logger.setLevel(logging.DEBUG)

        self.ch = logging.StreamHandler()
        self.ch.setLevel(logging.WARNING)

        self.formatter = logging.Formatter("[%(levelname)8s] --- %(message)s")
        self.ch.setFormatter(self.formatter)
        self.logger.addHandler(self.ch)


    # Create the graphs
    def setupGraphs(self):
        self.yData = []

        # Need to hold a list of lines for each graph, surely there must be a better way to do this?
        self.lines = []
        self.lines2 = []
        self.thresholdLines = []
        self.andallLines = []

        self.axes = []
        self.axes2 = []

        self.graphFigure = plt.figure()

        for i in range(0, self.locationCount):
            # Create the subplots and sync the axes
            if i != 0:
                axis = self.graphFigure.add_subplot(3, 3, i+1, sharex = self.axes[0], sharey = self.axes[0])
            else:
                axis = self.graphFigure.add_subplot(3, 3, i+1)

            axis.set_title("Location %d" % (i + 1))
            axis.grid(True)
            
            # Setup the first y axis
            for tl in axis.get_yticklabels():
                tl.set_color('b')
    
            # Setup the second y axis
            axis2 = axis.twinx()
            for tl in axis2.get_yticklabels():
                tl.set_color('r')
            axis2.set_ylim([0, self.boidCount])

            # Plot each line, returns a list of lines - the comma is needed
            # If the x data set is not specified, it is assumed to be the number 
            # of element contained in the y data set - which is fine here.
            line1, = axis.plot(self.yData, color = 'b')
            line2, = axis2.plot(self.yData, color = 'r')

            # Add the Andall(sp?) and max boid threshold lines
            thresholdLine, = axis2.plot([], [], color = 'm')
            andallLine, = axis2.plot([], [], color = 'g')
            
            # Customise the suplot grid so that axes don't overlap
            if (i % 3) != 0:
                plt.setp(axis.get_yticklabels(), visible = False)
            else:
                axis.set_ylabel("Computation time, ms", color = 'b')

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
            self.andallLines.append(andallLine)
            self.thresholdLines.append(thresholdLine)

            # Add the axes to the axes lists
            self.axes.append(axis)
            self.axes2.append(axis2)

        # Show the subplots in interactive mode (doesn't block)
        plt.ion()
        plt.show()


    # Create a sumary graph showing the number of locations exceeding the boid threshold over time
    def setupSummaryGraph(self):
        plt.ion()

        self.summaryFigure, self.summaryAxis = plt.subplots(1,1)
        self.summaryAxis.plot([], [])
        self.summaryAxis.fill_between([], 0, [])

        self.summaryAxis.set_ylim([0, self.locationCount])
        
        self.summaryAxis.set_title("Graph showing number of locations exceeding the boid threshold")
        self.summaryAxis.grid(True)

        self.summaryAxis.set_xlabel("Number of time steps")
        self.summaryAxis.set_ylabel("Locations over threshold")

        plt.show()



    # For each location, get the graph lines and set the data to the current data  
    # plus the new data. Then reformat both axes to accommodate the new data.
    def updateGraphs(self):

        for i in range(0, self.locationCount):
            # Update the location boid calculation time lines
            self.lines[i].set_xdata(self.locations[i].xData)
            self.lines[i].set_ydata(self.locations[i].yData)

            # Update the location boid count lines. Because the draw routine is called first, but 
            # not on time step 1, it has one less data element in than the values for the update 
            # stage. Therefore, [0:-1] is used to ignore the last x value.
            self.lines2[i].set_xdata(self.locations[i].xData[0:-1])
            self.lines2[i].set_ydata(self.locations[i].y2Data)

            # Update the Andall (sp?) lines
            self.andallLines[i].set_xdata([0, len(self.locations[i].xData)])
            self.andallLines[i].set_ydata([self.initialBoidCount, self.initialBoidCount])

            # Update the max boid threshold lines
            self.thresholdLines[i].set_xdata([0, len(self.locations[i].xData)])
            self.thresholdLines[i].set_ydata([self.BOID_THRESHOLD, self.BOID_THRESHOLD])

            # Adjust the axes accordingly
            self.axes[i].relim()
            self.axes[i].autoscale_view()

            self.axes2[i].relim()
            self.axes2[i].autoscale_view()

            # Re-draw the graphs
            self.graphFigure.canvas.draw()

        # Update summary graph
        self.summaryAxis.lines[0].set_xdata(range(0, self.timeStepCounter))
        self.summaryAxis.lines[0].set_ydata(self.violationList)
        self.summaryAxis.fill_between(range(0, self.timeStepCounter), 0, self.violationList)

        self.summaryAxis.relim()
        self.summaryAxis.autoscale_view()

        self.summaryFigure.canvas.draw()


    def simulationStep(self):
        if self.pauseSimulation == False:
            self.violationCount = 0

            # The draw method is called first to enable a the neighbours of a boid to be highlighted 
            # when a boid is being followed (during debugging). As no new boid positions have been 
            # calculated, the draw function isn't called on the first time step.
            if self.timeStepCounter != 0:
                for i in range(0, self.locationCount):
                    self.logger.debug("Moving boids to calculated positions for location " + 
                        str(self.locations[i].locationID) + "...")
        
                    # Update the canvas with the new boid positions
                    self.locations[i].update(True)

                    # Store the number of boids for later plotting
                    self.locations[i].y2Data.append(self.locations[i].boidCount)

            # Update the boid
            for i in range(0, self.locationCount):
                self.logger.debug("Calculating next boid positions for location " + 
                    str(self.locations[i].locationID) + "...")

                # Calculate the next boid positions and time this                
                self.startTime = time.clock()
                self.locations[i].update(False)
                self.endTime = time.clock()

                # Store the timing information for later plotting
                self.locations[i].xData.append(self.timeStepCounter)
                self.locations[i].yData.append((self.endTime - self.startTime) * 1000)

                # If the location is exceeding the threshold, increment counter
                if self.locations[i].boidCount > self.BOID_THRESHOLD:
                    self.violationCount += 1

            self.logger.debug("Location boid counts: " + " ".join(str(loc.boidCount) 
                for loc in self.locations))

            # Update the counter label
            self.timeStepCounter += 1
            self.counterLabel.config(text = self.timeStepCounter)

            # Update the violation list
            self.violationList.append(self.violationCount)

            if self.loadBalance:
                self.getLineStatus()

            # Call self after 20ms (50 Hz)
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


    # Currently, this method changes the bounds of all the applicable edges of an overloaded 
    # location and the other bounds of the other locations that would be affected by this change.
    #
    # FIXME: Cannot currently handle when multiple locations are overloaded
    # FIXME: Assumes that an overloaded location wishes to shrink all its boundaries
    def locationOverloaded(self, locationID):
        stepSize = 20

        self.logger.debug("Location " + str(locationID) + " overloaded")

        # 1) Query the other locations to determine how the requested change would affect them
        # 2) Determine what action to take
        # 3) Implement change

        # Determine the row and column of the location that is requesting load balancing
        [row, col] = self.locations[locationID - 1].gridPosition

        # # Determine which locations would be affected by this change
        locationsToChange = self.identifyAffectedLocations(row, col)

        # # Update the edges on the locations
        for edge in locationsToChange.keys():
            for locID in locationsToChange.get(edge):
                self.locations[locID - 1].changeBounds(edge, stepSize, [row, col])


    # Based on the position of the location in the simulation grid, determine which of the sides of 
    # the location would need changing (sides on simulation edge cannot be changed)
    def identifyAffectedLocations(self, row, col):
        top = False
        right = False
        bottom = False
        left = False

        # Used as a temporary value for the location width of the simulation (i.e. 3 by 3)
        locationThing = 3 

        # Corners
        if col == 0 and row == 0:
            right = True
            bottom = True
        elif (col == locationThing - 1) and (row == locationThing - 1):
            top = True
            left = True
        elif col == 0 and (row == locationThing - 1):
            top = True
            right = True
        elif (col == locationThing - 1) and row == 0:
            bottom = True
            left = True

        # Edges
        elif col == 0:
            top = True 
            right = True 
            bottom = True
        elif row == 0:
            right = True
            bottom = True
            left = True
        elif (col == locationThing - 1):
            top = True
            bottom = True
            left = True
        elif (row == locationThing - 1):
            top = True
            right = True
            left = True
        
        # Middle
        else:
            top = True
            right = True
            bottom = True
            left = True

        # Iterate over the locations to determine the IDs of those locations that would be affected 
        # by the requested change and which edges of the locations would need changing
        locationsToChange = {'top': [], 'right': [], 'bottom': [], 'left': []}
        for l in self.locations:
            if top and (l.gridPosition[0] == row - 1):
                locationsToChange['bottom'].append(l.locationID)

            if right and (l.gridPosition[1] == col + 1):
                locationsToChange['left'].append(l.locationID)

            if bottom and (l.gridPosition[0] == row + 1):
                locationsToChange['top'].append(l.locationID)

            if left and (l.gridPosition[1] == col - 1):
                locationsToChange['right'].append(l.locationID)


            if top and (l.gridPosition[0] == row):
                locationsToChange['top'].append(l.locationID)

            if right and (l.gridPosition[1] == col):
                locationsToChange['right'].append(l.locationID)

            if bottom and (l.gridPosition[0] == row):
                locationsToChange['bottom'].append(l.locationID)

            if left and (l.gridPosition[1] == col):
                locationsToChange['left'].append(l.locationID)

        return locationsToChange


    # Create dynamic lines that are initially over the location boundaries
    # FIXME: Avoid hardcoding to 9 locations
    def drawDynamicLines(self):
        line1 = self.canvas.create_line(self.locationSize, 0, self.locationSize, 
            self.canvas.winfo_width(), fill = "green", tags = "D1")
        line2 = self.canvas.create_line(self.locationSize * 2, 0, self.locationSize * 2, 
            self.canvas.winfo_width(), fill = "green", tags = "D2")

        line3 = self.canvas.create_line(0, self.locationSize, self.canvas.winfo_width(), 
            self.locationSize, fill = "green", tags = "D3")
        line4 = self.canvas.create_line(0, self.locationSize * 2, self.canvas.winfo_width(), 
            self.locationSize * 2, fill = "green", tags = "D4")

        self.dynamicLines = [line1, line2, line3, line4]


    # Move the lines depending on the number of boids to either side of the line. Designed to show 
    # how the location boundaries could change in order to balance the load.
    #
    # FIXME: If the lines move, the no. of boids either side is calculated from the initial position
    # FIXME: Avoid hardcoding to 9 locations
    def getLineStatus(self):
        self.lineStep = 10

        row1 = self.locations[0].boidCount + self.locations[1].boidCount + self.locations[2].boidCount
        row2 = self.locations[3].boidCount + self.locations[4].boidCount + self.locations[5].boidCount
        row3 = self.locations[6].boidCount + self.locations[7].boidCount + self.locations[8].boidCount

        col1 = self.locations[0].boidCount + self.locations[3].boidCount + self.locations[6].boidCount
        col2 = self.locations[1].boidCount + self.locations[4].boidCount + self.locations[7].boidCount
        col3 = self.locations[2].boidCount + self.locations[5].boidCount + self.locations[8].boidCount

        if row1 > row2: 
            move = "up"
            self.canvas.move("D3", 0, -self.lineStep)
        elif row2 > row1:
            move = "down"
            self.canvas.move("D3", 0, self.lineStep)
        else:
            move = "-"
        # print "Line 1 has " + str(row1) + " above and " + str(row2) + " below - should move " + move 

        if row2 > row3: 
            move = "up"
            self.canvas.move("D4", 0, -self.lineStep)
        elif row3 > row2:
            move = "down"
            self.canvas.move("D4", 0, self.lineStep)
        else:
            move = "-"
        # print "Line 2 has " + str(row2) + " above and " + str(row3) + " below - should move " + move  

        if col1 > col2: 
            move = "left"
            self.canvas.move("D1", -self.lineStep, 0)
        elif col2 > col1:
            move = "right"
            self.canvas.move("D1", self.lineStep, 0)
        else:
            move = "-"
        # print "Line 3 has " + str(col1) + " left and " + str(col2) + " right - should move " + move 

        if col2 > col3: 
            move = "left"
            self.canvas.move("D2", -self.lineStep, 0)
        elif col3 > col2:
            move = "right"
            self.canvas.move("D2", self.lineStep, 0)
        else:
            move = "-"
        # print "Line 4 has " + str(col2) + " left and " + str(col3) + " right - should move " + move 


    # Get the neighbouring locations of the specified location. Currently, this simply returns a 
    # hard-coded list of neighbours tailored to the asking location. Ideally, the neighbours would 
    # be calculated in a programmatic way.
    def getNeighbouringLocations(self, locationID):
        if locationID == 1:
            self.neighbouringLocations = [0, 0, 0, 2, 5, 4, 0, 0]
        elif locationID == 2:
            self.neighbouringLocations = [0, 0, 0, 3, 6, 5, 4, 1]
        elif locationID == 3:
            self.neighbouringLocations = [0, 0, 0, 0, 0, 6, 5, 2]
        elif locationID == 4:
            self.neighbouringLocations = [0, 1, 2, 5, 8, 7, 0, 0]
        elif locationID == 5:
            self.neighbouringLocations = [1, 2, 3, 6, 9, 8, 7, 4]
        elif locationID == 6:
            self.neighbouringLocations = [2, 3, 0, 0, 0, 9, 8, 5]
        elif locationID == 7:
            self.neighbouringLocations = [0, 4, 5, 8, 0, 0, 0, 0]
        elif locationID == 8:
            self.neighbouringLocations = [4, 5, 6, 9, 0, 0, 0, 7]
        elif locationID == 9:
            self.neighbouringLocations = [5, 6, 0, 0, 0, 0, 0, 8]

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

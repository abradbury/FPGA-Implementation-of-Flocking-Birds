#!/usr/bin/python
# -*- coding: utf-8 -*-


from boidCPU import BoidCPU       # Import the BoidCPU class
from boid import Boid               # Import the Boid class
from boidGPU import BoidGPU         # Import the BoidGPU class

import logging                      # Used to handle the textual output
import numpy as np                  # Used in various mathematical operations
import matplotlib.pyplot as plt     # Used to plot the graphs
import time                         # Used to time stuff


## MUST DOs ========================================================================================
# FIXME: Investigate arithmetic warning on rule calculation (causes boid to disappear from GUI)
# FIXME: Boids don't seem to be repelling each other that much, they are on top of one another
# FIXME: Obstacle avoidance does not seem to work well
# FIXME: Now that the boidCPUs resize, a boid's vision radius could cover a boidCPU that is not a 
#         direct neighbour of the current boidCPU which the boid resides in
# FIXME: Boids sometimes jump over the boidCPUs boundaries (i.e. temporary speedup). Also happens 
#         when the load balancing isn't done

# TODO: Modify code to handle different load balancing protocols
# TODO: Implement load balancing algorithm 1
#       - boidCPUs should know if they are overloaded
#       - They should signal the controller with a requested boundary step change
# TODO: Implement load balancing algorithm 2

## MAY DOs =========================================================================================
# TODO: Add keybinding to capture return key and simulate button press
# TODO: Add acceleration to smooth movement (especially around borders)
# TODO: Calculate a boidCPUs neighbours programmatically - rather than hardcoding
# TODO: Avoid double drawing - once for new position then again for rotation

# TODO: Only allow boids to see in front of them when looking at neighbours
# TODO: Add something so the boids can't immediately change direction - they have to turn


# The main application class. Sets up the simulation area and the number of boidCPUs that it is to 
# be divided into. 
#
# Note that the simulation begins with a user pressing the pauseButton (initially labelled 'Begin'). 
# Because the pause flag is initally set to true, in the action function the simulation is 'resumed'.
class Simulation:

    def __init__(self):
        # Setup the simulation parameters
        self.pauseSimulation = True
        self.timeStepCounter = 0

        # Define configuration parameters
        self.config = {}

        self.config['width'] = 720              # Define window size
        self.config['height'] = 720
        self.config['boidCount'] = 90
        self.config['updateInterval'] = 10      # The interval between successful update calls (ms) 
        self.config['widthInBoidCPUs'] = 3      # The number of BoidCPUs spanning the area width
        
        # Debugging parameters
        self.config['colourCode'] = False       # True to colour boids based on their BoidCPU
        self.config['trackBoid'] = True         # True to track the specified boid and neighbours
        self.config['boidToTrack'] = 2
        
        # Load balancing parameters
        self.config['loadBalance'] = True       # True to enable load balancing
        self.config['loadBalanceType'] = 1
        self.config['BOID_THRESHOLD'] = 30      # The maximum boids a BoidCPU should contain
        self.config['stepSize'] = 20            # The step size to change the boundaries

        # Boid movement parameters
        self.config['MAX_VELOCITY'] = 10
        self.config['VISION_RADIUS'] = 116      # Currently set to the width of a BoidGPU

        self.config['ALIGNMENT_WEIGHT'] = 1
        self.config['COHESION_WEIGHT'] = 1
        self.config['REPULSION_WEIGHT'] = 1

        # Setup logging
        self.setupLogging()

        # Instantiate the BoidGPU
        self.boidGPU = BoidGPU(self)
        self.boidGPU.initialiseSimulation()

        # Create the boidCPUs
        self.allowedBoidCPUCounts = np.array([1, 2, 4, 9, 16, 25, 36])
        self.boidCPUCount = self.allowedBoidCPUCounts[3]
        self.initialBoidCount = self.config['boidCount'] / self.boidCPUCount
        self.boidCPUSize = round(self.boidGPU.getWindowSize()[0] / np.sqrt(self.boidCPUCount))

        self.boidCPUs = []
        self.calcTimings = []
        self.drawTimings = []

        self.boidCPUColours = ["red", "blue", "green", "yellow", "white", "magenta", "dark grey", 
            "cyan", "dark green"]

        self.boidCPUCoords = np.array([0, 0, 0, 0])
        for i in range(0, self.boidCPUCount):
            self.boidCPUCoords[0] = (i % 3) * self.boidCPUSize
            self.boidCPUCoords[1] = int(np.floor(i / 3)) * self.boidCPUSize
            self.boidCPUCoords[2] = self.boidCPUCoords[0] + self.boidCPUSize
            self.boidCPUCoords[3] = self.boidCPUCoords[1] + self.boidCPUSize

            # Define the boidCPU's position in the grid of boidCPUs [row, col]
            self.boidCPUGridPos = [int(np.floor(i / 3)), (i % 3)]

            loc = BoidCPU(self.boidGPU, self, i + 1, self.boidCPUCoords, self.initialBoidCount, 
                self.boidCPUColours[i], self.boidCPUGridPos)
            self.boidCPUs.append(loc)

        # Setup variables to keep track of how many boidCPUs exceed the boid threshold
        self.violationCount = 0
        self.violationList = []

        self.logger.info("- Press the 'Begin' button to start the simulation")

        # Setup the graphs
        self.setupGraphs()
        self.setupSummaryGraph()

        # Start everything going
        self.boidGPU.beginMainLoop()


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

        for i in range(0, self.boidCPUCount):
            # Create the subplots and sync the axes
            if i != 0:
                axis = self.graphFigure.add_subplot(3, 3, i+1, sharex = self.axes[0], sharey = self.axes[0])
            else:
                axis = self.graphFigure.add_subplot(3, 3, i+1)

            axis.set_title("BoidCPU %d" % (i + 1))
            axis.grid(True)
            
            # Setup the first y axis
            for tl in axis.get_yticklabels():
                tl.set_color('b')
    
            # Setup the second y axis
            axis2 = axis.twinx()
            for tl in axis2.get_yticklabels():
                tl.set_color('r')
            axis2.set_ylim([0, self.config['boidCount']])

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


    # Create a sumary graph showing the number of boidCPUs exceeding the boid threshold over time
    def setupSummaryGraph(self):
        plt.ion()

        self.summaryFigure, self.summaryAxis = plt.subplots(1,1)
        self.summaryAxis.plot([], [])
        self.summaryAxis.fill_between([], 0, [])

        self.summaryAxis.set_ylim([0, self.boidCPUCount])
        
        self.summaryAxis.set_title("Graph showing number of boidCPUs exceeding the boid threshold")
        self.summaryAxis.grid(True)

        self.summaryAxis.set_xlabel("Number of time steps")
        self.summaryAxis.set_ylabel("BoidCPUs over threshold")

        plt.show()


    # For each boidCPU, get the graph lines and set the data to the current data  
    # plus the new data. Then reformat both axes to accommodate the new data.
    def updateGraphs(self):

        for i in range(0, self.boidCPUCount):
            # Update the boidCPU boid calculation time lines
            self.lines[i].set_xdata(self.boidCPUs[i].xData)
            self.lines[i].set_ydata(self.boidCPUs[i].yData)

            # Update the boidCPU boid count lines. Because the draw routine is called first, but 
            # not on time step 1, it has one less data element in than the values for the update 
            # stage. Therefore, [0:-1] is used to ignore the last x value.
            self.lines2[i].set_xdata(self.boidCPUs[i].xData[0:-1])
            self.lines2[i].set_ydata(self.boidCPUs[i].y2Data)

            # Update the Andall (sp?) lines
            self.andallLines[i].set_xdata([0, len(self.boidCPUs[i].xData)])
            self.andallLines[i].set_ydata([self.initialBoidCount, self.initialBoidCount])

            # Update the max boid threshold lines
            self.thresholdLines[i].set_xdata([0, len(self.boidCPUs[i].xData)])
            self.thresholdLines[i].set_ydata([self.config['BOID_THRESHOLD'], self.config['BOID_THRESHOLD']])

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
                for i in range(0, self.boidCPUCount):
                    self.logger.debug("Moving boids to calculated positions for boidCPU " + 
                        str(self.boidCPUs[i].boidCPUID) + "...")
        
                    # Update the canvas with the new boid positions
                    self.boidCPUs[i].update(True)

                    # Store the number of boids for later plotting
                    self.boidCPUs[i].y2Data.append(self.boidCPUs[i].boidCount)

            # Update the boid
            for i in range(0, self.boidCPUCount):
                self.logger.debug("Calculating next boid positions for boidCPU " + 
                    str(self.boidCPUs[i].boidCPUID) + "...")

                # Calculate the next boid positions and time this                
                self.startTime = time.clock()
                self.boidCPUs[i].update(False)
                self.endTime = time.clock()

                # Store the timing information for later plotting
                self.boidCPUs[i].xData.append(self.timeStepCounter)
                self.boidCPUs[i].yData.append((self.endTime - self.startTime) * 1000)

                # If the boidCPU is exceeding the threshold, increment counter
                if self.boidCPUs[i].boidCount > self.config['BOID_THRESHOLD']:
                    self.violationCount += 1

            self.logger.debug("BoidCPU boid counts: " + " ".join(str(loc.boidCount) 
                for loc in self.boidCPUs))

            # Update the counter label
            self.timeStepCounter += 1
            self.boidGPU.updateTimeStepLabel(self.timeStepCounter)

            # Update the violation list
            self.violationList.append(self.violationCount)

            # Call self after 20ms (50 Hz)
            self.boidGPU.nextSimulationStep(self.config['updateInterval'])
            

    
    # Manual timestep increment
    def nextStepButton(self):
        self.pauseSimulation = False
        self.simulationStep()
        self.pauseSimulation = True


    # Sets a flag that is used to pause and resume the simulation 
    def pause(self):
        if self.pauseSimulation == False:
            self.pauseSimulation = True
            self.boidGPU.togglePauseButton(False)

        else:
            self.pauseSimulation = False
            self.boidGPU.togglePauseButton(True)
            self.simulationStep()


    # Currently, this method changes the bounds of all the applicable edges of an overloaded 
    # boidCPU and the other bounds of the other boidCPUs that would be affected by this change.
    #
    # FIXME: Cannot currently handle when multiple boidCPUs are overloaded
    def boidCPUOverloaded(self, boidCPUID, requestedChange):
        self.logger.debug("BoidCPU " + str(boidCPUID) + " overloaded")

        # 1) Query the other boidCPUs to determine how the requested change would affect them
        # 2) Determine what action to take
        # 3) Implement change

        # logString = "BoidCPU " + str(boidCPUID) + " requests changing "
        # if requestedChange[0]:
        #     logString += "the TOP edge by " + str(requestedChange[0]) + " steps, "
        # if requestedChange[1]:
        #     logString += "the RIGHT edge by " + str(requestedChange[1]) + " steps, "
        # if requestedChange[2]:
        #     logString += "the BOTTOM edge by " + str(requestedChange[2]) + " steps, "
        # if requestedChange[3]:
        #     logString += "the LEFT edge by " + str(requestedChange[3]) + " steps, "
        # print logString

        # Determine the row and column of the boidCPU that is requesting load balancing
        [row, col] = self.boidCPUs[boidCPUID - 1].gridPosition

        # Determine which other BoidCPUs would be affected by this change
        boidCPUsToChange = self.identifyAffectedBoidCPUs(boidCPUID, row, col, requestedChange)

        # Update the edges of the affected BoidCPUs by the specified step size
        for edgeIndex, edgeBoidCPUs in enumerate(boidCPUsToChange):
            for boidCPU in edgeBoidCPUs:
                self.boidCPUs[boidCPU[0] - 1].changeBounds(edgeIndex, boidCPU[1], [row, col])

        # self.pause()

        # If there is a group of 60 boids, then as these pass between BoidCPUs, both BoidCPUs will 
        # request a edge change and there will be a bit of toing and froing.


    # Based on the position of the boidCPU in the simulation grid, determine which of the sides of 
    # the boidCPU would need changing (sides on simulation edge cannot be changed)
    def identifyAffectedBoidCPUs(self, boidCPUID, row, col, requestedChange):
        
        # If the BoidCPU doesn't specify a request for changing the edge, use 1 step for valid edges
        if not requestedChange:
            requestedChange = [0, 0, 0, 0] 
            if self.boidCPUs[boidCPUID - 1].validEdge(0):                   # Top
                requestedChange[0] = 1
            if self.boidCPUs[boidCPUID - 1].validEdge(1):                   # Right
                requestedChange[1] = 1
            if self.boidCPUs[boidCPUID - 1].validEdge(2):                   # Bottom
                requestedChange[2] = 1
            if self.boidCPUs[boidCPUID - 1].validEdge(3):                   # Left
                requestedChange[3] = 1

        # Create a list that is true if a change is requested for that edge and false otherwise
        edgeChanges = [True if v else False for v in requestedChange]

        # A structure to hold the BoidCPU edges to change. Used as follows:
        #  boidCPUsToChange[edgeIndex][boidCPUs][index 0 = boidCPUID, index 1 = stepChange]
        boidCPUsToChange = [[], [], [], []]

        # Iterate over the boidCPUs to determine the IDs of those boidCPUs that would be affected 
        # by the requested change and which edges of the boidCPUs would need changing
        for l in self.boidCPUs:
            if edgeChanges[0] and (l.gridPosition[0] == row - 1):
                boidCPUsToChange[2].append([l.boidCPUID, requestedChange[0]])

            if edgeChanges[1] and (l.gridPosition[1] == col + 1):
                boidCPUsToChange[3].append([l.boidCPUID, requestedChange[1]])

            if edgeChanges[2] and (l.gridPosition[0] == row + 1):
                boidCPUsToChange[0].append([l.boidCPUID, requestedChange[2]])

            if edgeChanges[3] and (l.gridPosition[1] == col - 1):
                boidCPUsToChange[1].append([l.boidCPUID, requestedChange[3]])


            if edgeChanges[0] and (l.gridPosition[0] == row):
                boidCPUsToChange[0].append([l.boidCPUID, requestedChange[0]])

            if edgeChanges[1] and (l.gridPosition[1] == col):
                boidCPUsToChange[1].append([l.boidCPUID, requestedChange[1]])

            if edgeChanges[2] and (l.gridPosition[0] == row):
                boidCPUsToChange[2].append([l.boidCPUID, requestedChange[2]])

            if edgeChanges[3] and (l.gridPosition[1] == col):
                boidCPUsToChange[3].append([l.boidCPUID, requestedChange[3]])

        # print boidCPUsToChange

        return boidCPUsToChange


    # Get the neighbouring boidCPUs of the specified boidCPU. Currently, this simply returns a 
    # hard-coded list of neighbours tailored to the asking boidCPU. Ideally, the neighbours would 
    # be calculated in a programmatic way.
    def getNeighbouringBoidCPUs(self, boidCPUID):
        if boidCPUID == 1:
            self.neighbouringBoidCPUs = [0, 0, 0, 2, 5, 4, 0, 0]
        elif boidCPUID == 2:
            self.neighbouringBoidCPUs = [0, 0, 0, 3, 6, 5, 4, 1]
        elif boidCPUID == 3:
            self.neighbouringBoidCPUs = [0, 0, 0, 0, 0, 6, 5, 2]
        elif boidCPUID == 4:
            self.neighbouringBoidCPUs = [0, 1, 2, 5, 8, 7, 0, 0]
        elif boidCPUID == 5:
            self.neighbouringBoidCPUs = [1, 2, 3, 6, 9, 8, 7, 4]
        elif boidCPUID == 6:
            self.neighbouringBoidCPUs = [2, 3, 0, 0, 0, 9, 8, 5]
        elif boidCPUID == 7:
            self.neighbouringBoidCPUs = [0, 4, 5, 8, 0, 0, 0, 0]
        elif boidCPUID == 8:
            self.neighbouringBoidCPUs = [4, 5, 6, 9, 0, 0, 0, 7]
        elif boidCPUID == 9:
            self.neighbouringBoidCPUs = [5, 6, 0, 0, 0, 0, 0, 8]

        return self.neighbouringBoidCPUs


    # Return a list of the boids for a specified boidCPU
    def getBoidCPUBoids(self, boidCPUID):
        return self.boidCPUs[boidCPUID - 1].getBoids()


    # Transfer a boid from one boidCPU to another
    def transferBoid(self, boid, toID, fromID):
        self.boidCPUs[toID - 1].acceptBoid(boid, fromID)


if __name__ == '__main__':
    # Start everything off
    boidSimulation = Simulation()

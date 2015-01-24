#!/usr/bin/python
# -*- coding: utf-8 -*-


from boidCPU import BoidCPU         # Import the BoidCPU class
from boid import Boid               # Import the Boid class
from boidGPU import BoidGPU         # Import the BoidGPU class

import logging                      # Used to handle the textual output
import numpy as np                  # Used in various mathematical operations
import matplotlib.pyplot as plt     # Used to plot the graphs

import time                         # Used to time stuff


## MUST DOs ========================================================================================
# FIXME: Boids don't seem to be repelling each other that much, they are on top of one another
# FIXME: Now that the boidCPUs resize, a boid's vision radius could cover a boidCPU that is not a 
#         direct neighbour of the current boidCPU which the boid resides in - affecting the result.

# TODO: Modify code to handle different load balancing protocols
# TODO: Enhance load balancing algorithm 1 so that BoidCPUs can be queried on proposed change
# TODO: Implement load balancing algorithm 2 - distribution already exists

## MAY DOs =========================================================================================
# TODO: Add keybinding to capture return key and simulate button press
# TODO: Calculate a boidCPUs neighbours programmatically - rather than hardcoding (and for 1 BoidCPU)

# TODO: Only allow boids to see in front of them when looking at neighbours
# TODO: Change simulation size to 1080p
# TODO: Experiment with rectangular BoidCPUs
# TODO: Parameterise numpy type
# TODO: No need to transfer boids if there is only one BoidCPU


# Load Balancing Types ==============================================================================
# 1) Simple load balancing - a BoidCPU signals to the controller that it is overloaded and the 
#   controller adjusts all of the BoidCPU's valid edges by one step size. 
# 
# 2) When a BoidCPU detects that it is overloaded, it analyses the distribution of boids in its area 
#   and uses this to request specific edge changes. The controller applies the requested changes.
# 
# 3) Same as type 2 but each BoidCPU is queried about the requested change. The BoidCPUs return the 
#   number of boids they expect to have after the edge change and the number of current boids. The
#   controller then determines the exact change to perform and issues the command. As an example, 
#   if BoidCPU 1 is overloaded and requests a change that would cause BoidCPU 2 to be overloaded,
#   the controller would adjust the request to try and avoid overloading BoidCPU 2.


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
        self.config['loggingLevel'] = logging.ERROR

        # Debugging parameters
        self.config['colourCode'] = False       # True to colour boids based on their BoidCPU
        self.config['trackBoid'] = False         # True to track the specified boid and neighbours
        self.config['boidToTrack'] = 2          # The ID of the boid to track, 0 for all boids
        
        # Load balancing parameters
        self.config['loadBalance'] = False       # True to enable load balancing
        self.config['loadBalanceType'] = 2      # See notes at top of file
        self.config['BOID_THRESHOLD'] = 30      # The maximum boids a BoidCPU should contain
        self.config['stepSize'] = 20            # The step size to change the boundaries

        self.config['percentageToRemove'] = 0.15
        self.config['boidsToRelease'] = self.config['BOID_THRESHOLD'] - (np.floor(self.config['BOID_THRESHOLD'] * (1 - self.config["percentageToRemove"])))

        # Boid movement parameters
        self.config['MAX_VELOCITY'] = 10
        self.config['MAX_FORCE'] = 1             # Determines how sharply a boid can turn
        self.config['VISION_RADIUS'] = 80      # Currently set to the width of a BoidGPU

        self.config['ALIGNMENT_WEIGHT'] = 1
        self.config['COHESION_WEIGHT'] = 1
        self.config['REPULSION_WEIGHT'] = 1

        self.config['minBoidCPUSize'] = self.config['VISION_RADIUS']

        # Define the testing state
        self.config['useTestingSetup'] = True   # True to use a known initial setup
        self.config['testingStopPoint'] = 500  # The timestep to stop the simulation for tests
        if self.config['useTestingSetup']:
            self.configureKnownInitialSetup()   # Makes the known initial setup available
        self.config["showBoidIds"] = False       # To show the boid and BoidCPU IDs

        # Setup logging
        self.setupLogging()

        # Instantiate the BoidGPU
        self.boidGPU = BoidGPU(self)
        self.boidGPU.initialiseSimulation()

        # Create the BoidCPUs
        self.allowedBoidCPUCounts = np.array([1, 2, 4, 9, 16, 25, 36])
        self.boidCPUCount = self.allowedBoidCPUCounts[3]
        self.initialBoidCount = self.config['boidCount'] / self.boidCPUCount
        self.boidCPUSize = round(self.config['width'] / np.sqrt(self.boidCPUCount))

        self.boidCPUs = []
        self.calcTimings = []
        self.drawTimings = []
        self.boidCPUCoords = np.array([0, 0, 0, 0])

        for i in range(0, self.boidCPUCount):
            # Define the BoidCPU's pixel position
            self.boidCPUCoords[0] = (i % 3) * self.boidCPUSize
            self.boidCPUCoords[1] = int(np.floor(i / 3)) * self.boidCPUSize
            self.boidCPUCoords[2] = self.boidCPUCoords[0] + self.boidCPUSize
            self.boidCPUCoords[3] = self.boidCPUCoords[1] + self.boidCPUSize

            # Define the BoidCPU's position in the grid of BoidCPUs [row, col]
            self.boidCPUGridPos = [int(np.floor(i / 3)), (i % 3)]

            # Create the BoidCPU and add to list
            loc = BoidCPU(self.boidGPU, self, i + 1, self.boidCPUCoords, self.initialBoidCount, 
                    self.boidCPUGridPos)
            self.boidCPUs.append(loc)

        # Setup variables to keep track of how many BoidCPUs exceed the boid threshold
        self.violationCount = 0
        self.violationList = []

        # Print out setup info for the simulation
        if self.config['loadBalance']:
            if self.config['loadBalanceType'] == 1:
                self.logger.info("BoidCPUs will signal when overloaded")
            elif self.config['loadBalanceType'] == 2:
                self.logger.info("BoidCPUs will analyse boid distribution to request load " +  
                    "balancing amounts")
            elif self.config['loadBalanceType'] == 3:
                self.logger.info("BoidCPUs will evaluate requested changes")
        else:
            self.logger.info("No load balancing will be performed")

        if self.config['useTestingSetup']:
            self.logger.info("Using testing setup for initialisation")
        else:
            self.logger.info("Using random setup for initialisation")
        
        self.logger.info("Press the 'Begin' button to start the simulation")

        # Setup the graphs
        self.boidGPU.setupGraphs(self.boidCPUCount)
        self.setupGraphData(4)

        # Print out the state of the boids in the simulation
        # self.saveState()

        # Start everything going
        self.boidGPU.beginMainLoop()


    # The main loop that is continually called during the simulation, though if the simulation is 
    # paused, it does nothing. 
    def simulationStep(self):
        if self.pauseSimulation == False:
            
            self.violationCount = 0

            self.drawStep()
            self.calcStep()
            self.loadStep()
            self.transferStep()

            # Update the counter label
            self.timeStepCounter += 1
            self.boidGPU.updateTimeStepLabel(self.timeStepCounter)

            # If testing, check to see if the simulation has reached the requested number of 
            # timesteps and if it has pause the simulation and update the graphs
            if self.config['useTestingSetup']:
                if self.timeStepCounter == self.config['testingStopPoint']:
                    self.pause()
                    self.boidGPU.updateGraphs()

            # Call self after 20ms (50 Hz)
            self.boidGPU.nextSimulationStep(self.config['updateInterval'])


    # The draw method is called first to enable a the neighbours of a boid to be highlighted 
    # when a boid is being followed (during debugging). As no new boid positions have been 
    # calculated, the draw function isn't called on the first time step.
    def drawStep(self):
        if self.timeStepCounter != 0:
            self.logger.debug("-"*70)

            for boidCPU in self.boidCPUs:
                self.logger.debug("Drawing boids at calculated positions for BoidCPU #" + 
                    str(boidCPU.BOIDCPU_ID) + "...")

                # Update the canvas with the new boid positions
                boidCPU.draw()

                # Store the number of boids for later plotting
                boidCPU.y2Data.append(boidCPU.boidCount)


    def setupGraphData(self, noOfDataTypes):
        self.graphData = [[[] for a in range(0, noOfDataTypes)] for b in range(0, self.boidCPUCount)]

    # def newGraphData(self, bcpuid, dtype):
        # self.graphData[bcpuid][dtype]

    def updateGraphData(self, bcpuid, dtype, data):
        self.graphData[bcpuid][dtype].append(data);


    def calcStep(self):
        self.logger.debug("=" + str(self.timeStepCounter) + "="*65)

        # Calculate the neighbours for each boid
        for boidCPU in self.boidCPUs:
            self.logger.debug("Calculating neighbours for BoidCPU #" + str(boidCPU.BOIDCPU_ID))
            
            startTime = time.clock()
            boidCPU.calculateBoidNeighbours()
            endTime = time.clock()

            # Store the timing information for later plotting
            data = (endTime - startTime) * 1000
            boidCPU.xData.append(self.timeStepCounter)
            boidCPU.yData.append(data)
            self.updateGraphData(boidCPU.BOIDCPU_ID - 1, 0, data)

        # Update each boid
        for boidCPU in self.boidCPUs:
            self.logger.debug("Calculating next boid positions for boidCPU " + 
                str(boidCPU.BOIDCPU_ID) + "...")

            # Calculate the next boid positions
            startTime = time.clock()
            boidCPU.update()
            endTime = time.clock()

            # Store the timing information for later plotting
            data = (endTime - startTime) * 1000
            boidCPU.yData[self.timeStepCounter] += (data)
            self.updateGraphData(boidCPU.BOIDCPU_ID - 1, 1, data)

            # If the boidCPU is exceeding the threshold, increment counter
            if boidCPU.boidCount > self.config['BOID_THRESHOLD']:
                self.violationCount += 1

        # Update the violation list
        self.violationList.append(self.violationCount)

        self.logger.debug("BoidCPU boid counts: " + " ".join(str(boidCPU.boidCount) 
            for boidCPU in self.boidCPUs))


    # If the number of boids in the boidCPU are greater than a threshold, signal controller
    def loadStep(self):
        if self.config['loadBalance']:
            for boidCPU in self.boidCPUs:

                startTime = time.clock()
                boidCPU.loadBalance()
                endTime = time.clock()

                # Store the timing information for later plotting
                data = (endTime - startTime) * 1000
                boidCPU.yData[self.timeStepCounter] += (data)
                self.updateGraphData(boidCPU.BOIDCPU_ID - 1, 2, data)


    # Determine if the new positions of the boids are outside their BoidCPU, if they are, transfere 
    # the boids to neighbouring BoidCPUS
    def transferStep(self):
        for boidCPU in self.boidCPUs:
            for boid in boidCPU.boids:

                startTime = time.clock()
                boidCPU.determineBoidTransfer(boid)
                endTime = time.clock()

                # Store the timing information for later plotting
                data = (endTime - startTime) * 1000
                boidCPU.yData[self.timeStepCounter] += (data)
                self.updateGraphData(boidCPU.BOIDCPU_ID - 1, 3, data)


    # Get the neighbouring boidCPUs of the specified boidCPU. Currently, this simply returns a 
    # hard-coded list of neighbours tailored to the asking boidCPU. Ideally, the neighbours would 
    # be calculated in a programmatic way.
    def getNeighbouringBoidCPUs(self, BOIDCPU_ID):
        # if BOIDCPU_ID == 1:
        #     neighbouringBoidCPUs = [0, 0, 0, 2, 5, 4, 0, 0]
        # elif BOIDCPU_ID == 2:
        #     neighbouringBoidCPUs = [0, 0, 0, 3, 6, 5, 4, 1]
        # elif BOIDCPU_ID == 3:
        #     neighbouringBoidCPUs = [0, 0, 0, 0, 0, 6, 5, 2]
        # elif BOIDCPU_ID == 4:
        #     neighbouringBoidCPUs = [0, 1, 2, 5, 8, 7, 0, 0]
        # elif BOIDCPU_ID == 5:
        #     neighbouringBoidCPUs = [1, 2, 3, 6, 9, 8, 7, 4]
        # elif BOIDCPU_ID == 6:
        #     neighbouringBoidCPUs = [2, 3, 0, 0, 0, 9, 8, 5]
        # elif BOIDCPU_ID == 7:
        #     neighbouringBoidCPUs = [0, 4, 5, 8, 0, 0, 0, 0]
        # elif BOIDCPU_ID == 8:
        #     neighbouringBoidCPUs = [4, 5, 6, 9, 0, 0, 0, 7]
        # elif BOIDCPU_ID == 9:
        #     neighbouringBoidCPUs = [5, 6, 0, 0, 0, 0, 0, 8]

        # Use these if the boundaries are wrap-around
        if BOIDCPU_ID == 1:
            neighbouringBoidCPUs = [9, 7, 8, 2, 5, 4, 6, 3]
        elif BOIDCPU_ID == 2:
            neighbouringBoidCPUs = [7, 8, 9, 3, 6, 5, 4, 1]
        elif BOIDCPU_ID == 3:
            neighbouringBoidCPUs = [8, 9, 7, 1, 4, 6, 5, 2]
        elif BOIDCPU_ID == 4:
            neighbouringBoidCPUs = [3, 1, 2, 5, 8, 7, 9, 6]
        elif BOIDCPU_ID == 5:
            neighbouringBoidCPUs = [1, 2, 3, 6, 9, 8, 7, 4]
        elif BOIDCPU_ID == 6:
            neighbouringBoidCPUs = [2, 3, 1, 4, 7, 9, 8, 5]
        elif BOIDCPU_ID == 7:
            neighbouringBoidCPUs = [6, 4, 5, 8, 2, 1, 3, 9]
        elif BOIDCPU_ID == 8:
            neighbouringBoidCPUs = [4, 5, 6, 9, 3, 2, 1, 7]
        elif BOIDCPU_ID == 9:
            neighbouringBoidCPUs = [5, 6, 4, 7, 1, 3, 2, 8]

        return neighbouringBoidCPUs


    # Return a list of the boids for a specified boidCPU
    def getBoidCPUBoids(self, BOIDCPU_ID):
        return self.boidCPUs[BOIDCPU_ID - 1].getBoids()


    # Transfer a boid from one boidCPU to another
    def transferBoid(self, boid, toID, fromID):
        self.boidCPUs[toID - 1].acceptBoid(boid, fromID)


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


    # Change the boid aligment weighting
    def changeBoidAlignment(self, value):
        self.config['ALIGNMENT_WEIGHT'] = float(value)
        self.logger.debug("Boid alignment changed to " + str(value))


    # Change the boid cohesion weighting
    def changeBoidCohesion(self, value):
        self.config['COHESION_WEIGHT'] = float(value)
        self.logger.debug("Boid cohesion changed to " + str(value))


    # Change the boid separation weighting
    def changeBoidSeparation(self, value):
        self.config['REPULSION_WEIGHT'] = float(value)
        self.logger.debug("Boid separation changed to " + str(value))


    # Change the boid vision radius
    def changeVisionRadius(self, value):
        self.config['VISION_RADIUS'] = int(value)
        self.logger.debug("Boid vision radius changed to " + str(value))


    # Change the maximum boid velocity
    def changeMaxVelocity(self, value):
        self.config['MAX_VELOCITY'] = int(value)
        self.logger.debug("Boid maximum velocity changed to " + str(value))


    # Change the maximum boid force
    def changeMaxForce(self, value):
        self.config['MAX_FORCE'] = float(value)
        self.logger.debug("Boid maximum force changed to " + str(value))

    # Toggle the track boid status
    def toggleTrackBoid(self):
        self.config['trackBoid'] = not self.config['trackBoid']
        self.boidGPU.removeObject("boidCircle" + str(self.config['boidToTrack']))
        self.logger.debug("Tracking boid changed to " + str(self.config['trackBoid']))


    # Setup logging
    def setupLogging(self):   
        self.logger = logging.getLogger('boidSimulation')
        self.logger.setLevel(self.config['loggingLevel'])

        self.ch = logging.StreamHandler()
        self.ch.setLevel(self.config['loggingLevel'])

        self.formatter = logging.Formatter("[%(levelname)8s] --- %(message)s")
        self.ch.setFormatter(self.formatter)
        self.logger.addHandler(self.ch)


    ################################################################################################
    ## Load Balancing Functions ------------------------------------------------------------------##
    ################################################################################################

    # Called when a BoidCPU is overloaded with boids. If the BoidCPU supplies a requested load 
    # balancing option for each of its valid edges then these are used outright. If the BoidCPU 
    # does not supply a request, then a step size of 1 is assumed for all of the BoidCPU's valid 
    # edges.
    # 
    # The other BoidCPUs that will be affected by the changes are identified and then the edges of 
    # all the affected BoidCPUs are adjusted in the requested manner.
    #
    # FIXME: Cannot currently handle when multiple boidCPUs are overloaded
    def boidCPUOverloaded(self, BOIDCPU_ID, requestedChange):
        self.logger.debug("BoidCPU #" + str(BOIDCPU_ID) + " is overloaded")

        # Determine the row and column of the boidCPU that is requesting load balancing
        [row, col] = self.boidCPUs[BOIDCPU_ID - 1].gridPosition

        # Determine which other BoidCPUs would be affected by this change
        [boidCPUsToChange, alt] = self.identifyAffectedBoidCPUs(BOIDCPU_ID, row, col, requestedChange)

        # Query the affected BoidCPUs to determine what the effect of the change would be
        if self.config['loadBalanceType'] == 3:
            newBoidCounts = []
            oldBoidCounts = []
            for boidCPUIndex, boidCPUEdges in enumerate(alt):
                [old, new] = self.boidCPUs[boidCPUIndex].evaluateBoundaryChange(boidCPUEdges, [row, col])
                newBoidCounts.append(new)
                oldBoidCounts.append(old)

            self.logger.debug("Old boid counts: " + str(oldBoidCounts) + " - total: " + 
                str(sum(oldBoidCounts)))
            self.logger.debug("New boid counts: " + str(newBoidCounts) + " - total: " + 
                str(sum(newBoidCounts)))

            # TODO: Use the predicted boid levels to evaluate change request
            #   Give overloaded info before analysis of distribution to limit options

        # Update the edges of the affected BoidCPUs by the specified step size
        for edgeIndex, edgeBoidCPUs in enumerate(boidCPUsToChange):
            for boidCPU in edgeBoidCPUs:
                self.boidCPUs[boidCPU[0] - 1].changeBounds(edgeIndex, boidCPU[1], [row, col])


        for boidCPU in self.boidCPUs:
            boidCPUWidth = boidCPU.boidCPUCoords[2] - boidCPU.boidCPUCoords[0]
            boidCPUHeight = boidCPU.boidCPUCoords[3] - boidCPU.boidCPUCoords[1]

            self.logger.debug("New BoidCPU #" + str(boidCPU.BOIDCPU_ID) + " width = " + str(boidCPUWidth) + ", height = " + str(boidCPUHeight))

        # self.pause()


    # Prints out the requested edge changes
    def printRequestedBoundChange(self, BOIDCPU_ID, requestedChange):
        stringStart = "BoidCPU #" + str(BOIDCPU_ID) + " requests changing "
        stringParts = []

        for edgeID, change in enumerate(requestedChange):
            stringParts.append("edge " + str(edgeID) + " by " + str(change) + " steps")

        self.logger.debug(stringStart + ", ".join(stringParts))


    # Based on the position of the boidCPU in the simulation grid, determine which of the sides of 
    # the boidCPU would need changing (sides on simulation edge cannot be changed)
    def identifyAffectedBoidCPUs(self, BOIDCPU_ID, row, col, requestedChange):
        
        # If the BoidCPU doesn't specify a request for changing the edge, use 1 step for all valid 
        # edges and check that the new size is greater than the minimum needed
        if not requestedChange:
            requestedChange = [0, 0, 0, 0] 
            change = 1

            if self.boidCPUs[BOIDCPU_ID - 1].validEdge(0):           # If the BoidCPU has a top edge
                if self.boidCPUs[BOIDCPU_ID - 1].minSizeEnforced(0, change): # If change within constraints
                    requestedChange[0] = change                           # Change the top edge

            if self.boidCPUs[BOIDCPU_ID - 1].validEdge(1):           # If the BoidCPU has a right edge
                if self.boidCPUs[BOIDCPU_ID - 1].minSizeEnforced(1, change): # If change within constraints
                    requestedChange[1] = change                           # Change the right edge

            if self.boidCPUs[BOIDCPU_ID - 1].validEdge(2):           # If the BoidCPU has a bottom edge
                if self.boidCPUs[BOIDCPU_ID - 1].minSizeEnforced(2, change): # If change within constraints
                    requestedChange[2] = change                           # Change the bottom edge

            if self.boidCPUs[BOIDCPU_ID - 1].validEdge(3):           # If the BoidCPU has a left edge
                if self.boidCPUs[BOIDCPU_ID - 1].minSizeEnforced(3, change): # If change within constraints
                    requestedChange[3] = change                           # Change the left edge

        self.printRequestedBoundChange(BOIDCPU_ID, requestedChange)

        # Create a list that is true if a change is requested for that edge and false otherwise
        edgeChanges = [True if v else False for v in requestedChange]

        # A structure to hold the BoidCPU edges to change. Used as follows:
        #  boidCPUsToChange[edgeIndex][boidCPUs][index 0 = BOIDCPU_ID, index 1 = stepChange]
        boidCPUsToChange = [[], [], [], []]

        boidCPUsToChangeB = [[] for b in range(self.boidCPUCount)]

        # Iterate over the boidCPUs to determine the IDs of those boidCPUs that would be affected 
        # by the requested change and which edges of the boidCPUs would need changing
        for b in self.boidCPUs:
            if edgeChanges[0] and (b.gridPosition[0] == row - 1):
                boidCPUsToChange[2].append([b.BOIDCPU_ID, requestedChange[0]])
                boidCPUsToChangeB[b.BOIDCPU_ID - 1].append([2, requestedChange[0]])

            if edgeChanges[1] and (b.gridPosition[1] == col + 1):
                boidCPUsToChange[3].append([b.BOIDCPU_ID, requestedChange[1]])
                boidCPUsToChangeB[b.BOIDCPU_ID - 1].append([3, requestedChange[1]])

            if edgeChanges[2] and (b.gridPosition[0] == row + 1):
                boidCPUsToChange[0].append([b.BOIDCPU_ID, requestedChange[2]])
                boidCPUsToChangeB[b.BOIDCPU_ID - 1].append([0, requestedChange[2]])

            if edgeChanges[3] and (b.gridPosition[1] == col - 1):
                boidCPUsToChange[1].append([b.BOIDCPU_ID, requestedChange[3]])
                boidCPUsToChangeB[b.BOIDCPU_ID - 1].append([1, requestedChange[3]])

            if edgeChanges[0] and (b.gridPosition[0] == row):
                boidCPUsToChange[0].append([b.BOIDCPU_ID, requestedChange[0]])
                boidCPUsToChangeB[b.BOIDCPU_ID - 1].append([0, requestedChange[0]])

            if edgeChanges[1] and (b.gridPosition[1] == col):
                boidCPUsToChange[1].append([b.BOIDCPU_ID, requestedChange[1]])
                boidCPUsToChangeB[b.BOIDCPU_ID - 1].append([1, requestedChange[1]])

            if edgeChanges[2] and (b.gridPosition[0] == row):
                boidCPUsToChange[2].append([b.BOIDCPU_ID, requestedChange[2]])
                boidCPUsToChangeB[b.BOIDCPU_ID - 1].append([2, requestedChange[2]])

            if edgeChanges[3] and (b.gridPosition[1] == col):
                boidCPUsToChange[3].append([b.BOIDCPU_ID, requestedChange[3]])
                boidCPUsToChangeB[b.BOIDCPU_ID - 1].append([3, requestedChange[3]])

        return [boidCPUsToChange, boidCPUsToChangeB]


    ################################################################################################
    ## Testing Functions -------------------------------------------------------------------------##
    ################################################################################################

    # Used to print out a string to the commandline which can be used to recreate the starting 
    # conditions of the simulation.
    def saveState(self):
        savedState = [0 for i in range(0, self.boidCPUCount)]
        for boidCPU in self.boidCPUs:
            savedState[boidCPU.BOIDCPU_ID - 1] = boidCPU.saveState()
        print savedState


    # Makes the known test state available in the configuration list
    def configureKnownInitialSetup(self):

        # Single BoidCPU with 10 boids
        # self.config['testState'] = ([[[51, [12, 11], [5, 0]], 
        #     [52, [19, 35], [-5, 1]], 
        #     [53, [12, 31], [-4, -2]], 
        #     [54, [35, 22], [0, -3]],
        #     [55, [4, 9], [-1, 0]],
        #     [56, [19, 18], [2, -3]],
        #     [57, [38, 19], [4, -4]],
        #     [58, [18, 5], [-1, 2]],
        #     [59, [15, 33], [2, -2]],
        #     [60, [3, 8], [-2, 0]]]])

        # self.config['testState'] = ([[[1, [106.0, 5.0], [2.0, -5.0]]], 
        #     [[2, [406.0, 193.0], [8.0, -3.0]]], 
        #     [[3, [495.0, 129.0], [3.0, -10.0]]], 
        #     [[4, [180.0, 273.0], [10.0, 3.0]]], 
        #     [[5, [283.0, 390.0], [8.0, 8.0]]], 
        #     [[6, [695.0, 252.0], [-5.0, -9.0]]], 
        #     [[7, [23.0, 565.0], [10.0, -9.0]]], 
        #     [[8, [387.0, 605.0], [5.0, -6.0]]], 
        #     [[9, [501.0, 678.0], [5.0, 0.0]]]])

        # self.config['testState'] = ([[[1, [174.0, 114.0], [9.0, 8.0]]], [[2, [288.0, 20.0], [2.0, 8.0]]], [[3, [507.0, 227.0], [9.0, -8.0]]], [[4, [44.0, 366.0], [-8.0, 0.0]]], [[5, [450.0, 347.0], [1.0, 4.0]]], [[6, [579.0, 467.0], [8.0, -3.0]]], [[7, [168.0, 490.0], [4.0, 9.0]]], [[8, [265.0, 593.0], [-2.0, -9.0]]], [[9, [690.0, 685.0], [-8.0, -7.0]]]])

        # self.config['testState'] = ([[[1, [50.0, 113.0], [4.0, 0.0]]], [[2, [374.0, 184.0], [1.0, 0.0]]], [[3, [546.0, 99.0], [9.0, -8.0]]], [[4, [69.0, 326.0], [7.0, -9.0]]], [[5, [305.0, 246.0], [-6.0, 1.0]]], [[6, [697.0, 339.0], [2.0, -8.0]]], [[7, [109.0, 663.0], [2.0, -4.0]]], [[8, [402.0, 589.0], [2.0, 4.0]]], [[9, [539.0, 700.0], [3.0, 3.0]]]])

        # self.config['testState'] = ([[[1, [176.0, 108.0], [-2.0, 8.0]]], [[2, [467.0, 151.0], [-10.0, -2.0]]], [[3, [542.0, 164.0], [-2.0, -6.0]]], [[4, [186.0, 416.0], [9.0, -10.0]]], [[5, [435.0, 448.0], [0.0, 4.0]]], [[6, [668.0, 406.0], [-7.0, 3.0]]], [[7, [205.0, 552.0], [-6.0, -9.0]]], [[8, [348.0, 540.0], [-5.0, -9.0]]], [[9, [577.0, 534.0], [-9.0, -1.0]]]])

        self.config['testState'] = ([[[1, [106.0, 5.0], [2.0, -5.0]], [2, [227.0, 76.0], [-6.0, 2.0]], 
            [3, [106.0, 44.0], [-7.0, 0.0]], [4, [7.0, 152.0], [-4.0, 10.0]], 
            [5, [168.0, 7.0], [1.0, 5.0]], [6, [158.0, 199.0], [10.0, -9.0]], 
            [7, [136.0, 79.0], [-1.0, 5.0]], [8, [224.0, 127.0], [6.0, -2.0]], 
            [9, [155.0, 70.0], [-4.0, -5.0]], [10, [129.0, 81.0], [0.0, 2.0]]],

            [[11, [406.0, 193.0], [8.0, -3.0]], [12, [357.0, 111.0], [8.0, 4.0]], 
            [13, [293.0, 46.0], [6.0, 7.0]], [14, [476.0, 99.0], [-7.0, 0.0]], 
            [15, [294.0, 28.0], [10.0, 10.0]], [16, [271.0, 179.0], [9.0, 1.0]], 
            [17, [403.0, 194.0], [7.0, 9.0]], [18, [412.0, 110.0], [9.0, -6.0]], 
            [19, [391.0, 69.0], [-1.0, -1.0]], [20, [454.0, 115.0], [-8.0, 8.0]]], 

            [[21, [495.0, 129.0], [3.0, -10.0]], [22, [699.0, 156.0], [4.0, -9.0]], 
            [23, [651.0, 233.0], [7.0, 3.0]], [24, [657.0, 98.0], [-5.0, -7.0]], 
            [25, [571.0, 29.0], [-3.0, -4.0]], [26, [718.0, 25.0], [-6.0, -10.0]], 
            [27, [640.0, 173.0], [-4.0, 5.0]], [28, [634.0, 35.0], [3.0, -5.0]], 
            [29, [594.0, 193.0], [2.0, -10.0]], [30, [657.0, 151.0], [6.0, -9.0]]],

            [[31, [180.0, 273.0], [10.0, 3.0]], [32, [174.0, 397.0], [-9.0, 9.0]], 
            [33, [165.0, 383.0], [-7.0, 3.0]], [34, [224.0, 329.0], [0.0, 6.0]], 
            [35, [13.0, 267.0], [8.0, -10.0]], [36, [235.0, 361.0], [-9.0, 9.0]], 
            [37, [192.0, 283.0], [1.0, -6.0]], [38, [176.0, 478.0], [7.0, -2.0]], 
            [39, [53.0, 314.0], [1.0, -4.0]], [40, [123.0, 484.0], [-1.0, 1.0]]], 

            [[41, [283.0, 390.0], [8.0, 8.0]], [42, [478.0, 382.0], [7.0, 7.0]], 
            [43, [335.0, 267.0], [-3.0, 4.0]], [44, [338.0, 244.0], [-3.0, -2.0]], 
            [45, [479.0, 313.0], [4.0, -1.0]], [46, [337.0, 288.0], [7.0, 1.0]], 
            [47, [324.0, 455.0], [5.0, -2.0]], [48, [272.0, 389.0], [-9.0, 10.0]], 
            [49, [356.0, 270.0], [-2.0, -5.0]], [50, [263.0, 362.0], [-6.0, 7.0]]],

            [[51, [695.0, 252.0], [-5.0, -9.0]], [52, [594.0, 404.0], [-10.0, -1.0]], 
            [53, [550.0, 350.0], [-10.0, -3.0]], [54, [661.0, 446.0], [-6.0, -4.0]], 
            [55, [539.0, 283.0], [-8.0, -2.0]], [56, [551.0, 256.0], [-5.0, 7.0]], 
            [57, [644.0, 342.0], [-1.0, -7.0]], [58, [592.0, 399.0], [-9.0, 6.0]], 
            [59, [644.0, 252.0], [-5.0, -8.0]], [60, [687.0, 478.0], [-9.0, 9.0]]],

            [[61, [23.0, 565.0], [10.0, -9.0]], [62, [85.0, 550.0], [9.0, -1.0]], 
            [63, [115.0, 549.0], [7.0, 1.0]], [64, [2.0, 506.0], [-9.0, -10.0]], 
            [65, [50.0, 499.0], [-1.0, 3.0]], [66, [1.0, 524.0], [6.0, -9.0]], 
            [67, [66.0, 613.0], [-10.0, -4.0]], [68, [171.0, 520.0], [5.0, -8.0]], 
            [69, [117.0, 516.0], [4.0, 0.0]], [70, [49.0, 637.0], [-6.0, 6.0]]], 

            [[71, [387.0, 605.0], [5.0, -6.0]], [72, [322.0, 655.0], [4.0, 8.0]], 
            [73, [342.0, 498.0], [-6.0, -1.0]], [74, [392.0, 577.0], [3.0, -2.0]], 
            [75, [468.0, 534.0], [-4.0, 8.0]], [76, [331.0, 529.0], [5.0, -7.0]], 
            [77, [348.0, 608.0], [-9.0, 6.0]], [78, [455.0, 705.0], [-1.0, -1.0]], 
            [79, [404.0, 653.0], [10.0, -5.0]], [80, [246.0, 505.0], [-1.0, -3.0]]],

            [[81, [501.0, 678.0], [5.0, 0.0]], [82, [711.0, 681.0], [-7.0, 5.0]], 
            [83, [638.0, 622.0], [-4.0, -9.0]], [84, [649.0, 628.0], [9.0, -4.0]], 
            [85, [710.0, 722.0], [-10.0, -9.0]], [86, [510.0, 603.0], [10.0, 7.0]], 
            [87, [569.0, 695.0], [4.0, 10.0]], [88, [718.0, 520.0], [-4.0, 10.0]], 
            [89, [726.0, 717.0], [4.0, 8.0]], [90, [713.0, 573.0], [-3.0, 5.0]]]])


if __name__ == '__main__':
    # Start everything off
    boidSimulation = Simulation()

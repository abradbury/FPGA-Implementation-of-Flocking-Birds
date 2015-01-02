#!/usr/bin/python
# -*- coding: utf-8 -*-


from boid import Boid               # Import the Boid class

import numpy as np                  # Used in various mathematical operations
import logging                      # Used to handle the textual output
import random                       # Used to randomly position the boids on initialisation


# A class representing a BoidCPU. A BoidCPU is an entity that controls all the 
# boids that enter its section of the simulation space. When boids exit a 
# BoidCPU's space, they are transfered to a neighbouring BoidCPU.
class BoidCPU:

    def __init__(self, boidGPU, _simulation, _boidCPUID, _boidCPUCoords, _initialBoidCount, _gridPosition):
        self.simulation = _simulation
        self.boidCPUID = _boidCPUID
        self.boidCPUCoords = np.copy(_boidCPUCoords)
        self.logger = self.simulation.logger
        self.gridPosition = _gridPosition
        self.boidGPU = boidGPU

        # Make the system configuration list available
        self.config = self.simulation.config

        # Define boid structures
        self.boids = []
        self.initialBoidCount = _initialBoidCount
        self.boidCount = self.initialBoidCount

        # Used to hold the plotting data
        self.xData = []
        self.yData = []
        self.y2Data = []

        # If the boidCPU bounds are to be coloured differently, generate a random colour
        # From: http://stackoverflow.com/a/14019260 
        if self.config['colourCode']:
            r = lambda: random.randint(0,255)
            self.colour = str('#%02X%02X%02X' % (r(),r(),r()))
        else:  
            self.colour = "yellow"

        # Draw the boidCPU bounds
        self.boidGPU.drawBoidCPU(self.boidCPUCoords, self.boidCPUID, self.colour)

        # If testing, use a known boid setup, else use a random setup
        if self.config['useTestingSetup']:
            # Get the boid details from the test state
            for boidInfo in self.config['testState'][self.boidCPUID - 1]:
                position = np.array(boidInfo[1])
                velocity = np.array(boidInfo[2])

                # Create the boid and add to list
                boid = Boid(self.boidGPU, self, boidInfo[0], position, velocity, self.colour)
                self.boids.append(boid)
        else:
            for i in range (0, self.boidCount):
                # Randomly position the boid on initialisation
                posX = random.randint(self.boidCPUCoords[0], self.boidCPUCoords[2]);
                posY = random.randint(self.boidCPUCoords[1], self.boidCPUCoords[3]);
                position = np.array([posX, posY], dtype = np.float_)

                # Randomly generate the boid's initial velocity
                velX = random.randint(-self.config['MAX_VELOCITY'], self.config['MAX_VELOCITY'])
                velY = random.randint(-self.config['MAX_VELOCITY'], self.config['MAX_VELOCITY'])
                velocity = np.array([velX, velY], dtype = np.float_)

                # Specify the boid's ID
                boidID = ((self.boidCPUID - 1) * self.boidCount) + i + 1

                # Create the boid and add to boid list
                boid = Boid(self.boidGPU, self, boidID, position, velocity, self.colour)
                self.boids.append(boid)

        self.logger.info("Created boidCPU " + str(self.boidCPUID) + " with " + 
            str(self.boidCount) + " boids") 


    # Calculate the next positions of every boid in the boidCPU. Then determine if any of the boids 
    # need to be transferred to neighbouring boidCPUs based on their new positions. 
    def update(self):
        self.possibleNeighbouringBoids = self.getPossibleNeighbouringBoids()

        # For each boid, calculate its new position
        for i in range(0, self.boidCount):
            self.boids[i].update(self.possibleNeighbouringBoids)

        # If the number of boids in the boidCPU are greater than a threshold, signal controller
        if self.config['loadBalance']:
            if self.boidCount >= self.config['BOID_THRESHOLD']:

                # Analyse the distribution of the boids in this BoidCPU to determine the step
                if (self.config['loadBalanceType'] == 2) or (self.config['loadBalanceType'] == 3): 
                    self.createBoidDistribution()
                    requestedChange = self.analyseBoidDistribution()
                elif self.config['loadBalanceType'] == 1: 
                    requestedChange = None

                self.simulation.boidCPUOverloaded(self.boidCPUID, requestedChange)

                # self.simulation.boidCPUOverloaded(self.boidCPUID)

        # Determine if the new positions of the boids lie outside the boidCPU's bounds
        for boid in self.boids:
            self.determineBoidTransfer(boid)
            

    # Draw the new positions of the boids.
    def draw(self):
        for i in range(0, self.boidCount):
            self.boids[i].draw(self.colour)


    # Return a list containing the boids currently controlled by this boidCPU
    def getBoids(self):
        return self.boids


    # Return a list containing the boids from each neighbouring boidCPU
    def getPossibleNeighbouringBoids(self):
        self.neighbouringBoidCPUs = self.simulation.getNeighbouringBoidCPUs(self.boidCPUID)

        # Need the slice operation or else updating 
        self.neighbouringBoids = self.boids[:]

        for boidCPUIndex in self.neighbouringBoidCPUs:
            if boidCPUIndex != 0:
                self.neighbouringBoids += self.simulation.getBoidCPUBoids(boidCPUIndex)

        return self.neighbouringBoids


    # Used to return the state of the initial setup of the boids for testing purposes
    def saveState(self):
        self.savedState = []

        for boid in self.boids:
            boidState = [boid.boidID, boid.position.tolist(), boid.velocity.tolist()]
            self.savedState.append(boidState)

        # print self.savedState
        return self.savedState


    ################################################################################################
    ## BoidCPU to BoidCPU Boid Transfer Functions ------------------------------------------------##
    ################################################################################################

    def determineBoidTransfer(self, boid):
        # If the boidCPU has a neighbour to the NORTHWEST and the boid is beyond its northern AND western boundaries
        if (self.neighbouringBoidCPUs[0] != 0) and (boid.position[1] < self.boidCPUCoords[1]) and (boid.position[0] < self.boidCPUCoords[0]):
            self.logger.debug("Boid " + str(boid.boidID) + 
                " is beyond the NORTH and WESTERN boundaries of boidCPU " + str(self.boidCPUID))
            self.transferBoid(boid, self.neighbouringBoidCPUs[0])

        # If the boidCPU has a neighbour to the NORTHEAST and the boid is beyond its northern AND eastern boundaries
        elif (self.neighbouringBoidCPUs[2] != 0) and (boid.position[1] < self.boidCPUCoords[1]) and (boid.position[0] > self.boidCPUCoords[2]):
            self.logger.debug("Boid " + str(boid.boidID) + 
                " is beyond the NORTH and EASTERN boundaries of boidCPU " + str(self.boidCPUID))
            self.transferBoid(boid, self.neighbouringBoidCPUs[2])

        # If the boidCPU has a neighbour to the SOUTHEAST and the boid is beyond its southern AND eastern boundaries
        elif (self.neighbouringBoidCPUs[4] != 0) and (boid.position[1] > self.boidCPUCoords[3]) and (boid.position[0] > self.boidCPUCoords[2]):
            self.logger.debug("Boid " + str(boid.boidID) + 
                " is beyond the SOUTHERN and EASTERN boundaries of boidCPU " + str(self.boidCPUID))
            self.transferBoid(boid, self.neighbouringBoidCPUs[4])

        # If the boidCPU has a neighbour to the SOUTHWEST and the boid is beyond its southern AND western boundaries
        elif (self.neighbouringBoidCPUs[6] != 0) and (boid.position[1] > self.boidCPUCoords[3]) and (boid.position[0] < self.boidCPUCoords[0]):
            self.logger.debug("Boid " + str(boid.boidID) + 
                " is beyond the SOUTHERN and WESTERN boundaries of boidCPU " + str(self.boidCPUID))
            self.transferBoid(boid, self.neighbouringBoidCPUs[6])

        # If the boidCPU has a neighbour to the NORTH and the boid is beyond its northern boundary
        elif (self.neighbouringBoidCPUs[1] != 0) and (boid.position[1] < self.boidCPUCoords[1]):
            self.logger.debug("Boid " + str(boid.boidID) + 
                " is beyond the NORTH boundary of boidCPU " + str(self.boidCPUID))
            self.transferBoid(boid, self.neighbouringBoidCPUs[1])

        # If the boidCPU has a neighbour to the EAST and the boid is beyond its eastern boundary
        elif (self.neighbouringBoidCPUs[3] != 0) and (boid.position[0] > self.boidCPUCoords[2]):
            self.logger.debug("Boid " + str(boid.boidID) + 
                " is beyond the EAST boundary of boidCPU " + str(self.boidCPUID))
            self.transferBoid(boid, self.neighbouringBoidCPUs[3])  
            
        # If the boidCPU has a neighbour to the SOUTH and the boid is beyond its southern boundary
        elif (self.neighbouringBoidCPUs[5] != 0) and (boid.position[1] > self.boidCPUCoords[3]):
            self.logger.debug("Boid " + str(boid.boidID) + 
                " is beyond the SOUTH boundary of boidCPU " + str(self.boidCPUID))
            self.transferBoid(boid, self.neighbouringBoidCPUs[5])

        # If the boidCPU has a neighbour to the WEST and the boid is beyond its western boundary
        elif (self.neighbouringBoidCPUs[7] != 0) and (boid.position[0] < self.boidCPUCoords[0]):
            self.logger.debug("Boid " + str(boid.boidID) + 
                " is beyond the WEST boundary of boidCPU " + str(self.boidCPUID))
            self.transferBoid(boid, self.neighbouringBoidCPUs[7])


    # Accept a boid transferred from another boidCPU and add it to this boidCPUs boid list
    def acceptBoid(self, boid, fromID):
        boid.boidCPU = self

        self.boids.append(boid)
        self.boidCount += 1

        self.logger.debug("BoidCPU " + str(self.boidCPUID) + " accepted boid " + str(boid.boidID) + 
            " from boidCPU " + str(fromID) + " and now has " + str(self.boidCount) + " boids")


    # Transfer a boid from this boidCPU to another boidCPU
    def transferBoid(self, boid, toID):
        self.logger.debug("BoidCPU " + str(self.boidCPUID) + " sent boid " + str(boid.boidID) + 
            " to boidCPU " + str(toID) + " and now has " + str(self.boidCount - 1) + " boids")

        self.simulation.transferBoid(boid, toID, self.boidCPUID)

        # Remove the transfered boid from the current BoidCPUs boid list
        self.boids[:] = [b for b in self.boids if b.boidID != boid.boidID]
        self.boidCount -= 1


    ################################################################################################
    ## Load Balancing Functions ------------------------------------------------------------------##
    ################################################################################################

    # Creates a distribution based on the position of the boids in the current BoidCPU. The 
    # distribution splits the BoidCPU area in to a grid where the size of one grid square is given 
    # by the step size. Requires that the BoidCPUs only resize using a multiple of step size.
    #
    # FIXME: Works in general, but seems to be slightly off due to draw/udpdate offset
    # FIXME: Come up with a better algorithm, this one is horrendous
    def createBoidDistribution(self):
        boidCPUWidth = self.boidCPUCoords[2] - self.boidCPUCoords[0]
        boidCPUHeight = self.boidCPUCoords[3] - self.boidCPUCoords[1]

        widthSegments = boidCPUWidth / self.config['stepSize']
        heightSegments = boidCPUHeight / self.config['stepSize']

        # Draw a grid on for the BoidCPU
        # self.boidGPU.drawBoidCPUGrid(self.boidCPUCoords, widthSegments, heightSegments)

        # Create a multidimensional array of the based on the number of segments
        self.distribution = np.zeros((heightSegments, widthSegments))
        counter = 0

        # For every boid in the BoidCPU
        for boid in self.boids:
            boidNotPlaced = True
            # For each step size segment in width
            for wStep in range(widthSegments):
                # If the boid is in that width segment
                wStepValue = (((wStep + 1) * self.config['stepSize']) + self.boidCPUCoords[0])
                if boidNotPlaced and boid.position[0] < wStepValue:
                    # For each step size segment in height
                    for hStep in range(heightSegments):
                        # If the boid is in that height segment
                        hStepValue = (((hStep + 1) * self.config['stepSize']) + self.boidCPUCoords[1])
                        if boidNotPlaced and boid.position[1] < hStepValue:
                            # Add it to the distribution array
                            self.distribution[hStep, wStep] += 1
                            boidNotPlaced = False
                            counter += 1

        # Remove the BoidGPU grid
        # self.boidGPU.removeObject("gridLines")


    # Analyses the distribution of boids for the current BoidCPU to determine which edges should be 
    # modified, and by how much, to reduce the number of boids in the overloaded BoidCPU.
    #
    # While the number of boids released is less than a certain amount, the effect of moving each 
    # of the edges by one step size is analysed. The edge that releases the most boids is selected 
    # and the process repeats. On the repeat, the edges that were no chosen are not recalculated 
    # and the edge that was is analysed for its second step. Again, the edge that releases the most 
    # boids is chosen.
    # 
    # TODO: If there is no single maximum, choose the edge that has had the least step changes
    # TODO: Put a limit on the number of step sizes to ensure the BoidCPU doesn't collapse
    def analyseBoidDistribution(self):
        requestedStepChanges = [0, 0, 0, 0]     # The number of step changes per border
        boidsReleasedPerEdge = [0, 0, 0, 0]     # The number of boids released per edge
        recalculate = [True, True, True, True]  # Used to save recalculation if not needed
        
        edgeNames = ["TOP", "RIGHT", "BOTTOM", "LEFT"]

        distHeight = self.distribution.shape[0]
        distWidth = self.distribution.shape[1]

        numberOfBoidsReleased = 0

        while numberOfBoidsReleased <= 5:
            if self.validEdge(0) and recalculate[0]:    # Top edge
                bound = requestedStepChanges[0]
                boidsReleasedPerEdge[0] = np.sum(self.distribution[bound, :])
            
            if self.validEdge(1) and recalculate[1]:    # Right edge
                bound = requestedStepChanges[1]
                boidsReleasedPerEdge[1] = np.sum(self.distribution[:, distWidth - 1 - bound])
            
            if self.validEdge(2) and recalculate[2]:    # Bottom edge
                bound = requestedStepChanges[2]
                boidsReleasedPerEdge[2] = np.sum(self.distribution[distHeight - 1 - bound, :])
            
            if self.validEdge(3) and recalculate[3]:    # Left edge
                bound = requestedStepChanges[3]
                boidsReleasedPerEdge[3] = np.sum(self.distribution[:, bound])

            # Check for all zero values
            if np.count_nonzero(boidsReleasedPerEdge):
                # Get the index of the edge that releases the most boids
                maxIndex = boidsReleasedPerEdge.index(max(boidsReleasedPerEdge))

                # Increment the step change counter for that edge
                requestedStepChanges[maxIndex] += 1

                # Increment the total number of boids release
                numberOfBoidsReleased += boidsReleasedPerEdge[maxIndex]

                # Do not recalculate the non-max values
                for i, v in enumerate(recalculate):
                    if i != maxIndex:
                        recalculate[i] = False 
                    else:
                        recalculate[i] = True

            # If boidsReleasedPerEdge is all zero, forcably increment the step count of all of the 
            # valid zero edges by 1 and raise the recalculation barrier
            else:
                if self.validEdge(0):                   # Top
                    requestedStepChanges[0] += 1
                    recalculate[0] = True
                if self.validEdge(1):                   # Right
                    requestedStepChanges[1] += 1
                    recalculate[0] = True
                if self.validEdge(2):                   # Bottom
                    requestedStepChanges[2] += 1
                    recalculate[0] = True
                if self.validEdge(3):                   # Left
                    requestedStepChanges[3] += 1
                    recalculate[0] = True

        return requestedStepChanges


    # Determines if the edge represented by the given edgeID is valid for the current BoidCPU. For 
    # example, a BoidCPU on the top of the simulation area would not have its top edge as valid. 
    def validEdge(self, edgeID):
        [row, col] = self.gridPosition

        # Top left position
        if col == 0 and row == 0:
            validEdges = [False, True, True, False]

        # Bottom right position
        elif (col == self.config['widthInBoidCPUs'] - 1) and (row == self.config['widthInBoidCPUs'] - 1):
            validEdges = [True, False, False, True]
        
        # Bottom left position
        elif col == 0 and (row == self.config['widthInBoidCPUs'] - 1):
            validEdges = [True, True, False, False]
        
        # Top right position
        elif (col == self.config['widthInBoidCPUs'] - 1) and row == 0:
            validEdges = [False, False, True, True]

         # Left column
        elif col == 0:
            validEdges = [True, True, True, False]

        # Top row
        elif row == 0:
            validEdges = [False, True, True, True]

        # Right column
        elif (col == self.config['widthInBoidCPUs'] - 1):
            validEdges = [True, False, True, True]

        # Bottom row
        elif (row == self.config['widthInBoidCPUs'] - 1):
            validEdges = [True, True, False, True]
        
        # Middle
        else:
            validEdges = [True, True, True, True]

        return validEdges[edgeID]


    # Evaluates the requested boundary adjustment to determine what the state of the current BoidCPU 
    # would be if the change is made. Returns either the predicted numer of boids.
    # 
    # Gets the boids of neighbouring BoidCPUs that would be affected by the change, uses this, and 
    # its own boids to calculate the number of boids the BoidCPU will have if the requested edge 
    # changes are implemented by iterating over the boids and checking their position.
    # 
    # An alternative would be to use a distribution, but this has to be created and maintained. 
    # 
    # FIXME: The reported numbers are not always correct, possibly due to a draw happening 
    def evaluateBoundaryChange(self, changeRequests, gridPosition):
        [row, col] = gridPosition

        # Print out request
        for i in changeRequests:
            if i[0] == 0:
                t = "top"
            if i[0] == 1:
                t = "right"
            if i[0] == 2:
                t = "bottom"
            if i[0] == 3:
                t = "left"

            self.logger.debug("Request to change " + t + " of BoidCPU " + str(self.boidCPUID) + 
                " by " + str(i[1]) + " step")

        # Make a list of the edges that are requested to change
        edgeChanges = [False, False, False, False]
        for edge in changeRequests:
            edgeChanges[edge[0]] = True

        # Get a list of the neighbouring BoidCPUs that would be affected by the edge changes
        neighbourIDs = set()
        if edgeChanges[0] and edgeChanges[3]:                   # If top and left edges change
            neighbourIDs.add(self.neighbouringBoidCPUs[0])      # Get BoidCPU to the northwest
        if edgeChanges[0] and edgeChanges[1]:                   # If top and right edges change
            neighbourIDs.add(self.neighbouringBoidCPUs[2])      # Get BoidCPU to the northeast
        if edgeChanges[2] and edgeChanges[1]:                   # If bottom and right edges change
            neighbourIDs.add(self.neighbouringBoidCPUs[4])      # Get BoidCPU to the southeast
        if edgeChanges[2] and edgeChanges[3]:                   # If bottom and left edges change
            neighbourIDs.add(self.neighbouringBoidCPUs[6])      # Get BoidCPU to the southwest

        if edgeChanges[0]:                                      # If top edge changes
            neighbourIDs.add(self.neighbouringBoidCPUs[1])      # Get BoidCPU to the north
        if edgeChanges[1]:                                      # If right edge changes
            neighbourIDs.add(self.neighbouringBoidCPUs[3])      # Get BoidCPU to the east
        if edgeChanges[2]:                                      # If bottom edge changes
            neighbourIDs.add(self.neighbouringBoidCPUs[5])      # Get BoidCPU to the south
        if edgeChanges[3]:                                      # If left edge changes
            neighbourIDs.add(self.neighbouringBoidCPUs[7])      # Get BoidCPU to the west

        # Get the boids of the neighbouring BoidCPUs that would be affected by the change
        boidsToCheck = np.copy(self.boids)
        for neighbourID in neighbourIDs:
            extraBoids = self.simulation.getBoidCPUBoids(neighbourID)
            boidsToCheck = np.concatenate((boidsToCheck, extraBoids))
            
        # Determine the new bounds, but not by changing the actual bounds
        tempBounds = self.boidCPUCoords
        for edge in changeRequests:
            stepChange = edge[1] * self.config['stepSize']
            if row == self.gridPosition[0]:     # If this is on the same row as the overloaded one
                if edge[0] == 0:                #   If the top edge is to be changed
                    tempBounds[1] += stepChange #     Increase the top boundary
                elif edge[0] == 2:              #   If the bottom edge is to be changed
                    tempBounds[3] -= stepChange #     Decrease the bottom boundary
            else:
                if edge[0] == 0:                # Decrease top edge
                    tempBounds[1] -= stepChange
                elif edge[0] == 2:              # Increase bottom edge
                    tempBounds[3] += stepChange

            if col == self.gridPosition[1]:
                if edge[0] == 1:                # Decrease right edge
                    tempBounds[2] -= stepChange
                elif edge[0] == 3:              # Increase left edge
                    tempBounds[0] += stepChange
            else:
                if edge[0] == 1:                # Increase right edge
                    tempBounds[2] += stepChange
                elif edge[0] == 3:              # Decrease left edge
                    tempBounds[0] -= stepChange

        # Iterate over all the boids in the current BoidCPU and those gather from affected 
        # neighbouring BoidCPUs to determine how many boids lie within the new bounds
        counter = 0
        for boid in boidsToCheck:
            # If the boid lies within the new x bounds of the BoidCPU
            if (boid.position[0] >= tempBounds[0]) and (boid.position[0] <= tempBounds[2]):
                # And if the boid lies within the new y bounds of the BoidCPU
                if (boid.position[1] >= tempBounds[1]) and (boid.position[1] <= tempBounds[3]):
                    # Increment counter
                    counter += 1

        self.logger.debug("BoidCPU " + str(self.boidCPUID) + " would have " + str(counter) + 
            " boids (" + str(self.boidCount) + " before)")

        return [len(self.boids), counter]


    # Changes the bounds of the BoidCPU by the specifed step size. Used during load balancing.
    # 
    # If the BoidCPU is in the same row or column as the BoidCPU that is overloaded, then the 
    # BoidCPU boundaries need changing in the opporsite way than if the BoidCPU was in a different 
    # row to the BoidCPU that is overloaded
    def changeBounds(self, edgeType, steps, overloadedBoidCPUPosition):
        [row, col] = overloadedBoidCPUPosition
        stepChange = steps * self.config['stepSize']

        if row == self.gridPosition[0]:
            if edgeType == 0:         # Top
                self.boidCPUCoords[1] += stepChange
                self.logger.debug("Increased the top edge of BoidCPU " + str(self.boidCPUID) + 
                    " by " + str(stepChange))

            elif edgeType == 2:       # Bottom
                self.boidCPUCoords[3] -= stepChange
                self.logger.debug("Decreased the bottom edge of BoidCPU " + str(self.boidCPUID) + 
                    " by " + str(stepChange))
        else:
            if edgeType == 0:         # Top
                self.boidCPUCoords[1] -= stepChange
                self.logger.debug("Decreased the top edge of BoidCPU " + str(self.boidCPUID) + 
                    " by " + str(stepChange))

            elif edgeType == 2:       # Bottom
                self.boidCPUCoords[3] += stepChange
                self.logger.debug("Increased the bottom edge of BoidCPU " + str(self.boidCPUID) + 
                    " by " + str(stepChange))

        if col == self.gridPosition[1]:
            if edgeType == 1:         # Right
                self.boidCPUCoords[2] -= stepChange
                self.logger.debug("Decreased the right edge of BoidCPU " + str(self.boidCPUID) + 
                    " by " + str(stepChange))

            elif edgeType == 3:       # Left
                self.boidCPUCoords[0] += stepChange
                self.logger.debug("Increased the left edge of BoidCPU " + str(self.boidCPUID) + 
                    " by " + str(stepChange))

        else:
            if edgeType == 1:         # Right
                self.boidCPUCoords[2] += stepChange
                self.logger.debug("Increased the right edge of BoidCPU " + str(self.boidCPUID) + 
                    " by " + str(stepChange))

            elif edgeType == 3:       # Left
                self.boidCPUCoords[0] -= stepChange
                self.logger.debug("Decreased the left edge of BoidCPU " + str(self.boidCPUID) + 
                    " by " + str(stepChange))

        self.boidGPU.updateBoidCPU(self.boidCPUID, self.boidCPUCoords)


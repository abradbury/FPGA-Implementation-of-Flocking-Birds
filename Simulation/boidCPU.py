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

    def __init__(self, boidGPU, _simulation, _boidCPUID, _boidCPUCoords, _initialBoidCount, _colour, _gridPosition):
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

        # If the boidCPU bounds are to be coloured differently, use the given colour 
        if self.config['colourCode']:
            self.colour = _colour
        else:  
            self.colour = "yellow"

        # # Draw the boidCPU bounds
        self.boidGPU.drawBoidCPU(self.boidCPUCoords, self.boidCPUID, self.colour)

        # Draw the boidCPU's boids
        for i in range (0, self.boidCount):
            # Randomly position the boid on initialisation
            self.randomX = random.randint(self.boidCPUCoords[0], self.boidCPUCoords[2]);
            self.randomY = random.randint(self.boidCPUCoords[1], self.boidCPUCoords[3]);
            self.initialPosition = np.array([self.randomX, self.randomY], dtype = np.float_)
            self.boidID = ((self.boidCPUID - 1) * self.boidCount) + i + 1

            boid = Boid(self.boidGPU, self, self.boidID, self.initialPosition, self.colour)
            self.boids.append(boid)

        self.logger.info("Created boidCPU " + str(self.boidCPUID) + " with " + 
            str(self.boidCount) + " boids") 


    # Calculate the next positions of every boid in the boidCPU. Then determine if any of the boids 
    # need to be transferred to neighbouring boidCPUs based on their new positions. Finally, draw 
    # the new positions of the boids.
    def update(self, draw):
        if draw == False:
            self.possibleNeighbouringBoids = self.getPossibleNeighbouringBoids()

            # For each boid, calculate its new position
            for i in range(0, self.boidCount):
                self.boids[i].update(self.possibleNeighbouringBoids)

            # If the number of boids in the boidCPU are greater than a threshold, signal controller
            if self.config['loadBalance']:
                if self.boidCount >= self.config['BOID_THRESHOLD']:

                    # Analyse the distribution of the boids in this BoidCPU to determine the step
                    if self.config['loadBalanceType'] == 2: 
                        self.createBoidDistribution()
                        requestedChange = self.analyseBoidDistribution()
                    elif self.config['loadBalanceType'] == 1: 
                        requestedChange = None

                    self.simulation.boidCPUOverloaded(self.boidCPUID, requestedChange)

                    # self.simulation.boidCPUOverloaded(self.boidCPUID)

            # Determine if the new positions of the boids lie outside the boidCPU's bounds
            for boid in self.boids:
                self.determineBoidTransfer(boid)
            
        else:
            for i in range(0, self.boidCount):
                self.boids[i].draw(self.colour)


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

                # print str(self.boidCPUID) + " Moving the " + edgeNames[maxIndex] + " edge releases " + str(boidsReleasedPerEdge[maxIndex]) + " boids (total: " + str(numberOfBoidsReleased) + ")"
                # print str(requestedStepChanges) + " " + str(boidsReleasedPerEdge) + " " + str(recalculate)

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

                # print "Forcing step count increment due to zero-filled array"

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


    # Analyses the distribution of the boids in the current BoidCPU to determine what size step 
    # change to request and which boundary to request the change on. Requires that the BoidCPUs 
    # only resize using a multiple of step size.
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
        # distribution = [[0 for i in range(widthSegments)] for i in range(heightSegments)]
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

        # print self.distribution

        # Remove the BoidGPU grid
        # self.boidGPU.removeBoidCPUGrid()


    # Evaluates the requested boundary adjustment to determine what the state of the current BoidCPU 
    # would be if the change is made. Returns either the predicted numer of boids or the status. 
    def evaluateRequestedBoundaryChange(self, edge, step):
        print


    # Changes the bounds of the boidCPU by the specifed step size. Used during load balancing.
    def changeBounds(self, edgeType, steps, overloadedBoidCPUPosition):
        [row, col] = overloadedBoidCPUPosition
        stepChange = steps * self.config['stepSize']

        # If the boidCPU is in the same row or column as the boidCPU that is overloaded, then the 
        # boidCPU boundaries need changing in the opporsite way than if the boidCPU was in a 
        # different row to the boidCPU that is overloaded

        if row == self.gridPosition[0]:
            if edgeType == 0:         # Top
                self.boidCPUCoords[1] += stepChange
                # print "Increased the top edge of boidCPU " + str(self.boidCPUID) + " by " + str(stepChange)

            elif edgeType == 2:       # Bottom
                self.boidCPUCoords[3] -= stepChange
                # print "Decreased the bottom edge of boidCPU " + str(self.boidCPUID) + " by " + str(stepChange)
        else:
            if edgeType == 0:         # Top
                self.boidCPUCoords[1] -= stepChange
                # print "Decreased the top edge of boidCPU " + str(self.boidCPUID) + " by " + str(stepChange)

            elif edgeType == 2:       # Bottom
                self.boidCPUCoords[3] += stepChange
                # print "Increased the bottom edge of boidCPU " + str(self.boidCPUID) + " by " + str(stepChange)

        if col == self.gridPosition[1]:
            if edgeType == 1:         # Right
                self.boidCPUCoords[2] -= stepChange
                # print "Decreased the right edge of boidCPU " + str(self.boidCPUID) + " by " + str(stepChange)

            elif edgeType == 3:       # Left
                self.boidCPUCoords[0] += stepChange
                # print "Increased the left edge of boidCPU " + str(self.boidCPUID) + " by " + str(stepChange)

        else:
            if edgeType == 1:         # Right
                self.boidCPUCoords[2] += stepChange
                # print "Increased the right edge of boidCPU " + str(self.boidCPUID) + " by " + str(stepChange)

            elif edgeType == 3:       # Left
                self.boidCPUCoords[0] -= stepChange
                # print "Decreased the left edge of boidCPU " + str(self.boidCPUID) + " by " + str(stepChange)    

        self.boidGPU.updateBoidCPU(self.boidCPUID, self.boidCPUCoords)


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
        self.boids.append(boid)
        self.boidCount += 1

        self.logger.debug("BoidCPU " + str(self.boidCPUID) + " accepted boid " + str(boid.boidID) + 
            " from boidCPU " + str(fromID) + " and now has " + str(self.boidCount) + " boids")


    # Transfer a boid from this boidCPU to another boidCPU
    def transferBoid(self, boid, toID):
        self.logger.debug("BoidCPU " + str(self.boidCPUID) + " sent boid " + str(boid.boidID) + 
            " to boidCPU " + str(toID) + " and now has " + str(self.boidCount - 1) + " boids")

        self.simulation.transferBoid(boid, toID, self.boidCPUID)
        self.boids[:] = [b for b in self.boids if b.boidID != boid.boidID]
        self.boidCount -= 1


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



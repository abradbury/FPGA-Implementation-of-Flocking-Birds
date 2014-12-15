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

        # Define boid structures
        self.boids = []
        self.initialBoidCount = _initialBoidCount
        self.boidCount = self.initialBoidCount

        # Define debugging flags
        self.colourCode = self.simulation.colourCode
        self.trackBoid = self.simulation.trackBoid

        # Used to hold the plotting data
        self.xData = []
        self.yData = []
        self.y2Data = []

        # If the boidCPU bounds are to be coloured differently, use the given colour 
        if self.colourCode:
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
            if self.boidCount >= self.simulation.BOID_THRESHOLD:
                self.simulation.boidCPUOverloaded(self.boidCPUID)

            # Determine if the new positions of the boids lie outside the boidCPU's bounds
            for boid in self.boids:
                self.determineBoidTransfer(boid)
            
        else:
            for i in range(0, self.boidCount):
                self.boids[i].draw(self.colour)


    # Changes the bounds of the boidCPU by the specifed step size. Used during load balancing.
    def changeBounds(self, edgeType, stepSize, overloadedBoidCPUPosition):
        [row, col] = overloadedBoidCPUPosition

        # If the boidCPU is in the same row or column as the boidCPU that is overloaded, then the 
        # boidCPU boundaries need changing in the opporsite way than if the boidCPU was in a 
        # different row to the boidCPU that is overloaded

        if row == self.gridPosition[0]:
            if edgeType == "top":
                self.boidCPUCoords[1] += stepSize
                # print "Increased the top edge of boidCPU " + str(self.boidCPUID) + " by " + str(stepSize)

            elif edgeType == "bottom":
                self.boidCPUCoords[3] -= stepSize
                # print "Decreased the bottom edge of boidCPU " + str(self.boidCPUID) + " by " + str(stepSize)
        else:
            if edgeType == "top":
                self.boidCPUCoords[1] -= stepSize
                # print "Decreased the top edge of boidCPU " + str(self.boidCPUID) + " by " + str(stepSize)

            elif edgeType == "bottom":
                self.boidCPUCoords[3] += stepSize
                # print "Increased the bottom edge of boidCPU " + str(self.boidCPUID) + " by " + str(stepSize)

        if col == self.gridPosition[1]:
            if edgeType == "right":
                self.boidCPUCoords[2] -= stepSize
                # print "Decreased the right edge of boidCPU " + str(self.boidCPUID) + " by " + str(stepSize)

            elif edgeType == "left":
                self.boidCPUCoords[0] += stepSize
                # print "Increased the left edge of boidCPU " + str(self.boidCPUID) + " by " + str(stepSize)

        else:
            if edgeType == "right":
                self.boidCPUCoords[2] += stepSize
                # print "Increased the right edge of boidCPU " + str(self.boidCPUID) + " by " + str(stepSize)

            elif edgeType == "left":
                self.boidCPUCoords[0] -= stepSize
                # print "Decreased the left edge of boidCPU " + str(self.boidCPUID) + " by " + str(stepSize)    

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



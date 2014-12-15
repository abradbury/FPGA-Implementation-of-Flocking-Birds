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

    def __init__(self, boidGPU, _simulation, _locationID, _locationCoords, _initialBoidCount, _colour, _gridPosition):
        self.simulation = _simulation
        self.locationID = _locationID
        self.locationCoords = np.copy(_locationCoords)
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

        # If the location bounds are to be coloured differently, use the given colour 
        if self.colourCode:
            self.colour = _colour
        else:  
            self.colour = "yellow"

        # # Draw the location bounds
        self.boidGPU.drawLocation(self.locationCoords, self.locationID, self.colour)

        # Draw the location's boids
        for i in range (0, self.boidCount):
            # Randomly position the boid on initialisation
            self.randomX = random.randint(self.locationCoords[0], self.locationCoords[2]);
            self.randomY = random.randint(self.locationCoords[1], self.locationCoords[3]);
            self.initialPosition = np.array([self.randomX, self.randomY], dtype = np.float_)
            self.boidID = ((self.locationID - 1) * self.boidCount) + i + 1

            boid = Boid(self.boidGPU, self, self.boidID, self.initialPosition, self.colour)
            self.boids.append(boid)

        self.logger.info("Created location " + str(self.locationID) + " with " + 
            str(self.boidCount) + " boids") 


    # Calculate the next positions of every boid in the location. Then determine if any of the boids 
    # need to be transferred to neighbouring locations based on their new positions. Finally, draw 
    # the new positions of the boids.
    def update(self, draw):
        if draw == False:
            self.possibleNeighbouringBoids = self.getPossibleNeighbouringBoids()

            # For each boid, calculate its new position
            for i in range(0, self.boidCount):
                self.boids[i].update(self.possibleNeighbouringBoids)

            # If the number of boids in the location are greater than a threshold, signal controller
            if self.boidCount >= self.simulation.BOID_THRESHOLD:
                self.simulation.locationOverloaded(self.locationID)

            # Determine if the new positions of the boids lie outside the location's bounds
            for boid in self.boids:
                self.determineBoidTransfer(boid)
            
        else:
            for i in range(0, self.boidCount):
                self.boids[i].draw(self.colour)


    # Changes the bounds of the location by the specifed step size. Used during load balancing.
    def changeBounds(self, edgeType, stepSize, overloadedLocationPosition):
        [row, col] = overloadedLocationPosition

        # If the location is in the same row or column as the location that is overloaded, then the 
        # location boundaries need changing in the opporsite way than if the location was in a 
        # different row to the location that is overloaded

        if row == self.gridPosition[0]:
            if edgeType == "top":
                self.locationCoords[1] += stepSize
                # print "Increased the top edge of location " + str(self.locationID) + " by " + str(stepSize)

            elif edgeType == "bottom":
                self.locationCoords[3] -= stepSize
                # print "Decreased the bottom edge of location " + str(self.locationID) + " by " + str(stepSize)
        else:
            if edgeType == "top":
                self.locationCoords[1] -= stepSize
                # print "Decreased the top edge of location " + str(self.locationID) + " by " + str(stepSize)

            elif edgeType == "bottom":
                self.locationCoords[3] += stepSize
                # print "Increased the bottom edge of location " + str(self.locationID) + " by " + str(stepSize)

        if col == self.gridPosition[1]:
            if edgeType == "right":
                self.locationCoords[2] -= stepSize
                # print "Decreased the right edge of location " + str(self.locationID) + " by " + str(stepSize)

            elif edgeType == "left":
                self.locationCoords[0] += stepSize
                # print "Increased the left edge of location " + str(self.locationID) + " by " + str(stepSize)

        else:
            if edgeType == "right":
                self.locationCoords[2] += stepSize
                # print "Increased the right edge of location " + str(self.locationID) + " by " + str(stepSize)

            elif edgeType == "left":
                self.locationCoords[0] -= stepSize
                # print "Decreased the left edge of location " + str(self.locationID) + " by " + str(stepSize)    

        self.boidGPU.updateLocation(self.locationID, self.locationCoords)


    def determineBoidTransfer(self, boid):
        # If the location has a neighbour to the NORTHWEST and the boid is beyond its northern AND western boundaries
        if (self.neighbouringLocations[0] != 0) and (boid.position[1] < self.locationCoords[1]) and (boid.position[0] < self.locationCoords[0]):
            self.logger.debug("Boid " + str(boid.boidID) + 
                " is beyond the NORTH and WESTERN boundaries of location " + str(self.locationID))
            self.transferBoid(boid, self.neighbouringLocations[0])

        # If the location has a neighbour to the NORTHEAST and the boid is beyond its northern AND eastern boundaries
        elif (self.neighbouringLocations[2] != 0) and (boid.position[1] < self.locationCoords[1]) and (boid.position[0] > self.locationCoords[2]):
            self.logger.debug("Boid " + str(boid.boidID) + 
                " is beyond the NORTH and EASTERN boundaries of location " + str(self.locationID))
            self.transferBoid(boid, self.neighbouringLocations[2])

        # If the location has a neighbour to the SOUTHEAST and the boid is beyond its southern AND eastern boundaries
        elif (self.neighbouringLocations[4] != 0) and (boid.position[1] > self.locationCoords[3]) and (boid.position[0] > self.locationCoords[2]):
            self.logger.debug("Boid " + str(boid.boidID) + 
                " is beyond the SOUTHERN and EASTERN boundaries of location " + str(self.locationID))
            self.transferBoid(boid, self.neighbouringLocations[4])

        # If the location has a neighbour to the SOUTHWEST and the boid is beyond its southern AND western boundaries
        elif (self.neighbouringLocations[6] != 0) and (boid.position[1] > self.locationCoords[3]) and (boid.position[0] < self.locationCoords[0]):
            self.logger.debug("Boid " + str(boid.boidID) + 
                " is beyond the SOUTHERN and WESTERN boundaries of location " + str(self.locationID))
            self.transferBoid(boid, self.neighbouringLocations[6])

        # If the location has a neighbour to the NORTH and the boid is beyond its northern boundary
        elif (self.neighbouringLocations[1] != 0) and (boid.position[1] < self.locationCoords[1]):
            self.logger.debug("Boid " + str(boid.boidID) + 
                " is beyond the NORTH boundary of location " + str(self.locationID))
            self.transferBoid(boid, self.neighbouringLocations[1])

        # If the location has a neighbour to the EAST and the boid is beyond its eastern boundary
        elif (self.neighbouringLocations[3] != 0) and (boid.position[0] > self.locationCoords[2]):
            self.logger.debug("Boid " + str(boid.boidID) + 
                " is beyond the EAST boundary of location " + str(self.locationID))
            self.transferBoid(boid, self.neighbouringLocations[3])  
            
        # If the location has a neighbour to the SOUTH and the boid is beyond its southern boundary
        elif (self.neighbouringLocations[5] != 0) and (boid.position[1] > self.locationCoords[3]):
            self.logger.debug("Boid " + str(boid.boidID) + 
                " is beyond the SOUTH boundary of location " + str(self.locationID))
            self.transferBoid(boid, self.neighbouringLocations[5])

        # If the location has a neighbour to the WEST and the boid is beyond its western boundary
        elif (self.neighbouringLocations[7] != 0) and (boid.position[0] < self.locationCoords[0]):
            self.logger.debug("Boid " + str(boid.boidID) + 
                " is beyond the WEST boundary of location " + str(self.locationID))
            self.transferBoid(boid, self.neighbouringLocations[7])


    # Accept a boid transferred from another location and add it to this locations boid list
    def acceptBoid(self, boid, fromID):
        self.boids.append(boid)
        self.boidCount += 1

        self.logger.debug("Location " + str(self.locationID) + " accepted boid " + str(boid.boidID) + 
            " from location " + str(fromID) + " and now has " + str(self.boidCount) + " boids")


    # Transfer a boid from this location to another location
    def transferBoid(self, boid, toID):
        self.logger.debug("Location " + str(self.locationID) + " sent boid " + str(boid.boidID) + 
            " to location " + str(toID) + " and now has " + str(self.boidCount - 1) + " boids")

        self.simulation.transferBoid(boid, toID, self.locationID)
        self.boids[:] = [b for b in self.boids if b.boidID != boid.boidID]
        self.boidCount -= 1


    # Return a list containing the boids currently controlled by this location
    def getBoids(self):
        return self.boids


    # Return a list containing the boids from each neighbouring location
    def getPossibleNeighbouringBoids(self):
        self.neighbouringLocations = self.simulation.getNeighbouringLocations(self.locationID)

        # Need the slice operation or else updating 
        self.neighbouringBoids = self.boids[:]

        for locationIndex in self.neighbouringLocations:
            if locationIndex != 0:
                self.neighbouringBoids += self.simulation.getLocationBoids(locationIndex)

        return self.neighbouringBoids



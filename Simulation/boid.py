#!/usr/bin/python
# -*- coding: utf-8 -*-


import numpy as np                  # Used in various mathematical operations
import logging                      # Used to handle the textual output
import math                         # Used to calculate the square root for normalisation
import copy                         # Used to copy lists to avoid passing by reference


# A class representing an individual boid. Based on the model presented by Craig Reynolds in 1986 
# (http://www.red3d.com/cwr/boids/) and 'The Nature of Code' by Daniel Shiffman.
class Boid:

    def __init__(self, _boidGPU, _boidCPU, _BOID_ID, initPosition, initVelocity, _colour, copy):
        self.boidCPU = _boidCPU
        self.boidGPU = _boidGPU

        self.config = self.boidCPU.config
        self.logger = self.boidCPU.logger

        # True if the boid has had its next position calculated, false otherwise
        self.processed = False

        # Define the initial movement parameters
        self.position = initPosition[:]
        self.velocity = initVelocity[:]
        self.acceleration = np.array([0, 0], dtype = np.float_)

        # Create the boid
        self.BOID_ID = _BOID_ID        
        self.neighbouringBoids = []

        # If the instance is a new boid and not a copy
        if not copy:
            self.logger.debug("Created boid with ID " + str(self.BOID_ID))

            # Draw the boid
            self.boidGPU.createBoid(self.position, self.velocity, _colour, self.BOID_ID)


    # Calculate the neighbouring boids based on the Euclidean distance between the current boid and 
    # the possible neighbour. Possible neighbouring boids include boids from neighbouring boidCPUs.
    def calculateNeighbours(self, possibleNeighbours):
        for boid in possibleNeighbours:
            if boid.BOID_ID != self.BOID_ID:
                dist = self.distanceBetweenTwoPoints(self.position, boid.position)
                if dist < self.config['VISION_RADIUS']:
                    self.neighbouringBoids.append(boid)


    ################################################################################################
    ## Boid Update Routines ----------------------------------------------------------------------##
    ################################################################################################

    # Update the boid's position based on the information contained in its neighbourhood. Only 
    # commit the changes when all the boids have calculated their next move. Otherwise, a boid 
    # would move based on its neighbours and when one of its neighbouts comes to move it will use 
    # the new position of the original boid, not its original position. 
    def update(self, possibleNeighbouringBoids):
        # If the boid has not already been processed, calculate its next position.
        # A boid would already be processed if it was transferred to another BoidCPU
        if not self.processed:
            # self.calculateNeighbours(possibleNeighbouringBoids)

            # If tracking a boid, highlight its neighbouring boids
            if self.config['trackBoid'] and (self.BOID_ID == self.config['boidToTrack']): 
                for boid in self.neighbouringBoids:
                    self.boidGPU.highlightBoid(True, boid.BOID_ID)

            # self.logger.debug("Updating boid #" + str(self.BOID_ID) + " (from BoidCPU #" + str(self.boidCPU.BOIDCPU_ID) + ")")
            # for boid in self.neighbouringBoids:
            #     self.logger.debug("  - Boid #" + str(self.BOID_ID) + " neighbour (Boid #" + str(boid.BOID_ID) + ") has position: " + str(boid.position))

            # If the boids has neighbours, calculate its next position 
            if len(self.neighbouringBoids) > 0:
                self.applyBehaviours()

            # Calculate the new position of the boid
            self.calcNewPos()

            # Contain the boids in the simulation area
            self.containBoids()

            self.processed = True


    def calcNewPos(self):
        # Update and limit the velocity
        self.velocity += self.acceleration
        self.velocity = self.limit(self.velocity, self.config['MAX_VELOCITY'])

        # Update the position
        self.position += self.velocity

        # Clear acceleration
        self.acceleration *= 0


    # Used to create a copy of the current boid. This is needed to get around Python's 
    # pass-by-reference nature and because deepcopy is unable to copy the boids.
    def copy(self):
        copyOfPosition = np.copy(self.position)
        copyOfVelocity = np.copy(self.velocity)

        copyOfBoid = Boid(self.boidGPU, self.boidCPU, self.BOID_ID, copyOfPosition, copyOfVelocity, None, True)
        
        return copyOfBoid


    # Mass could optionally be added here to get different behaviours
    def applyForce(self, force):
        self.acceleration += force


    def applyBehaviours(self):
        sep = self.separate()
        ali = self.align()
        coh = self.cohesion()

        sep *= self.config['REPULSION_WEIGHT']
        ali *= self.config['ALIGNMENT_WEIGHT']
        coh *= self.config['COHESION_WEIGHT']

        self.applyForce(sep)
        self.applyForce(ali)
        self.applyForce(coh)


    # Move the boid to the calculated positon
    def draw(self, colour):
        self.boidGPU.updateBoid(self.position, self.velocity, colour, self.BOID_ID)

        self.processed = False  # Reset the processed flag for the next simulation step


    ################################################################################################
    ## Boid Steering Rules -----------------------------------------------------------------------##
    ################################################################################################

    # A boid will align itself with the average orientation of its neighbours
    def align(self):
        total = np.array([0, 0], dtype = np.float_)

        for boid in self.neighbouringBoids:
            total += boid.velocity

        total /= len(self.neighbouringBoids)
        total = self.setMag(total, self.config['MAX_VELOCITY'])

        steer = total - self.velocity
        steer = self.limit(steer, self.config['MAX_FORCE'])

        return steer


    # A boid does not want to get too close to its neighbours and needs its personal space
    #
    # TODO: Scale it depending on how close the boid is
    def separate(self):
        total = np.array([0, 0], dtype = np.float_)

        for boid in self.neighbouringBoids:
            diff = self.position - boid.position        #  TODO: Check subtraction is right
            diff = self.normalise(diff)
            total += diff

        total /= len(self.neighbouringBoids)
        total = self.setMag(total, self.config['MAX_VELOCITY'])

        steer = total - self.velocity
        steer = self.limit(steer, self.config['MAX_FORCE'])

        return steer

    # A boid will move towards the centre of mass of its neighbourhood
    def cohesion(self):
        total = 0

        for boid in self.neighbouringBoids:
            total += boid.position

        total /= len(self.neighbouringBoids)
        steer = self.seek(total)

        return steer


    ################################################################################################
    ## Other Boid Movement Rules -----------------------------------------------------------------##
    ################################################################################################

    # Calculates the desired velocity to the target at the maximum allowed speed. Uses this to 
    # calculate a steering force to the target which is applied to the boid.
    def seek(self, target):
        self.desired = target - self.position
        self.desired = self.setMag(self.desired, self.config['MAX_VELOCITY'])

        # Calculate the steering force
        self.steer = self.desired - self.velocity

        # Limit the size of the steering force
        self.steer = self.limit(self.steer, self.config['MAX_FORCE'])

        return self.steer


    # Contain the boids within the simulation area
    def containBoids(self):
        # Simple wrap around
        # NOTE: The neighbouring BoidCPUs needs updating to account for this
        if self.position[0] > self.config['width']:
            self.position[0] = 0
        elif self.position[0] < 0:
            self.position[0] = self.config['width']

        if self.position[1] > self.config['width']:
            self.position[1] = 0
        elif self.position[1] < 0:
            self.position[1] = self.config['width']

        # Simple repel
        # if self.position[0] > self.config['width']:
        #     self.position[0] = self.config['width']
        #     self.velocity[0] *= -1;
        # elif self.position[0] < 0:
        #     self.velocity[0] *= -1;
        #     self.position[0] = 0;

        # if self.position[1] > self.config['width']:
        #     self.position[1] = self.config['width']
        #     self.velocity[1] *= -1;
        # elif self.position[1] < 0:
        #     self.position[1] = 0
        #     self.velocity[1] *= -1;
        
        # The closer it is the stronger the repulsion 
        # This works much better but when the boids approach head on, the change is quite sudden
        # What about, the smaller the gap between the boid x and ahead x, the greater the x push?
        # repulsionFactor = 3000
        # if self.position[1] < 0 + self.config['VISION_RADIUS']:
        #     d = self.distanceFromPointToLine([0, 0], [self.config['width'], 0], self.position)
        #     repulsionForce = repulsionFactor / (d ** 2)
        #     self.velocity[1] += repulsionForce
        # # Bottom edge
        # elif self.position[1] > self.config['width'] - self.config['VISION_RADIUS']:
        #     d = self.distanceFromPointToLine([0, self.config['width']], [self.config['width'], self.config['width']], self.position)
        #     repulsionForce = repulsionFactor / (d ** 2)
        #     self.velocity[1] -= repulsionForce
        # # Left edge
        # elif self.position[0] < 0 + self.config['VISION_RADIUS']:
        #     d = self.distanceFromPointToLine([0, 0], [0, self.config['width']], self.position)
        #     repulsionForce = repulsionFactor / (d ** 2)
        #     self.velocity[0] += repulsionForce
        # # Right edge
        # elif self.position[0] > self.config['width'] - self.config['VISION_RADIUS']:
        #     d = self.distanceFromPointToLine([self.config['width'], 0], [self.config['width'], self.config['width']], self.position)
        #     repulsionForce = repulsionFactor / (d ** 2)
        #     self.velocity[0] -= repulsionForce


    ################################################################################################
    ## Vector operations -------------------------------------------------------------------------##
    ################################################################################################

    # Used to normalise a 2D or 3D vector
    def normalise(self, vector):
        magnetude = self.absolute(vector)

        if magnetude:
            result = vector / magnetude
        else:
            result = 0

        return result 


    # Calculate the magnetude or absolute value of the given vector
    def absolute(self, vector):
        if len(vector) == 2:
            magnetude = math.sqrt((vector[0] ** 2) + (vector[1] ** 2))
        elif len(vector) == 3:
            magnetude = math.sqrt((vector[0] ** 2) + (vector[1] ** 2) + (vector[2] ** 2))

        return magnetude


    # Set the magnetude of a vector to the given value
    def setMag(self, vector, value):
        result = self.normalise(vector)
        result *= value
        return result


    # Limit a vector to the maximum allowed
    def limit(self, vector, maximum):
        if self.absolute(vector) > maximum:
            vector = self.normalise(vector) * maximum
        return vector


    def distanceBetweenTwoPoints(self, u, v):
        if len(u) == 2:
            result = math.sqrt(((u[0] - v[0]) ** 2) + ((u[1] - v[1]) ** 2))
        elif len(u) == 3:
            result = math.sqrt(((u[0] - v[0]) ** 2) + ((u[1] - v[1]) ** 2) + ((u[2] - v[2]) ** 2))
        return result


    # Calculates the shortest distance from a boid to a boundary line
    # From http://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line 
    def distanceFromPointToLine(self, lineStart, lineEnd, point):
        lineEnd[1] - lineStart[1]

        partOne = ((lineEnd[1] - lineStart[1]) * point[0]) - ((lineEnd[0] - lineStart[0]) * 
            point[1]) + (lineEnd[0] * lineStart[1]) - (lineEnd[1] * lineStart[0])

        partTwo = ((lineEnd[1] - lineStart[1]) ** 2) + ((lineEnd[0] - lineStart[0]) ** 2)

        result = abs(partOne) / np.sqrt(partTwo)

        return result


#!/usr/bin/python
# -*- coding: utf-8 -*-


import numpy as np                  # Used in various mathematical operations
import random                       # Used to randomly position the boids on initialisation
import logging                      # Used to handle the textual output
import warnings                     # Used to catch an invalid divide warning
import math                         # Used to calculate the square root for normalisation


# A class representing an individual boid
class Boid:

    def __init__(self, boidGPU, _boidCPU, _boidID, initPosition, _colour):
        self.boidCPU = _boidCPU
        self.logger = self.boidCPU.logger

        # Make the system configuration list available
        self.config = self.boidCPU.config

        # Colour the boids based on their original boidCPU if debugging
        if self.config['colourCode']:
            self.colour = _colour
            self.outlineColour = _colour
            self.boidWidth = 2
        else:
            self.colour = "red"
            self.outlineColour = "white"
            self.boidWidth = 1

        # Create the boids
        self.boidID = _boidID
        # self.bearing = np.pi
        # self.step = 10
        self.boidGPU = boidGPU
        self.position = initPosition

        self.neighbouringBoids = []

        self.randomVelX = random.randint(-self.config['MAX_VELOCITY'], self.config['MAX_VELOCITY'])
        self.randomVelY = random.randint(-self.config['MAX_VELOCITY'], self.config['MAX_VELOCITY'])
        self.velocity = np.array([self.randomVelX, self.randomVelY], dtype = np.float_)

        # Draw the boid
        self.boidGPU.createBoid(self.position, self.velocity, self.colour, self.outlineColour, self.boidWidth, self.boidID)

        self.logger.debug("Created boid with ID " + str(self.boidID))


    # Update the boid's position based on the information contained in its neighbourhood. Only 
    # commit the changes when all the boids have calculated their next move. Otherwise, a boid 
    # would move based on its neighbours and when one of its neighbouts comes to move it will use 
    # the new position of the original boid, not its original position. 
    def update(self, possibleNeighbouringBoids):
        self.possibleNeighbours = possibleNeighbouringBoids
        self.calculateNeighbours()

        # If tracking boid, highlight its neighbouring boids
        if self.config['trackBoid'] and (self.boidID == self.config['boidToTrack']): 
            for boid in self.neighbouringBoids:
                self.boidGPU.highlightBoid(True, boid.boidID)

        # If the boids has neighbours, calculate its next position 
        if len(self.neighbouringBoids) > 0:
            self.cohesionMod = self.coehsion()
            self.alignmentMod = self.alignment()
            self.repulsionMod = self.repulsion()

            self.velocity = (self.velocity + 
                (self.config['COHESION_WEIGHT'] * self.cohesionMod) + 
                (self.config['ALIGNMENT_WEIGHT'] * self.alignmentMod) + 
                (self.config['REPULSION_WEIGHT'] * self.repulsionMod))

        # Avoid obstacles
        self.avoidObstacles()

        # Bound the velocity to the maximum allowed
        self.limitVelocity()

        # Contain the boids in the simulation area
        self.containBoids()

        # Update the position of the boid
        self.position += self.velocity

        # Contain the boids in the simulation area
        self.containBoids()


    # Move the boid to the calculate positon
    def draw(self, colour):

        # Specify the boid fill colour based on the debug flag
        if self.config['colourCode']:
            self.fillColour = colour
        else:
            self.fillColour = "red"

        self.boidGPU.updateBoid(self.position, self.velocity, self.fillColour, self.boidID)


    # Calculate the neighbouring boids based on the Euclidean distance between the current boid and 
    # the possible neighbour. Possible neighbouring boids include boids from neighbouring boidCPUs.
    def calculateNeighbours(self):
        self.neighbouringBoids = []

        for boid in self.possibleNeighbours:
            if boid.boidID != self.boidID:
                dist = np.linalg.norm(boid.position - self.position)
                if dist < self.config['VISION_RADIUS']:
                    self.neighbouringBoids.append(boid)


    # A boid will move towards the centre of mass of its neighbourhood
    def coehsion(self):
        self.cohesionMod = 0;
        for boid in self.neighbouringBoids:
            self.cohesionMod += boid.position

        self.cohesionMod /= len(self.neighbouringBoids)
        self.cohesionMod -= self.position
        self.cohesionMod = self.normalise(self.cohesionMod)

        return self.cohesionMod


    # A boid will align itself with the average orientation of its neighbours
    def alignment(self):
        self.alignmentMod = 0
        for boid in self.neighbouringBoids:
            self.alignmentMod += boid.velocity

        self.alignmentMod /= len(self.neighbouringBoids)
        self.alignmentMod = self.normalise(self.alignmentMod)

        return self.alignmentMod


    # A boid does not want to get too close to its neighbours and needs its personal space
    def repulsion(self):
        self.repulsionMod = 0
        for boid in self.neighbouringBoids:
            self.repulsionMod += (boid.position - self.position)

        self.repulsionMod /= len(self.neighbouringBoids)
        self.repulsionMod *= -1
        self.repulsionMod = self.normalise(self.repulsionMod)

        return self.repulsionMod


    # Limit the boid's velocity to the maximum allowed
    def limitVelocity(self):
        # if self.velocity[0] > self.config['MAX_VELOCITY']:
        #     self.velocity[0] = self.config['MAX_VELOCITY']

        # if self.velocity[1] > self.config['MAX_VELOCITY']:
        #     self.velocity[1] = self.config['MAX_VELOCITY']

        # if self.velocity[0] < -self.config['MAX_VELOCITY']:
        #     self.velocity[0] = -self.config['MAX_VELOCITY']

        # if self.velocity[1] < -self.config['MAX_VELOCITY']:
        #     self.velocity[1] = -self.config['MAX_VELOCITY']

        # Results in slightly different behaviour, though not better
        if self.absolute(self.velocity) > self.config['MAX_VELOCITY']:
            self.velocity = self.normalise(self.velocity) * self.config['MAX_VELOCITY']


    # Contain the boids within the simulation area
    def containBoids(self):
        # Left
        if self.position[0] < 0:
            self.velocity[0] = -self.velocity[0]
            self.position += self.velocity

        # Right
        elif self.position[0] > self.config['width']:
            self.velocity[0] = -self.velocity[0]
            self.position += self.velocity

        # Top
        elif self.position[1] < 0:
            self.velocity[1] = -self.velocity[1]
            self.position += self.velocity

        # Bottom
        elif self.position[1] > self.config['width']:
            self.velocity[1] = -self.velocity[1]
            self.position += self.velocity

        # Alternative method
        # Doesn't strictly conatain then, but slightly changes velocity to move back
        # step = 5
        # if self.position[0] < 0:
        #     self.velocity[0] = step
        # elif self.position[0] > self.config['width']:
        #     self.velocity[0] = -step

        # if self.position[1] < 0:
        #     self.velocity[1] = step
        # elif self.position[1] > self.config['width']:
        #     self.velocity[1] = -step


    def avoidObstacles(self):
        # ahead = self.position + self.normalise(self.velocity) * self.config['VISION_RADIUS']
        # self.boidGPU.drawLine(self.position, ahead, ("AheadLine" + str(self.boidID)))

        # The closer it is the stronger the repulsion 
        # This works much better but when the boids approach head on, the change is quite sudden
        # What about, the smaller the gap between the boid x and ahead x, the greater the x push?
        repulsionFactor = 3000
        if self.position[1] < 0 + self.config['VISION_RADIUS']:
            d = self.distanceFromPointToLine([0, 0], [self.config['width'], 0], self.position)
            repulsionForce = repulsionFactor / (d ** 2)
            self.velocity[1] += repulsionForce
        # Bottom edge
        elif self.position[1] > self.config['width'] - self.config['VISION_RADIUS']:
            d = self.distanceFromPointToLine([0, self.config['width']], [self.config['width'], self.config['width']], self.position)
            repulsionForce = repulsionFactor / (d ** 2)
            self.velocity[1] -= repulsionForce
        # Left edge
        elif self.position[0] < 0 + self.config['VISION_RADIUS']:
            d = self.distanceFromPointToLine([0, 0], [0, self.config['width']], self.position)
            repulsionForce = repulsionFactor / (d ** 2)
            self.velocity[0] += repulsionForce
        # Right edge
        elif self.position[0] > self.config['width'] - self.config['VISION_RADIUS']:
            d = self.distanceFromPointToLine([self.config['width'], 0], [self.config['width'], self.config['width']], self.position)
            repulsionForce = repulsionFactor / (d ** 2)
            self.velocity[0] -= repulsionForce


    def circleLineIntercept(self, x, y, ahead):
        # print x
        # print y

        if (x >= 0) and not y:
            # y = b ± sqrt(r^2 - (x - a)^2)
            yPos = self.position[1] + math.sqrt((self.config['VISION_RADIUS'] ** 2) - ((x - self.position[0]) ** 2))
            yNeg = self.position[1] - math.sqrt((self.config['VISION_RADIUS'] ** 2) - ((x - self.position[0]) ** 2))
        
            if abs(yPos - ahead[1]) > abs(yNeg - ahead[1]):
                result = [x, yNeg]
            else:
                result = [x, yPos]

        elif (y >= 0) and not x:
            # x = a ± sqrt(r^2 - (y - b)^2)
            xPos = self.position[0] + math.sqrt((self.config['VISION_RADIUS'] ** 2) - ((y - self.position[1]) ** 2))
            xNeg = self.position[0] - math.sqrt((self.config['VISION_RADIUS'] ** 2) - ((y - self.position[1]) ** 2))

            if abs(xPos - ahead[0]) > abs(xNeg - ahead[0]):
                result = [xNeg, y]
            else:
                result = [xPos, y]

        return result


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


    ################################################################################################
    # The code below is not finalised - was an attempt to improve the boid movement
    ################################################################################################

    # Limit a vector to a maximum value
    def truncate(self, vector, limit):
        ratio = limit / len(vector)

        if ratio < 1:
            result = 1
        else:
            result = ratio

        return vector * result


    def update0(self, possibleNeighbouringBoids):
        self.config['MAX_VELOCITY'] = 4

        # Calculate the neighbouring boids for the current boid
        self.possibleNeighbours = possibleNeighbouringBoids
        self.calculateNeighbours()

        # If tracking boid, highlight its neighbouring boids
        if self.config['trackBoid'] and (self.boidID == self.config['boidToTrack']): 
            for boid in self.neighbouringBoids:
                self.boidGPU.highlightBoid(True, boid.boidID)

        # Use the properties of the neighbouring boids to determine where the current boid should go
        if len(self.neighbouringBoids) > 0:
            alignment = self.alignment()
            cohesion = self.coehsion()
            repulsion = self.repulsion()

            # Identify a target point
            target = self.position + (alignment + cohesion + repulsion)

        else:
            # TODO: If a boid has no neighbours, probably want it to wander
            target = self.position + self.velocity

        # Seek -------------------------------------------------------------------------------------
        # The desired velocity is a vector from the boid to the target
        desiredVelocity = (self.normalise(self.position - target)) * self.config['MAX_VELOCITY']

        # The steering force is the result of the desired velocity subtracted by the current 
        # velocity and it pushes the character towards the target. 
        steering = desiredVelocity - self.velocity       

        # Contain ----------------------------------------------------------------------------------
        # TODO: Move the steer point if it outside the simulation area

        # Obstacle Avoidance -----------------------------------------------------------------------
        # if self.config['trackBoid'] and (self.boidID == self.config['boidToTrack']): 
        #     # dynamicLength = abs(self.velocity) / self.config['MAX_VELOCITY']
        #     ahead = self.position + self.normalise(self.velocity) * self.config['VISION_RADIUS']
        #     ahead2 = self.position + self.normalise(self.velocity) * self.config['VISION_RADIUS'] * 0.5

        #     print "Position: " + str(self.position)
        #     print "Ahead:    " + str(ahead)
        #     print "velocity: " + str(self.velocity)

        #     self.boidGPU.drawLine(self.position, ahead, "AheadLine")

        #     obstacle = True

        #     # Top edge
        #     if ahead[1] < 0:
        #         lineA = [0, 0, self.config['width'], 0]
        #         lineB = np.concatenate((self.position, ahead), axis = 1)
        #         obstacleCenter = self.intersectionBetweenTwoLines(lineA, lineB)
        #     # Bottom edge
        #     elif ahead[1] > self.config['width']:
        #         lineA = [0, self.config['width'], self.config['width'], self.config['width']]
        #         lineB = np.concatenate((self.position, ahead), axis = 1)
        #         obstacleCenter = self.intersectionBetweenTwoLines(lineA, lineB)
        #     # Left edge
        #     elif ahead[0] < 0:
        #         lineA = [0, 0, 0, self.config['width']]
        #         lineB = np.concatenate((self.position, ahead), axis = 1)
        #         obstacleCenter = self.intersectionBetweenTwoLines(lineA, lineB)
        #     # Right edge
        #     elif ahead[0] > self.config['width']:
        #         lineA = [0, self.config['width'], self.config['width'], self.config['width']]
        #         lineB = np.concatenate((self.position, ahead), axis = 1)
        #         obstacleCenter = self.intersectionBetweenTwoLines(lineA, lineB)
        #     else:
        #         obstacle = False

        #     if obstacle:
        #         MAX_AVOID_FORCE = 1
        #         avoidanceForce = ahead - obstacleCenter
        #         avoidanceForce = self.normalise(avoidanceForce) * MAX_AVOID_FORCE

        #         print "Obstacle Center: " + str(obstacleCenter)
        #         print "Aoidance Force:  " + str(avoidanceForce)

        #         steering += avoidanceForce

        # Update -----------------------------------------------------------------------------------
        # The steering force is truncated to ensure it does not exceed the maximum force on boid
        # steering = self.truncate (steering, max_force)
        
        # The velocity vector is truncated to limit the speed of a boid
        self.velocity = self.truncate(self.velocity + steering, self.config['MAX_VELOCITY'])

        # Calculate the boid's position
        self.position = self.position + self.velocity



        # Obstacle avoidance - assume everything is a sphere
        # Need a bar ahead of the boid of length vision radius and width that of the sphere containing the boid
        # Determines if an ostacle intersects this bar
        # Steering to avoid the nearest obstacle is computed by negating the (lateral) side-up projection
        # of the obstacle's centre


        # Wall containment
        # First predict the boid's future position. If the future position is not in the allowed 
        # region, then the boid need to steer back to the allowed region.
        # This can be done using seek with an inside point
        # OR determine the intersection with the boundary, get the surface normal at the point

        # Acceleration??
        # acceleration = steering_force / mass



    # # Contain the boid within the simulation bounds
    # # 
    # # Calculate the new position
    # # If the new position is beyond the boundary
    # #   Determine the perpendicular distance to the nearest point on the boundary
    # #   Add this distance to the boundary point to give a new point in the area
    # #   This new point is the seek point
    # # 
    # # TODO:  Supply with ahead point, not actual position
    # # FIXME: Very jumpy behaviour at the moment, need to change velocity
    # def containBoid(self, futurePosition):
    #     correctedPosition = futurePosition

    #     # Top edge
    #     if futurePosition[1] < 0:
    #         dist = self.distanceFromPointToLine([0, 0], [self.config['width'], 0], futurePosition)
    #         # print "Future position is " + str(dist) + " beyond top edge"
    #         correctedPosition[1] = 0 + dist     
    #     # Bottom edge
    #     elif futurePosition[1] > self.config['width']:
    #         dist = self.distanceFromPointToLine([0, self.config['width']], [self.config['width'], self.config['width']], futurePosition)
    #         # print "Future position is " + str(dist) + " beyond bottom edge"
    #         correctedPosition[1] = self.config['width'] - dist  

    #     # Left edge
    #     if futurePosition[0] < 0:
    #         dist = self.distanceFromPointToLine([0, 0], [0, self.config['width']], futurePosition)
    #         # print "Future position is " + str(dist) + " beyond left edge"
    #         correctedPosition[0] = 0 + dist  
    #     # Right edge
    #     elif futurePosition[0] > self.config['width']:
    #         dist = self.distanceFromPointToLine([0, self.config['width']], [self.config['width'], self.config['width']], futurePosition)
    #         # print "Future position is " + str(dist) + " beyond right edge"
    #         correctedPosition[0] = self.config['width'] - dist  

    #     # print correctedPosition
    #     # self.position = correctedPosition
    #     return correctedPosition


    # # Based on http://gamedevelopment.tutsplus.com/tutorials/understanding-steering-behaviors-collision-avoidance--gamedev-7777
    # def obstacleAvoidance(self):
        
    #     # Define a vector in the direction of the boid's velocity that is the length of the vision radius
    #     # dynamicLength = len(self.velocity) / self.config['MAX_VELOCITY']
    #     ahead = self.position + self.normalise(self.velocity) * self.config['VISION_RADIUS']

        
    #     # Draw the line on the screen
    #     if self.config['trackBoid'] and (self.boidID == self.config['boidToTrack']): 
    #         self.boidGPU.drawLine(self.position, ahead, "AheadLine")

    #     # # If the difference in distance between the boundary and ahead point is less than the vision radius - collision
    #     # distance = self.distanceFromPointToLine([0, 0], [self.boidGPU.getWindowSize()[0], 0], ahead)
    #     # print distance
        
    #     # The greater the MAX_AVOID_FORCE< the stronger the boid is repelled from the obstacle
    #     MAX_AVOID_FORCE = 10

    #     avoidance_force = 0

    #     # # If the ahead vector is above the top boundary
    #     # if ahead[1] < 0:
    #     #     # Determine the intersection point between the boundary line and the ahead vector
    #     #     lineA = [0, 0, self.config['width'], 0]
    #     #     lineB = np.concatenate((self.position, ahead), axis = 1)
    #     #     insersectionPoint = self.intersectionBetweenTwoLines(lineA, lineB)

            
    #     # Top edge
    #     if ahead[1] < 0:
    #         lineA = [0, 0, self.config['width'], 0]
    #         lineB = np.concatenate((self.position, ahead), axis = 1)
    #         insersectionPoint = self.intersectionBetweenTwoLines(lineA, lineB)
    #         avoidance_force = ahead - insersectionPoint
    #         avoidance_force = self.normalise(avoidance_force) * MAX_AVOID_FORCE
    #     # Bottom edge
    #     elif ahead[1] > self.config['width']:
    #         lineA = [0, self.config['width'], self.config['width'], self.config['width']]
    #         lineB = np.concatenate((self.position, ahead), axis = 1)
    #         insersectionPoint = self.intersectionBetweenTwoLines(lineA, lineB)
    #         avoidance_force = ahead - insersectionPoint
    #         avoidance_force = self.normalise(avoidance_force) * MAX_AVOID_FORCE
    #     # Left edge
    #     elif ahead[0] < 0:
    #         lineA = [0, 0, 0, self.config['width']]
    #         lineB = np.concatenate((self.position, ahead), axis = 1)
    #         insersectionPoint = self.intersectionBetweenTwoLines(lineA, lineB)
    #         avoidance_force = ahead - insersectionPoint
    #         avoidance_force = self.normalise(avoidance_force) * MAX_AVOID_FORCE
    #     # Right edge
    #     elif ahead[0] > self.config['width']:
    #         lineA = [0, self.config['width'], self.config['width'], self.config['width']]
    #         lineB = np.concatenate((self.position, ahead), axis = 1)
    #         insersectionPoint = self.intersectionBetweenTwoLines(lineA, lineB)

    #         # Calculate avoidance force
    #         avoidance_force = ahead - insersectionPoint
    #         avoidance_force = self.normalise(avoidance_force) * MAX_AVOID_FORCE


    #         # print str(insersectionPoint) + " - " + str(avoidance_force) 

    #     # Check if there is an obstacle ahead
    #     # FIXME: Fails if 90 degree
    #     # unitVel = self.velocity / np.abs(self.velocity)
    #     # print str(self.velocity) + " - " + str(unitVel) + " - " + str(unitVel * self.config['VISION_RADIUS'])

    #     # Check the distance from the end of the ahead vector to the top boundary
    #     # print self.distanceFromPointToLine([0, 0], [self.boidGPU.getWindowSize()[0], 0], self.position + ahead)
        

    #     # if top:
    #     #     self.repulsionMod += ([self.position[0], 0] - self.position)
    #     #     nearEdgeCount += 1
    #     # if left:
    #     #     self.repulsionMod += ([0, self.position[1]] - self.position)
    #     #     nearEdgeCount += 1
    #     # if bottom:
    #     #     self.repulsionMod += ([self.position[0], self.canvas.winfo_width()] - self.position)
    #     #     nearEdgeCount += 1
    #     # if right:
    #     #     self.repulsionMod += ([self.canvas.winfo_width(), self.position[1]] - self.position)
    #     #     nearEdgeCount += 1

    #     # tempVector = self.position + ahead

    #     # if tempVector[0] < 0:
    #     #     self.velocity = -self.velocity

    #     # elif tempVector[0] > self.boidGPU.getWindowSize()[0]:
    #     #     self.velocity = -self.velocity

    #     # elif tempVector[1] < 0:
    #     #     self.velocity = -self.velocity

    #     # elif tempVector[1] > self.boidGPU.getWindowSize()[0]:
    #     #     self.velocity = -self.velocity

    #     return avoidance_force


    # Intersection of two lines
    # From: http://stackoverflow.com/questions/4543506/algorithm-for-intersection-of-2-lines
    def intersectionBetweenTwoLines(self, lineA, lineB):
        A1 = lineA[3] - lineA[1]
        B1 = lineA[0] - lineA[2]
        C1 = (A1 * lineA[0]) + (B1 * lineA[1])

        A2 = lineB[3] - lineB[1]
        B2 = lineB[0] - lineB[2]
        C2 = (A2 * lineB[0]) + (B2 * lineB[1])

        delta = (A1 * B2) - (A2 * B1)
        if delta == 0:
            raise ZeroDivisionError("Lines are parallel")

        x = ((B2 * C1) - (B1 * C2)) / delta
        y = ((A1 * C2) - (A2 * C1)) / delta

        return [x, y]


    # Calculates the shortest distance from a boid to a boundary line
    # From http://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line 
    def distanceFromPointToLine(self, lineStart, lineEnd, point):
        # tmp = lineEnd[0]
        # lineEnd[0] = lineEnd[1]
        # lineEnd[1] = tmp

        lineEnd[1] - lineStart[1]

        partOne = ((lineEnd[1] - lineStart[1]) * point[0]) - ((lineEnd[0] - lineStart[0]) * 
            point[1]) + (lineEnd[0] * lineStart[1]) - (lineEnd[1] * lineStart[0])

        partTwo = ((lineEnd[1] - lineStart[1]) ** 2) + ((lineEnd[0] - lineStart[0]) ** 2)

        result = abs(partOne) / np.sqrt(partTwo)

        return result


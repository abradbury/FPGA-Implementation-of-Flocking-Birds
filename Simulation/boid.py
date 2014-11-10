#!/usr/bin/python


import numpy as np                  # Used in various mathematical operations
import random                       # Used to randomly position the boids on initialisation
import logging                      # Used to handle the textual output
import warnings                     # Used to catch an invalid divide warning


# A class representing an individual boid
class Boid:

    def __init__(self, canvas, _location, _boidID, initPosition, _colour):
        self.location = _location
        self.logger = self.location.logger

        # Colour the boids based on their original location if debugging
        if self.logger.getEffectiveLevel() == logging.DEBUG:
            self.colour = _colour
            self.outlineColour = _colour
            self.boidWidth = 2
        else:
            self.colour = "red"
            self.outlineColour = "white"
            self.boidWidth = 1

        # Create the boids
        self.boidID = _boidID
        self.bearing = np.pi
        self.step = 10
        self.canvas = canvas
        self.position = initPosition

        self.neighbouringBoids = []

        self.MAX_VELOCITY = 10
        self.VISION_RADIUS = 200

        self.ALIGNMENT_WEIGHT = 1
        self.COHESION_WEIGHT = 1
        self.REPULSION_WEIGHT = 1

        self.randomVelX = random.randint(-self.MAX_VELOCITY, self.MAX_VELOCITY)
        self.randomVelY = random.randint(-self.MAX_VELOCITY, self.MAX_VELOCITY)
        self.velocity = np.array([self.randomVelX, self.randomVelY], dtype = np.float_)

        # Define the boid polygon points
        self.x0 = self.position[0] - self.step
        self.y0 = self.position[1] - self.step
        self.x1 = self.position[0]
        self.y1 = self.position[1] + self.step
        self.x2 = self.position[0] + self.step
        self.y2 = self.position[1] - self.step
        self.x3 = self.position[0]
        self.y3 = self.position[1] - (self.step/2)

        # Draw the boid
        self.boid = self.canvas.create_polygon(self.x0, self.y0, self.x1, self.y1, self.x2, 
            self.y2, self.x3, self.y3, fill = self.colour, outline = self.outlineColour, 
            width = self.boidWidth, tags = ("B" + str(self.boidID)))

        self.rotate(np.arctan2(self.velocity[0], self.velocity[1]))

        self.logger.debug("Created boid with ID " + str(self.boidID))

    # Update the boid's position based on the information contained in its neighbourhood. Only 
    # commit the changes when all the boids have calculated their next move. Otherwise, a boid 
    # would move based on its neighbours and when one of its neighbouts comes to move it will use 
    # the new position of the original boid, not its original position. 
    def update(self, possibleNeighbouringBoids):
        self.possibleNeighbours = possibleNeighbouringBoids

        # for boid in self.neighbouringBoids:
        #     boid.highlightBoid(False)

        self.calculateNeighbours()
        # self.followBoid(42, True)

        if len(self.neighbouringBoids) > 0:
            self.cohesionMod = self.coehsion()
            self.alignmentMod = self.alignment()
            self.repulsionMod = self.repulsion()

            self.movement = ((self.COHESION_WEIGHT * self.cohesionMod) + 
                (self.ALIGNMENT_WEIGHT * self.alignmentMod) + 
                (self.REPULSION_WEIGHT * self.repulsionMod))

        else:
            self.movement = 0
           

    # Move the boid to the calculate positon
    def draw(self, colour):
        self.velocity += self.movement

        # Specify the boid fill colour based on the debug level
        if self.logger.getEffectiveLevel() == logging.DEBUG:
            self.fillColour = colour
        else:
            self.fillColour = "red"

        # Bound the velocity to the maximum allowed
        if self.velocity[0] > self.MAX_VELOCITY:
            self.velocity[0] = self.MAX_VELOCITY

        if self.velocity[1] > self.MAX_VELOCITY:
            self.velocity[1] = self.MAX_VELOCITY

        if self.velocity[0] < -self.MAX_VELOCITY:
            self.velocity[0] = -self.MAX_VELOCITY

        if self.velocity[1] < -self.MAX_VELOCITY:
            self.velocity[1] = -self.MAX_VELOCITY

        self.position += self.velocity

        # Contain the boids in the simulation area
        if self.position[0] < 0:
            self.velocity = -self.velocity
            self.position += self.velocity

        elif self.position[0] > self.canvas.winfo_width():
            self.velocity = -self.velocity
            self.position += self.velocity

        elif self.position[1] < 0:
            self.velocity = -self.velocity
            self.position += self.velocity

        elif self.position[1] > self.canvas.winfo_width():
            self.velocity = -self.velocity
            self.position += self.velocity

        # Calculate the position of the points of the boid object
        self.x0 = self.position[0] - self.step
        self.y0 = self.position[1] - self.step
        self.x1 = self.position[0]
        self.y1 = self.position[1] + self.step
        self.x2 = self.position[0] + self.step
        self.y2 = self.position[1] - self.step
        self.x3 = self.position[0]
        self.y3 = self.position[1] - (self.step/2)

        self.canvas.coords("B" + str(self.boidID), self.x0, self.y0, self.x1, self.y1, self.x2, 
            self.y2, self.x3, self.y3)
        self.canvas.itemconfig(self.boid, fill = self.fillColour) 

        self.rotate(np.arctan2(self.velocity[0], self.velocity[1]))

        # self.followBoid(42, False)


    # Follows a boid as it moves around the area. The boid has its vision circle shown and is 
    # coloured blue. Any neighbouring boid is coloured green. 
    def followBoid(self, _boidID, update):
        if self.boidID == _boidID:
            if update == True:
                self.canvas.itemconfig(self.boid, fill = "red") 
                self.canvas.delete("boidCircle")
                for boid in self.neighbouringBoids:
                    boid.highlightBoid(False)

            else:
                self.canvas.itemconfig(self.boid, fill = "blue")

                self.canvas.create_oval(self.position[0] - self.VISION_RADIUS, 
                    self.position[1] - self.VISION_RADIUS, self.position[0] + self.VISION_RADIUS, 
                    self.position[1] + self.VISION_RADIUS, outline = "yellow", tags = "boidCircle")

                self.logger.debug("Boid " + str(self.boidID) + " has " + 
                    str(len(self.neighbouringBoids)) + " neighbouring boids: ")
                self.logger.debug(" ".join([str(b.boidID) for b in self.neighbouringBoids]))

                for boid in self.neighbouringBoids:
                    boid.highlightBoid(True)


    def highlightBoid(self, on):
        if on:
            self.canvas.itemconfig(self.boid, fill = "green")
        else:
            self.canvas.itemconfig(self.boid, fill = "red")


    # Calculate the neighbouring boids based on the Euclidean distance between the current boid and 
    # the possible neighbour. Possible neighbouring boids include boids from neighbouring locations.
    def calculateNeighbours(self):
        self.neighbouringBoids = []

        for boid in self.possibleNeighbours:
            if boid.boidID != self.boidID:
                dist = np.linalg.norm(boid.position - self.position)
                if dist < self.VISION_RADIUS:
                    self.neighbouringBoids.append(boid)


    # A boid will move towards the centre of mass of its neighbourhood
    def coehsion(self):
        self.cohesionMod = 0;
        for boid in self.neighbouringBoids:
            self.cohesionMod += boid.position

        self.cohesionMod /= len(self.neighbouringBoids)
        self.cohesionMod -= self.position
        self.cohesionMod = (self.cohesionMod / np.linalg.norm(self.cohesionMod))

        return self.cohesionMod


    # A boid will align itself with the average orientation of its neighbours
    def alignment(self):
        self.alignmentMod = 0
        for boid in self.neighbouringBoids:
            self.alignmentMod += boid.velocity

        warnings.filterwarnings('error')

        self.alignmentModPrev = self.alignmentMod
        try:
            self.alignmentMod /= len(self.neighbouringBoids)
        except RuntimeWarning, w:
            print w
            print self.alignmentModPrev
            print self.alignmentMod
            print len(self.neighbouringBoids)


        self.norm = np.linalg.norm(self.alignmentMod)
        if np.all(self.norm == 0):
            self.alignmentMod = 0
        else:
            self.alignmentModPrev = self.alignmentMod
            try:
                self.alignmentMod = (self.alignmentMod / np.linalg.norm(self.alignmentMod))
            except RuntimeWarning, w:
                print w
                print self.alignmentModPrev
                print self.alignmentMod
                print len(self.neighbouringBoids)
                print np.linalg.norm(self.alignmentMod)

    # [ 0.  0.] = [ 0.  0.] / 0.0 

    #       boids.py:222: RuntimeWarning: invalid value encountered in divide
    #           self.alignmentMod = (self.alignmentMod / np.linalg.norm(self.alignmentMod))
    # due to 0 values e.g. [0. 0.] in alignment mod

        return self.alignmentMod


    # A boid does not want to get too close to its neighbours and needs its personal space
    def repulsion(self):
        self.repulsionMod = 0
        for boid in self.neighbouringBoids:
            self.repulsionMod += (boid.position - self.position)

        self.repulsionMod /= len(self.neighbouringBoids)
        self.repulsionMod *= -1
        self.repulsionMod = (self.repulsionMod / np.linalg.norm(self.repulsionMod))

        return self.repulsionMod


    # Rotation definition based on an answer from StackOverflow: http://stackoverflow.com/a/3409039
    def rotate(self, radians):
        # Convert to radians for trig functions
        # radians = degrees * np.pi / 180
        self.bearing -= radians
        self.bearing = self.bearing % (2 * np.pi)

        def _rot(x, y):
            # Note: the rotation is done in the opposite fashion from for a right-handed coordinate 
            #   system due to the left-handedness of computer coordinates
            x -= self.position[0]
            y -= self.position[1]
            _x = x * np.cos(radians) + y * np.sin(radians)
            _y = -x * np.sin(radians) + y * np.cos(radians)
            return _x + self.position[0], _y + self.position[1]

        self.x0, self.y0 = _rot(self.x0, self.y0)
        self.x1, self.y1 = _rot(self.x1, self.y1)
        self.x2, self.y2 = _rot(self.x2, self.y2)
        self.x3, self.y3 = _rot(self.x3, self.y3)

        self.canvas.coords(self.boid, self.x0, self.y0, self.x1, self.y1, self.x2, self.y2, self.x3, self.y3)


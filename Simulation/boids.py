#!/usr/bin/python

from Tkinter import *       # Used to draw shapes for the simulation
import numpy as np          # Used in various mathematical operations
import random               # Used to randomly position the boids on initialisation

import sys
import time


## MUST DOs ========================================================================================
# TODO: Create boid handover functionality between locations
# TODO: Rework drawing routine so that it can be automatic
# TODO: Implement locations as threads
# TODO: Add timing functions to computation

# FIXME: Investigate arithmetic warning on rule calculation
# FIXME: Boids don't seem to be repelling each other that much, they are on top of one another

## MAY DOs =========================================================================================
# TODO: Enable logging to control debug output
# TODO: Add keybinding to capture return key and simulate button press
# TODO: Add acceleration to smooth movement
# TODO: Add timestep counter


class Boid:

    def __init__(self, canvas, _location, _boidID, initPosition):
        self.location = _location

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
            self.y2, self.x3, self.y3, fill = "red", outline = "white", tags = ("B" + str(self.boidID)))

        self.rotate(np.arctan2(self.velocity[0], self.velocity[1]))

        print "Created boid with ID " + str(self.boidID)


    def getPosition(self):
        return self.position


    def getVelocity(self):
        return self.velocity


    # Update the boid's position based on the information contained in its neighbourhood. Only 
    # commit the changes when all the boids have calculated their next move. Otherwise, a boid 
    # would move based on its neighbours and when one of its neighbouts comes to move it will use 
    # the new position of the original boid, not its original position. 
    def update(self, possibleNeighbouringBoids):
        self.possibleNeighbours = possibleNeighbouringBoids

        for boid in self.neighbouringBoids:
            boid.highlightBoid(False)

        self.calculateNeighbours()
        self.followBoid(42, True)

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
    def commit(self):
        self.velocity += self.movement

        # Bounds the velocity to the maximum allowed
        if self.velocity[0] > self.MAX_VELOCITY:
            self.velocity[0] = self.MAX_VELOCITY

        if self.velocity[1] > self.MAX_VELOCITY:
            self.velocity[1] = self.MAX_VELOCITY

        if self.velocity[0] < -self.MAX_VELOCITY:
            self.velocity[0] = -self.MAX_VELOCITY

        if self.velocity[1] < -self.MAX_VELOCITY:
            self.velocity[1] = -self.MAX_VELOCITY


        self.position += self.velocity

        # print self.velocity
        # print self.position

        if self.position[0] < 0:
            # print "X less than 0"
            # self.velocity -= self.movement
            self.velocity = -self.velocity
            self.position += self.velocity

        elif self.position[0] > self.canvas.winfo_width():
            # print "X greater than max"
            # self.velocity -= self.movement
            self.velocity = -self.velocity
            self.position += self.velocity

        elif self.position[1] < 0:
            # print "Y less than 0"
            # self.velocity -= self.movement
            self.velocity = -self.velocity
            self.position += self.velocity

        elif self.position[1] > self.canvas.winfo_width():
            # print "Y greater than max"
            # self.velocity -= self.movement
            self.velocity = -self.velocity
            self.position += self.velocity

        # print self.velocity
        # print self.position

        # self.canvas.itemconfig(self.boid, fill = "blue") 

        # print "=======Press enter key"
        # raw_input()
        # time.sleep(1)


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

        self.rotate(np.arctan2(self.velocity[0], self.velocity[1]))

        self.followBoid(42, False)


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

                print("Boid " + str(self.boidID) + " has " + str(len(self.neighbouringBoids)) + 
                    " neighbouring boids: ")
                print (" ".join([str(b.boidID) for b in self.neighbouringBoids]))

                for boid in self.neighbouringBoids:
                    boid.highlightBoid(True)
                #     print("Boid " + str(boid.boidID) + " has position " + str(boid.position) + 
                #         " and velocty " + str(boid.velocity)) 
        
                # print str(boid.movement)


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

        self.alignmentMod /= len(self.neighbouringBoids)
        self.alignmentMod = (self.alignmentMod / np.linalg.norm(self.alignmentMod))

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


class Location:

    def __init__(self, canvas, _simulation, _locationID, locationCoords, _initialBoidCount):
        self.simulation = _simulation
        self.locationID = _locationID

        self.boids = []
        self.initialBoidCount = _initialBoidCount

        # Draw the location bounds
        canvas.create_rectangle(locationCoords[0], locationCoords[1], locationCoords[2], 
            locationCoords[3], outline = "yellow", tags = "L" + str(self.locationID))

        # Draw the location's boids
        for i in range (0, self.initialBoidCount):
            # Randomly position the boid on initialisation
            self.randomX = random.randint(locationCoords[0], locationCoords[2]);
            self.randomY = random.randint(locationCoords[1], locationCoords[3]);
            self.initialPosition = np.array([self.randomX, self.randomY], dtype = np.float_)
            self.boidID = ((self.locationID - 1)* self.initialBoidCount) + i + 1

            boid = Boid(canvas, self, self.boidID, self.initialPosition)
            self.boids.append(boid)

        print "Created location " + str(self.locationID) + " with " + str(self.initialBoidCount) + " boids"


    def update(self, commit):
        if commit == False:
            self.possibleNeighbouringBoids = self.getPossibleNeighbouringBoids()

            for i in range(0, self.initialBoidCount):
                self.boids[i].update(self.possibleNeighbouringBoids)
        else:
            for i in range(0, self.initialBoidCount):
                self.boids[i].commit()


    # Return a list containing the boids currently controlled by this location
    def getBoids(self):
        return self.boids


    # Return a list containing the boids from each neighbouring location
    def getPossibleNeighbouringBoids(self):
        self.neighbouringLocations = self.simulation.getNeighbouringLocations(self.locationID)
        # print "This location has the following location neighbours: " + (" ".join([str(l) for l in self.neighbouringLocations]))

        # Need the slice operation or else updating 
        self.neighbouringBoids = self.boids[:]

        for locationIndex in self.neighbouringLocations:
            if locationIndex != 0:
                self.neighbouringBoids += self.simulation.getLocationBoids(locationIndex)

        return self.neighbouringBoids



class Simulation:

    def __init__(self):
        self.root = Tk()
        self.root.wm_title("Boid Simulation")

        frame = Frame(self.root)
        frame.pack()

        self.width = 700
        self.height = 700
        self.boidCount = 90

        # Create the window
        self.canvas = Canvas(frame, bg = "black", width = self.width, height = self.height)
        self.canvas.pack();
        
        # Create the buttons
        self.timeButton = Button(frame, text = "Next Time Step", command = self.buttonAction)
        self.timeButton.pack(side = LEFT)
        self.quitButton = Button(frame, text = "Quit", command = frame.quit)
        self.quitButton.pack(side = LEFT)

        # Needed so that the canvas sizes can be used later
        self.root.update()

        # Create the locations
        self.allowedLocationCounts = np.array([1, 2, 4, 9, 16, 25, 36])
        self.locationCount = self.allowedLocationCounts[3]
        self.initialBoidCount = self.boidCount / self.locationCount
        self.locationSize = round(self.canvas.winfo_width() / np.sqrt(self.locationCount))

        self.locations = []

        self.locationCoords = np.array([0, 0, 0, 0])
        for i in range(0, self.locationCount):
            self.locationCoords[0] = (i % 3) * self.locationSize
            self.locationCoords[1] = int(np.floor(i / 3)) * self.locationSize
            self.locationCoords[2] = self.locationCoords[0] + self.locationSize
            self.locationCoords[3] = self.locationCoords[1] + self.locationSize

            print "[" + str(self.locationCoords[0]) + "," + str(self.locationCoords[1]) + "]"

            loc = Location(self.canvas, self, i + 1, self.locationCoords, self.initialBoidCount)
            self.locations.append(loc)

        print "=======Press button"

        # Start everything going
        self.root.mainloop()


    def buttonAction(self):
        # for b in range(1, 1000):
        for i in range(0, self.locationCount):
            # print "Calculating next boid positions for location " + str(self.locations[i].locationID) + "..."
            self.locations[i].update(False)

        for i in range(0, self.locationCount):
            # print "Moving boids to calculated positions for location " + str(self.locations[i].locationID) + "..."
            self.locations[i].update(True)


    def getNeighbouringLocations(self, locationID):
        if locationID == 1:
            # self.neighbouringLocations = [0, 0, 0, 2, 5, 4, 0, 0]
            self.neighbouringLocations = [2, 3, 4, 5, 6, 7, 8, 9]
        elif locationID == 2:
            # self.neighbouringLocations = [0, 0, 0, 3, 6, 5, 4, 1]
            self.neighbouringLocations = [1, 3, 4, 5, 6, 7, 8, 9]
        elif locationID == 3:
            # self.neighbouringLocations = [0, 0, 0, 0, 0, 6, 5, 2]
            self.neighbouringLocations = [2, 1, 4, 5, 6, 7, 8, 9]
        elif locationID == 4:
            # self.neighbouringLocations = [0, 1, 2, 5, 8, 7, 0, 0]
            self.neighbouringLocations = [2, 3, 1, 5, 6, 7, 8, 9]
        elif locationID == 5:
            # self.neighbouringLocations = [1, 2, 3, 6, 9, 8, 7, 4]
            self.neighbouringLocations = [2, 3, 4, 1, 6, 7, 8, 9]
        elif locationID == 6:
            # self.neighbouringLocations = [2, 3, 0, 0, 0, 9, 8, 5]
            self.neighbouringLocations = [2, 3, 4, 5, 1, 7, 8, 9]
        elif locationID == 7:
            # self.neighbouringLocations = [0, 4, 5, 8, 0, 0, 0, 0]
            self.neighbouringLocations = [2, 3, 4, 5, 6, 1, 8, 9]
        elif locationID == 8:
            # self.neighbouringLocations = [4, 5, 6, 9, 0, 0, 0, 7]
            self.neighbouringLocations = [2, 3, 4, 5, 6, 7, 1, 9]
        elif locationID == 9:
            # self.neighbouringLocations = [5, 6, 0, 0, 0, 0, 0, 8]
            self.neighbouringLocations = [2, 3, 4, 5, 6, 7, 8, 1]

        return self.neighbouringLocations


    def getLocationBoids(self, locationID):
        return self.locations[locationID - 1].getBoids()


boidSimulation = Simulation()

if __name__ == '__main__':
    print "yolo";

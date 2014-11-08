#!/usr/bin/python

from Tkinter import *       # Used to draw shapes for the simulation
import numpy as np          # Used in various mathematical operations
import random               # Used to randomly position the boids on initialisation

import sys
import time


## MUST DOs ========================================================================================
# TODO: Implement locations as threads
# TODO: Add timing functions to computation

# FIXME: Investigate arithmetic warning on rule calculation
# FIXME: Boids don't seem to be repelling each other that much, they are on top of one another

## MAY DOs =========================================================================================
# TODO: Enable logging to control debug output
# TODO: Add keybinding to capture return key and simulate button press
# TODO: Add acceleration to smooth movement
# TODO: Calculate a locations neighbours programmatically


class Boid:

    def __init__(self, canvas, _location, _boidID, initPosition, _colour):
        self.location = _location
        self.colour = _colour

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
            self.y2, self.x3, self.y3, fill = self.colour, outline = self.colour, width = 2, tags = ("B" + str(self.boidID)))

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
        self.canvas.itemconfig(self.boid, fill = colour) 

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

    def __init__(self, canvas, _simulation, _locationID, _locationCoords, _initialBoidCount, _colour):
        self.simulation = _simulation
        self.locationID = _locationID
        self.locationCoords = np.copy(_locationCoords)
        self.colour = _colour

        self.boids = []
        self.boidCount = _initialBoidCount

        print self.locationCoords

        # Draw the location bounds
        canvas.create_rectangle(self.locationCoords[0], self.locationCoords[1], self.locationCoords[2], 
            self.locationCoords[3], outline = self.colour, tags = "L" + str(self.locationID))

        # Draw the location's boids
        for i in range (0, self.boidCount):
            # Randomly position the boid on initialisation
            self.randomX = random.randint(self.locationCoords[0], self.locationCoords[2]);
            self.randomY = random.randint(self.locationCoords[1], self.locationCoords[3]);
            self.initialPosition = np.array([self.randomX, self.randomY], dtype = np.float_)
            self.boidID = ((self.locationID - 1)* self.boidCount) + i + 1

            boid = Boid(canvas, self, self.boidID, self.initialPosition, self.colour)
            self.boids.append(boid)

        print "Created location " + str(self.locationID) + " with " + str(self.boidCount) + " boids"


    def update(self, draw):
        if draw == False:
            self.possibleNeighbouringBoids = self.getPossibleNeighbouringBoids()

            for i in range(0, self.boidCount):
                self.boids[i].update(self.possibleNeighbouringBoids)

            for boid in self.boids:
                self.determineBoidTransfer(boid)

        else:
            for i in range(0, self.boidCount):
                # print str(i) + " - " + str(self.boidCount) +  " - " + str(len(self.boids)) + " " + str(self.boids[0].boidID)
                self.boids[i].draw(self.colour)


    #TODO: indexing
    def determineBoidTransfer(self, boid):

        # print str(self.locationCoords) + str(boid.position)

        # If the location has a neighbour to the NORTHWEST and the boid is beyond its northern AND western boundaries
        if (self.neighbouringLocations[0] != 0) and (boid.position[1] < self.locationCoords[1]) and (boid.position[0] < self.locationCoords[0]):
            print "Boid " + str(boid.boidID) + " is beyond the NORTH and WESTERN boundaries of location " + str(self.locationID)
            self.transferBoid(boid, self.neighbouringLocations[0])

        # If the location has a neighbour to the NORTHEAST and the boid is beyond its northern AND eastern boundaries
        elif (self.neighbouringLocations[2] != 0) and (boid.position[1] < self.locationCoords[1]) and (boid.position[0] > self.locationCoords[2]):
            print "Boid " + str(boid.boidID) + " is beyond the NORTH and EASTERN boundaries of location " + str(self.locationID)
            self.transferBoid(boid, self.neighbouringLocations[2])

        # If the location has a neighbour to the SOUTHEAST and the boid is beyond its southern AND eastern boundaries
        elif (self.neighbouringLocations[4] != 0) and (boid.position[1] > self.locationCoords[3]) and (boid.position[0] > self.locationCoords[2]):
            print "Boid " + str(boid.boidID) + " is beyond the SOUTHERN and EASTERN boundaries of location " + str(self.locationID)
            self.transferBoid(boid, self.neighbouringLocations[4])

        # If the location has a neighbour to the SOUTHWEST and the boid is beyond its southern AND western boundaries
        elif (self.neighbouringLocations[6] != 0) and (boid.position[1] > self.locationCoords[3]) and (boid.position[0] < self.locationCoords[0]):
            print "Boid " + str(boid.boidID) + " is beyond the SOUTHERN and WESTERN boundaries of location " + str(self.locationID)
            self.transferBoid(boid, self.neighbouringLocations[6])

        # If the location has a neighbour to the NORTH and the boid is beyond its northern boundary
        elif (self.neighbouringLocations[1] != 0) and (boid.position[1] < self.locationCoords[1]):
            print "Boid " + str(boid.boidID) + " is beyond the NORTH boundary of location " + str(self.locationID)
            self.transferBoid(boid, self.neighbouringLocations[1])

        # If the location has a neighbour to the EAST and the boid is beyond its eastern boundary
        elif (self.neighbouringLocations[3] != 0) and (boid.position[0] > self.locationCoords[2]):
            print "Boid " + str(boid.boidID) + " is beyond the EAST boundary of location " + str(self.locationID)
            self.transferBoid(boid, self.neighbouringLocations[3])  
            
        # If the location has a neighbour to the SOUTH and the boid is beyond its southern boundary
        elif (self.neighbouringLocations[5] != 0) and (boid.position[1] > self.locationCoords[3]):
            print "Boid " + str(boid.boidID) + " is beyond the SOUTH boundary of location " + str(self.locationID)
            self.transferBoid(boid, self.neighbouringLocations[5])

        # If the location has a neighbour to the WEST and the boid is beyond its western boundary
        elif (self.neighbouringLocations[7] != 0) and (boid.position[0] < self.locationCoords[0]):
            print "Boid " + str(boid.boidID) + " is beyond the WEST boundary of location " + str(self.locationID)
            self.transferBoid(boid, self.neighbouringLocations[7])

        # boid.highlightBoid(True)
        # raw_input()
        # boid.highlightBoid(False)


    # Accept a boid transferred from another location and add it to this locations boid list
    def acceptBoid(self, boid, fromID):
        self.boids.append(boid)
        self.boidCount += 1

        print("Location " + str(self.locationID) + " accepted boid " + str(boid.boidID) + 
            " from location " + str(fromID) + " and now has " + str(self.boidCount) + " boids")


    # Transfer a boid from this location to another location
    def transferBoid(self, boid, toID):
        print("Location " + str(self.locationID) + " sent boid " + str(boid.boidID) + 
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
        # print "This location has the following location neighbours: " + (" ".join([str(l) for l in self.neighbouringLocations]))

        # Need the slice operation or else updating 
        self.neighbouringBoids = self.boids[:]

        for locationIndex in self.neighbouringLocations:
            if locationIndex != 0:
                self.neighbouringBoids += self.simulation.getLocationBoids(locationIndex)

        return self.neighbouringBoids



# The main application class. Sets up the simulation area and the number of locations that it is to 
# be divided into. 
#
# Note that the simulation begins with a user pressing the pauseButton (initially labelled 'Begin'). 
# Because the pause flag is initally set to true, in the action function the simulation is 'resumed'.
class Simulation:

    def __init__(self):
        # Setup the simulation parameters
        self.width = 700
        self.height = 700
        self.boidCount = 90
        self.pauseSimulation = True
        self.timeStepCounter = 0

        # Create the window
        self.root = Tk()
        self.root.wm_title("Boid Simulation")

        frame = Frame(self.root)
        frame.pack()
        
        self.canvas = Canvas(frame, bg = "black", width = self.width, height = self.height)
        self.canvas.pack();
        
        # Create the buttons
        self.timeButton = Button(frame, text = "Next Time Step", command = self.nextStepButton)
        self.timeButton.pack(side = LEFT)
        self.pauseButton = Button(frame, text = "Begin", command = self.pause)
        self.pauseButton.pack(side = LEFT)
        self.quitButton = Button(frame, text = "Quit", command = frame.quit)
        self.quitButton.pack(side = LEFT)

        self.counterLabel = Label(frame, text = self.timeStepCounter, width = 6)
        self.counterLabel.pack(side = RIGHT)

        # Needed so that the canvas sizes can be used later
        self.root.update()

        # Create the locations
        self.allowedLocationCounts = np.array([1, 2, 4, 9, 16, 25, 36])
        self.locationCount = self.allowedLocationCounts[3]
        self.initialBoidCount = self.boidCount / self.locationCount
        self.locationSize = round(self.canvas.winfo_width() / np.sqrt(self.locationCount))

        self.locations = []

        self.locationColours = ["red", "blue", "green", "yellow", "white", "magenta", "dark grey", "cyan", "dark green"]

        self.locationCoords = np.array([0, 0, 0, 0])
        for i in range(0, self.locationCount):
            self.locationCoords[0] = (i % 3) * self.locationSize
            self.locationCoords[1] = int(np.floor(i / 3)) * self.locationSize
            self.locationCoords[2] = self.locationCoords[0] + self.locationSize
            self.locationCoords[3] = self.locationCoords[1] + self.locationSize

            print str(self.locationCoords)

            loc = Location(self.canvas, self, i + 1, self.locationCoords, self.initialBoidCount, self.locationColours[i])
            self.locations.append(loc)

        print "======= Press the 'Begin' button to start the simulation"

        # Start everything going
        self.root.mainloop()


    def simulationStep(self):
        if self.pauseSimulation == False:
            for i in range(0, self.locationCount):
                # print "Calculating next boid positions for location " + str(self.locations[i].locationID) + "..."
                self.locations[i].update(False)

            print "Location boid counts: " + " ".join(str(loc.boidCount) for loc in self.locations)
            print

            for i in range(0, self.locationCount):
                # print "Moving boids to calculated positions for location " + str(self.locations[i].locationID) + "..."
                self.locations[i].update(True)

            # Update the counter label
            self.timeStepCounter += 1
            self.counterLabel.config(text = self.timeStepCounter)

            # Call self after 100ms
            self.canvas.after(100, self.simulationStep)

    
    # Manual timestep increment
    def nextStepButton(self):
        self.pauseSimulation = False
        self.simulationStep()
        self.pauseSimulation = True


    # Sets a flag that is used to pause and resume the simulation 
    def pause(self):
        if self.pauseSimulation == False:
            self.pauseSimulation = True
            self.pauseButton.config(text = "Resume")
        else:
            self.pauseSimulation = False
            self.pauseButton.config(text = "Pause")
            self.simulationStep()


    # Get the neighbouring locations of the specified location. Currently, this simply returns a 
    # hard-coded list of neighbours tailored to the asking location. Ideally, the neighbours would 
    # be calculated in a programmatic way.
    def getNeighbouringLocations(self, locationID):
        if locationID == 1:
            self.neighbouringLocations = [0, 0, 0, 2, 5, 4, 0, 0]
            # self.neighbouringLocations = [2, 3, 4, 5, 6, 7, 8, 9]
        elif locationID == 2:
            self.neighbouringLocations = [0, 0, 0, 3, 6, 5, 4, 1]
            # self.neighbouringLocations = [1, 3, 4, 5, 6, 7, 8, 9]
        elif locationID == 3:
            self.neighbouringLocations = [0, 0, 0, 0, 0, 6, 5, 2]
            # self.neighbouringLocations = [2, 1, 4, 5, 6, 7, 8, 9]
        elif locationID == 4:
            self.neighbouringLocations = [0, 1, 2, 5, 8, 7, 0, 0]
            # self.neighbouringLocations = [2, 3, 1, 5, 6, 7, 8, 9]
        elif locationID == 5:
            self.neighbouringLocations = [1, 2, 3, 6, 9, 8, 7, 4]
            # self.neighbouringLocations = [2, 3, 4, 1, 6, 7, 8, 9]
        elif locationID == 6:
            self.neighbouringLocations = [2, 3, 0, 0, 0, 9, 8, 5]
            # self.neighbouringLocations = [2, 3, 4, 5, 1, 7, 8, 9]
        elif locationID == 7:
            self.neighbouringLocations = [0, 4, 5, 8, 0, 0, 0, 0]
            # self.neighbouringLocations = [2, 3, 4, 5, 6, 1, 8, 9]
        elif locationID == 8:
            self.neighbouringLocations = [4, 5, 6, 9, 0, 0, 0, 7]
            # self.neighbouringLocations = [2, 3, 4, 5, 6, 7, 1, 9]
        elif locationID == 9:
            self.neighbouringLocations = [5, 6, 0, 0, 0, 0, 0, 8]
            # self.neighbouringLocations = [2, 3, 4, 5, 6, 7, 8, 1]

        return self.neighbouringLocations


    # Return a list of the boids for a specified location
    def getLocationBoids(self, locationID):
        return self.locations[locationID - 1].getBoids()


    # Transfer a boid from one location to another
    def transferBoid(self, boid, toID, fromID):
        self.locations[toID - 1].acceptBoid(boid, fromID)


# Start everything off
boidSimulation = Simulation()


if __name__ == '__main__':
    print "yolo";

#!/usr/bin/python
# -*- coding: utf-8 -*-


from boid import Boid               # Import the Boid class

import numpy as np                  # Used in various mathematical operations
import logging                      # Used to handle the textual output
import random                       # Used to randomly position the boids on initialisation
import copy                         # Used to copy lists to avoid passing by reference


# A class representing a BoidCPU. A BoidCPU is an entity that controls all the 
# boids that enter its section of the simulation space. When boids exit a 
# BoidCPU's space, they are transfered to a neighbouring BoidCPU.
class BoidCPU:

    def __init__(self, boidGPU, _simulation, _BOIDCPU_ID, _boidCPUCoords, _initialBoidCount, _gridPosition):
        self.simulation = _simulation
        self.BOIDCPU_ID = _BOIDCPU_ID
        self.boidCPUCoords = np.copy(_boidCPUCoords)
        self.logger = self.simulation.logger
        self.gridPosition = _gridPosition
        self.boidGPU = boidGPU

        self.boidCPUAtMinimalSize = False

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
        self.boidGPU.drawBoidCPU(self.boidCPUCoords, self.BOIDCPU_ID, self.colour)

        # If testing, use a known boid setup, else use a random setup
        if self.config['useTestingSetup']:
            # Get the boid details from the test state
            for boidInfo in self.config['testState'][self.BOIDCPU_ID - 1]:
                position = np.array(boidInfo[1])
                velocity = np.array(boidInfo[2])

                # Create the boid and add to list
                boid = Boid(self.boidGPU, self, boidInfo[0], position, velocity, self.colour, False)
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
                BOID_ID = ((self.BOIDCPU_ID - 1) * self.boidCount) + i + 1

                # Create the boid and add to boid list
                boid = Boid(self.boidGPU, self, BOID_ID, position, velocity, self.colour, False)
                self.boids.append(boid)

        self.logger.info("Created BoidCPU #" + str(self.BOIDCPU_ID) + " with " + 
            str(self.boidCount) + " boids") 


    # Calculate the next positions of every boid in the BoidCPU. Then determine if any of the boids 
    # need to be transferred to neighbouring boidCPUs based on their new positions. 
    def update(self):
        for boid in self.boids:
            # self.logger.debug("Boid #" + str(self.boids[i].BOID_ID) + ":  OLD position = " + str(self.boids[i].position) + ", OLD velocity = " + str(self.boids[i].velocity))
            boid.update()
            # self.logger.debug("Boid #" + str(self.boids[i].BOID_ID) + ":  NEW position = " + str(self.boids[i].position) + ", NEW velocity = " + str(self.boids[i].velocity))
  
        # # If the number of boids in the boidCPU are greater than a threshold, signal controller
        # if self.config['loadBalance']:
        #     if self.boidCount >= self.config['BOID_THRESHOLD']:
        #         self.loadBalance()

        # Determine if the new positions of the boids lie outside the boidCPU's bounds
        # Do not perform this step here if load balancing is performed
        # if not self.config['loadBalance']:
        #     for boid in self.boids:
        #         self.determineBoidTransfer(boid)
                

    # Draw the new positions of the boids.
    def draw(self):
        for i in range(0, self.boidCount):
            self.boids[i].draw(self.colour)


    # Return a list containing the boids currently controlled by this BoidCPU
    def getBoids(self):
        copyOfBoids = []
        for boid in self.boids:
            copyOfBoids.append(boid.copy())

        return copyOfBoids


    # Calculate the neighbouring boids for each boid
    def calculateBoidNeighbours(self):
        # Get the list of neighbouring BoidCPUs
        self.neighbouringBoidCPUs = self.simulation.getNeighbouringBoidCPUs(self.BOIDCPU_ID)

        # Initialise with the boids from the current BoidCPU
        self.possibleNeighbouringBoids = self.getBoids()

        # Get the boids from the neighbouring BoidCPUs
        for boidCPUIndex in self.neighbouringBoidCPUs:
            if boidCPUIndex != 0:
                self.possibleNeighbouringBoids += self.simulation.getBoidCPUBoids(boidCPUIndex)

        # Calculate the neighbouring boids for each boid
        for boid in self.boids:
            boid.calculateNeighbours(self.possibleNeighbouringBoids)


    # Used to return the state of the initial setup of the boids for testing purposes
    def saveState(self):
        self.savedState = []

        for boid in self.boids:
            boidState = [boid.BOID_ID, boid.position.tolist(), boid.velocity.tolist()]
            self.savedState.append(boidState)

        # print self.savedState
        return self.savedState


    ################################################################################################
    ## BoidCPU to BoidCPU Boid Transfer Functions ------------------------------------------------##
    ################################################################################################

    def determineBoidTransfer(self, boid):
        # If the boidCPU has a neighbour to the NORTHWEST and the boid is beyond its northern AND western boundaries
        if (self.neighbouringBoidCPUs[0] != 0) and (boid.position[1] < self.boidCPUCoords[1]) and (boid.position[0] < self.boidCPUCoords[0]):
            self.logger.debug("\tBoid " + str(boid.BOID_ID) + 
                " is beyond the NORTH and WESTERN boundaries of boidCPU " + str(self.BOIDCPU_ID))
            self.transferBoid(boid, self.neighbouringBoidCPUs[0])

        # If the boidCPU has a neighbour to the NORTHEAST and the boid is beyond its northern AND eastern boundaries
        elif (self.neighbouringBoidCPUs[2] != 0) and (boid.position[1] < self.boidCPUCoords[1]) and (boid.position[0] > self.boidCPUCoords[2]):
            self.logger.debug("\tBoid " + str(boid.BOID_ID) + 
                " is beyond the NORTH and EASTERN boundaries of boidCPU " + str(self.BOIDCPU_ID))
            self.transferBoid(boid, self.neighbouringBoidCPUs[2])

        # If the boidCPU has a neighbour to the SOUTHEAST and the boid is beyond its southern AND eastern boundaries
        elif (self.neighbouringBoidCPUs[4] != 0) and (boid.position[1] > self.boidCPUCoords[3]) and (boid.position[0] > self.boidCPUCoords[2]):
            self.logger.debug("\tBoid " + str(boid.BOID_ID) + 
                " is beyond the SOUTHERN and EASTERN boundaries of boidCPU " + str(self.BOIDCPU_ID))
            self.transferBoid(boid, self.neighbouringBoidCPUs[4])

        # If the boidCPU has a neighbour to the SOUTHWEST and the boid is beyond its southern AND western boundaries
        elif (self.neighbouringBoidCPUs[6] != 0) and (boid.position[1] > self.boidCPUCoords[3]) and (boid.position[0] < self.boidCPUCoords[0]):
            self.logger.debug("\tBoid " + str(boid.BOID_ID) + 
                " is beyond the SOUTHERN and WESTERN boundaries of boidCPU " + str(self.BOIDCPU_ID))
            self.transferBoid(boid, self.neighbouringBoidCPUs[6])

        # If the boidCPU has a neighbour to the NORTH and the boid is beyond its northern boundary
        elif (self.neighbouringBoidCPUs[1] != 0) and (boid.position[1] < self.boidCPUCoords[1]):
            self.logger.debug("\tBoid " + str(boid.BOID_ID) + 
                " is beyond the NORTH boundary of boidCPU " + str(self.BOIDCPU_ID))
            self.transferBoid(boid, self.neighbouringBoidCPUs[1])

        # If the boidCPU has a neighbour to the EAST and the boid is beyond its eastern boundary
        elif (self.neighbouringBoidCPUs[3] != 0) and (boid.position[0] > self.boidCPUCoords[2]):
            self.logger.debug("\tBoid " + str(boid.BOID_ID) + 
                " is beyond the EAST boundary of boidCPU " + str(self.BOIDCPU_ID))
            self.transferBoid(boid, self.neighbouringBoidCPUs[3])  
            
        # If the boidCPU has a neighbour to the SOUTH and the boid is beyond its southern boundary
        elif (self.neighbouringBoidCPUs[5] != 0) and (boid.position[1] > self.boidCPUCoords[3]):
            self.logger.debug("\tBoid " + str(boid.BOID_ID) + 
                " is beyond the SOUTH boundary of boidCPU " + str(self.BOIDCPU_ID))
            self.transferBoid(boid, self.neighbouringBoidCPUs[5])

        # If the boidCPU has a neighbour to the WEST and the boid is beyond its western boundary
        elif (self.neighbouringBoidCPUs[7] != 0) and (boid.position[0] < self.boidCPUCoords[0]):
            self.logger.debug("\tBoid " + str(boid.BOID_ID) + 
                " is beyond the WEST boundary of boidCPU " + str(self.BOIDCPU_ID))
            self.transferBoid(boid, self.neighbouringBoidCPUs[7])


    # Accept a boid transferred from another boidCPU and add it to this boidCPUs boid list
    def acceptBoid(self, boid, fromID):
        boid.boidCPU = self

        self.boids.append(boid)
        self.boidCount += 1

        self.logger.debug("\tBoidCPU " + str(self.BOIDCPU_ID) + " accepted boid " + str(boid.BOID_ID) 
            + " from boidCPU " + str(fromID) + " and now has " + str(self.boidCount) + " boids")


    # Transfer a boid from this boidCPU to another boidCPU
    def transferBoid(self, boid, toID):
        self.logger.debug("\tBoidCPU " + str(self.BOIDCPU_ID) + " sent boid " + str(boid.BOID_ID) + 
            " to boidCPU " + str(toID) + " and now has " + str(self.boidCount - 1) + " boids")

        self.simulation.transferBoid(boid, toID, self.BOIDCPU_ID)

        # Remove the transfered boid from the current BoidCPUs boid list
        self.boids[:] = [b for b in self.boids if b.BOID_ID != boid.BOID_ID]
        self.boidCount -= 1


    ################################################################################################
    ## Load Balancing Functions ------------------------------------------------------------------##
    ################################################################################################

    def loadBalance(self):
        if self.boidCount >= self.config['BOID_THRESHOLD']:
            # Analyse the distribution of the boids in this BoidCPU to determine the step
            if (self.config['loadBalanceType'] == 2) or (self.config['loadBalanceType'] == 3): 
                if not self.boidCPUAtMinimalSize:
                    self.createBoidDistribution()
                    requestedChange = self.analyseBoidDistribution()
                else:
                    self.logger.debug("BoidCPU #" + str(self.BOIDCPU_ID) + " at minimal size")
            elif self.config['loadBalanceType'] == 1: 
                requestedChange = None

            self.simulation.boidCPUOverloaded(self.BOIDCPU_ID, requestedChange)


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

        self.logger.debug("BoidCPU #" + str(self.BOIDCPU_ID) + " width = " + str(boidCPUWidth) + ", height = " + str(boidCPUHeight))
        self.logger.debug("BoidCPU #" + str(self.BOIDCPU_ID) + " coords: " + str(self.boidCPUCoords))

        # Draw a grid on for the BoidCPU
        self.boidGPU.drawBoidCPUGrid(self.boidCPUCoords, widthSegments, heightSegments)

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


    # Returns the coordinate index that would be affected by a change in the given edge
    def edgeToCoord(self, edgeID):
        if edgeID > 3: self.logger.warning("Edge ID is greater than 3")

        return (edgeID + 1) % 4


    # Returns the appropriate row or col of the distribution depending on the edge specified
    def getDistributionRowCol(self, edgeID, bound):
        distHeight = self.distribution.shape[0]
        distWidth = self.distribution.shape[1]

        if edgeID == 0:
            return np.sum(self.distribution[bound, :])
        elif edgeID == 1:
            return np.sum(self.distribution[:, distWidth - 1 - bound])
        elif edgeID == 2:
            return np.sum(self.distribution[distHeight - 1 - bound, :])
        elif edgeID == 3:
            return np.sum(self.distribution[:, bound])


    # Analyses the distribution of boids for the current BoidCPU to determine which edges should be 
    # modified, and by how much, to reduce the number of boids in the overloaded BoidCPU.
    #
    # Whilst there are still boids that need to be released and the BoidCPU is not at its minimal 
    # size, for each edge, if the edge is valid for this BoidCPU and if boids still need to be 
    # released check that an additional step change for the edge doesn't make the BoidCPU 
    # smaller than its minimum. If not, get the number of boids released for this step 
    # change and update the temporary coordinates and other structures. 
    # 
    # FIXME: The issue with this new algorithm is that it can result in a toing and froing motion. 
    #   So BoidCPU #2 is overloaded and decreases its bottom edge to compensate. BoidCPU #4 is 
    #   overloaded soon after. Decreasing its top edge released no boids, decreasing its right edge 
    #   releases enough boids to no longer be overloaded. So the change it makes is to decrease the 
    #   top and right edges. Decreasing the top edge now causes BoidCPU #2 to to overloaded again - 
    #   the top edge needn't have been decreased. Therefore, don't move zero edges.
    # 
    # TODO: Do not recalculate rows/columns that have not changed since the last loop. Take into 
    #   account that if the top edge is decreased, the left and right edges will need recalculating.
    def analyseBoidDistribution(self):
        currentChanges = [0, 0, 0, 0]                   # The number of step changes per border
        boidsReleasedPerEdge = [0, 0, 0, 0]             # The number of boids released per edge
        recalculate = [True, True, True, True]          # Used to save recalculation if not needed
        tmpCoords = copy.deepcopy(self.boidCPUCoords)   # Make a copy of the BoidCPU coordinates
        numberOfBoidsReleased = 0                       # The number of boids currently released
        
        # Initialise to be True for edges that cannot be changed e.g. top edge of BoidCPU #1
        edgeAtMinimum = [True if not self.validEdge(edge) else False for edge in range(0, 4) ]

        while (numberOfBoidsReleased < self.config['boidsToRelease']) and not self.boidCPUAtMinimalSize:
            for edge in range(0, 4):
                if self.validEdge(edge) and (numberOfBoidsReleased < self.config['boidsToRelease']):
                    proposedChange = currentChanges[edge] + 1

                    if self.minSizeEnforced(edge, proposedChange, tmpCoords):
                        boidsReleasedPerEdge[edge] = self.getDistributionRowCol(edge, proposedChange - 1)

                        coordIdx = self.edgeToCoord(edge)
                        tmpCoords = self.moveEdge(edge, True, proposedChange, tmpCoords)

                        currentChanges[edge] += 1   # FIXME: Seems to work without this, strange...
                        numberOfBoidsReleased += boidsReleasedPerEdge[edge]
                    else:
                        edgeAtMinimum[edge] = True

            # If all the edges for the BoidCPU are at their minimum, raise a flag and exit.
            if all(edgeAtMinimum):
                self.boidCPUAtMinimalSize = True 
                self.logger.debug("Minimum size reached for BoidCPU #" + str(self.BOIDCPU_ID))

        # Remove the BoidGPU grid lines
        self.boidGPU.removeObject("gridLines")

        return currentChanges


    # Used to check that the proposed change doesn't violate the minimum size of the BoidCPU. The 
    # minimum BoidCPU size is defined as the boid vision radius. This ensures that a boid's 
    # neighbours would always be in an adjacent BoidCPU and not one that is further away.
    def minSizeEnforced(self, edgeID, proposedChange, tmpCoords):
        change = (self.config['stepSize'] * proposedChange)
        result = False

        # If the edge to change is the top or the bottom edge, check the proposed BoidCPU height
        if (edgeID == 0) or (edgeID == 2):
            size = (tmpCoords[3] - tmpCoords[1]) - change
            changeType = "height"

        # If the edge to change is the right or the left edge, check the propsed BoidCPU width
        elif (edgeID == 1) or (edgeID == 3):
            size = (tmpCoords[2] - tmpCoords[0]) - change
            changeType = "width"

        # Log the change 
        self.logger.debug("(Edge " + str(edgeID) + ") Change of " + str(change) + " gives new " + 
            changeType + " of BoidCPU #" + str(self.BOIDCPU_ID) + " as " + str(size))

        # Check if the new width/height is less than the constraint
        if size < self.config['minBoidCPUSize']:
            result = False      # The new size is too small
            self.logger.debug("Requested change of " + str(proposedChange) + " for edge " + 
                str(edgeID) + " for BoidCPU #" + str(self.BOIDCPU_ID) + " rejected")
        else:
            result = True       # The new size is ok

        return result


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

            self.logger.debug("\tRequest to change " + t + " of BoidCPU " + str(self.BOIDCPU_ID) + 
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
        tempBounds = copy.deepcopy(self.boidCPUCoords)
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

        self.logger.debug("\tBoidCPU " + str(self.BOIDCPU_ID) + " would have " + str(counter) + 
            " boids (" + str(self.boidCount) + " before)")

        return [len(self.boids), counter]


    def moveEdge(self, edgeID, decrease, steps, coords):
        change = self.config['stepSize'] * steps
        coordsIdx = self.edgeToCoord(edgeID)

        if edgeID == 0:     # Top edge
            if decrease:
                coords[coordsIdx] += change
            else:
                coords[coordsIdx] -= change

        elif edgeID == 1:   # Right edge
            if decrease:
                coords[coordsIdx] -= change
            else:
                coords[coordsIdx] += change

        elif edgeID == 2:   # Bottom edge
            if decrease:
                coords[coordsIdx] -= change
            else:
                coords[coordsIdx] += change

        elif edgeID == 3:   # Left edge
            if decrease:
                coords[coordsIdx] += change
            else:
                coords[coordsIdx] -= change

        return coords


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
                self.logger.debug("\tIncreased the top edge of BoidCPU " + str(self.BOIDCPU_ID) + 
                    " by " + str(stepChange))

            elif edgeType == 2:       # Bottom
                self.boidCPUCoords[3] -= stepChange
                self.logger.debug("\tDecreased the bottom edge of BoidCPU " + str(self.BOIDCPU_ID) + 
                    " by " + str(stepChange))
        else:
            if edgeType == 0:         # Top
                self.boidCPUCoords[1] -= stepChange
                self.logger.debug("\tDecreased the top edge of BoidCPU " + str(self.BOIDCPU_ID) + 
                    " by " + str(stepChange))

            elif edgeType == 2:       # Bottom
                self.boidCPUCoords[3] += stepChange
                self.logger.debug("\tIncreased the bottom edge of BoidCPU " + str(self.BOIDCPU_ID) + 
                    " by " + str(stepChange))

        if col == self.gridPosition[1]:
            if edgeType == 1:         # Right
                self.boidCPUCoords[2] -= stepChange
                self.logger.debug("\tDecreased the right edge of BoidCPU " + str(self.BOIDCPU_ID) + 
                    " by " + str(stepChange))

            elif edgeType == 3:       # Left
                self.boidCPUCoords[0] += stepChange
                self.logger.debug("\tIncreased the left edge of BoidCPU " + str(self.BOIDCPU_ID) + 
                    " by " + str(stepChange))

        else:
            if edgeType == 1:         # Right
                self.boidCPUCoords[2] += stepChange
                self.logger.debug("\tIncreased the right edge of BoidCPU " + str(self.BOIDCPU_ID) + 
                    " by " + str(stepChange))

            elif edgeType == 3:       # Left
                self.boidCPUCoords[0] -= stepChange
                self.logger.debug("\tDecreased the left edge of BoidCPU " + str(self.BOIDCPU_ID) + 
                    " by " + str(stepChange))


        # Assume that the boid is no longer at its minimal size when its bounds are changed
        self.boidCPUAtMinimalSize = False

        self.boidGPU.updateBoidCPU(self.BOIDCPU_ID, self.boidCPUCoords)


        # boidCPUWidth = self.boidCPUCoords[2] - self.boidCPUCoords[0]
        # boidCPUHeight = self.boidCPUCoords[3] - self.boidCPUCoords[1]

        # self.logger.debug("New BoidCPU #" + str(self.BOIDCPU_ID) + " width = " + str(boidCPUWidth) + ", height = " + str(boidCPUHeight))
        # self.logger.debug("New BoidCPU #" + str(self.BOIDCPU_ID) + " coords: " + str(self.boidCPUCoords))


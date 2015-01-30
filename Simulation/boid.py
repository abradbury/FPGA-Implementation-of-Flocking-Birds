#!/usr/bin/python
# -*- coding: utf-8 -*-

# To ignore numpy errors and docstrings:
#     pylint: disable=E1101
#     pylint: disable=C0111

import numpy as np                  # Used in various mathematical operations
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

        self.acceleration = np.array([0, 0], dtype=self.config['dataType'])

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
    def calculateNeighbours(self, possible_neighbours):
        self.neighbouringBoids = []
        for boid in possible_neighbours:
            if boid.BOID_ID != self.BOID_ID:
                dist = self.distance_between_two_points(self.position, boid.position)
                if dist < self.config['VISION_RADIUS']:
                    self.neighbouringBoids.append(boid)


    ################################################################################################
    ## Boid Update Routines ----------------------------------------------------------------------##
    ################################################################################################

    # Update the boid's position based on the information contained in its neighbourhood. Only
    # commit the changes when all the boids have calculated their next move. Otherwise, a boid
    # would move based on its neighbours and when one of its neighbouts comes to move it will use
    # the new position of the original boid, not its original position.
    def update(self):
        # If the boid has not already been processed, calculate its next position.
        # A boid would already be processed if it was transferred to another BoidCPU
        if not self.processed:
            # If tracking a boid, highlight its neighbouring boids
            if self.config['trackBoid'] and (self.BOID_ID == self.config['boidToTrack']):
                for boid in self.neighbouringBoids:
                    self.boidGPU.highlightBoid(True, boid.BOID_ID)

            # If the boids has neighbours, calculate its next position
            if len(self.neighbouringBoids) > 0:
                self.apply_behaviours()

            # Calculate the new position of the boid
            self.calc_new_pos()

            # Contain the boids in the simulation area
            self.contain_boids()

            self.processed = True


    def calc_new_pos(self):
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
        pos_copy = np.copy(self.position)
        vel_copy = np.copy(self.velocity)

        boid_copy = Boid(self.boidGPU, self.boidCPU, self.BOID_ID, pos_copy, vel_copy, None, True)
        return boid_copy


    # Mass could optionally be added here to get different behaviours
    def apply_force(self, force):
        self.acceleration += force


    def apply_behaviours(self):
        sep = self.separate()
        ali = self.align()
        coh = self.cohesion()

        sep *= self.config['REPULSION_WEIGHT']
        ali *= self.config['ALIGNMENT_WEIGHT']
        coh *= self.config['COHESION_WEIGHT']

        self.apply_force(sep)
        self.apply_force(ali)
        self.apply_force(coh)


    # Move the boid to the calculated positon
    def draw(self, colour):
        self.boidGPU.updateBoid(self.position, self.velocity, colour, self.BOID_ID)

        self.processed = False  # Reset the processed flag for the next simulation step


    ################################################################################################
    ## Boid Steering Rules -----------------------------------------------------------------------##
    ################################################################################################

    # A boid will align itself with the average orientation of its neighbours
    def align(self):
        total = np.array([0, 0], dtype=self.config['dataType'])

        for boid in self.neighbouringBoids:
            total += boid.velocity

        total = self.vector_divide(total, len(self.neighbouringBoids))
        total = self.set_mag(total, self.config['MAX_VELOCITY'])

        steer = total - self.velocity
        steer = self.limit(steer, self.config['MAX_FORCE'])

        return steer


    # A boid does not want to get too close to its neighbours and needs its personal space
    #
    # TODO: Scale it depending on how close the boid is
    def separate(self):
        total = np.array([0, 0], dtype=self.config['dataType'])

        for boid in self.neighbouringBoids:
            diff = self.position - boid.position        #  TODO: Check subtraction is right
            diff = self.normalise(diff)
            total += diff

        total = self.vector_divide(total, len(self.neighbouringBoids))
        total = self.set_mag(total, self.config['MAX_VELOCITY'])

        steer = total - self.velocity
        steer = self.limit(steer, self.config['MAX_FORCE'])

        return steer

    # A boid will move towards the centre of mass of its neighbourhood
    def cohesion(self):
        total = np.array([0, 0], dtype=self.config['dataType'])

        for boid in self.neighbouringBoids:
            total += boid.position

        total = self.vector_divide(total, len(self.neighbouringBoids))
        steer = self.seek(total)

        return steer


    ################################################################################################
    ## Other Boid Movement Rules -----------------------------------------------------------------##
    ################################################################################################

    # Calculates the desired velocity to the target at the maximum allowed speed. Uses this to
    # calculate a steering force to the target which is applied to the boid.
    def seek(self, target):
        desired = target - self.position
        desired = self.set_mag(desired, self.config['MAX_VELOCITY'])

        # Calculate the steering force
        steer = desired - self.velocity

        # Limit the size of the steering force
        steer = self.limit(steer, self.config['MAX_FORCE'])

        return steer


    # Contain the boids within the simulation area
    def contain_boids(self):
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
        #     d = self.distance_from_point_to_line([0, 0], [self.config['width'], 0], self.position)
        #     repulsionForce = repulsionFactor / (d ** 2)
        #     self.velocity[1] += repulsionForce
        # # Bottom edge
        # elif self.position[1] > self.config['width'] - self.config['VISION_RADIUS']:
        #     d = (self.distance_from_point_to_line([0, self.config['width']], \
        #       [self.config['width'], self.config['width']], self.position))
        #     repulsionForce = repulsionFactor / (d ** 2)
        #     self.velocity[1] -= repulsionForce
        # # Left edge
        # elif self.position[0] < 0 + self.config['VISION_RADIUS']:
        #     d = self.distance_from_point_to_line([0, 0], [0, self.config['width']], self.position)
        #     repulsionForce = repulsionFactor / (d ** 2)
        #     self.velocity[0] += repulsionForce
        # # Right edge
        # elif self.position[0] > self.config['width'] - self.config['VISION_RADIUS']:
        #     d = self.distance_from_point_to_line([self.config['width'], 0], \
        #       self.config['width']], self.position)
        #     repulsionForce = repulsionFactor / (d ** 2)
        #     self.velocity[0] -= repulsionForce


    ################################################################################################
    ## Vector operations -------------------------------------------------------------------------##
    ################################################################################################

    # Used to normalise a 2D or 3D vector
    def normalise(self, vector):
        magnetude = self.absolute(vector)

        if magnetude:
            result = self.vector_divide(vector, magnetude)
        else:
            result = 0

        return result


    # Integer division in C++ truncates towards 0, so -7/10 = 0 whereas in Python, this would equal
    # -1. To ensure the same behaviour, the vector components are cast as a float here in python
    # and the result of the division is cast back to an integer.
    #
    # TODO: Verify that float operation matches C++ behaviour
    #
    # http://stackoverflow.com/a/3602857/1433614
    # http://stackoverflow.com/a/19919450/1433614
    def vector_divide(self, vector, n):
        if self.config['dataType'] == np.int_:
            return np.array([int(float(i) / n) for i in vector], dtype=np.int_)
        elif self.config['dataType'] == np.float_:
            return vector / n


    # Calculate the magnetude or absolute value of the given vector
    def absolute(self, vector):
        if self.config['dataType'] == np.int_:
            return int(round(math.sqrt(sum([i ** 2 for i in vector]))))
        elif self.config['dataType'] == np.float_:
            return round(math.sqrt(sum([i ** 2 for i in vector])))


    # Set the magnetude of a vector to the given value
    def set_mag(self, vector, value):
        result = self.normalise(vector)
        result *= value
        return result


    # Limit a vector to the maximum allowed
    def limit(self, vector, maximum):
        if self.absolute(vector) > maximum:
            vector = self.normalise(vector) * maximum
        return vector


    # TODO: Adjust to handle any length vector
    def distance_between_two_points(self, point_a, point_b):
        if len(point_a) == 2:
            result = math.sqrt(((point_a[0] - point_b[0]) ** 2) + ((point_a[1] - point_b[1]) ** 2))
        elif len(point_a) == 3:
            result = math.sqrt(((point_a[0] - point_b[0]) ** 2) + ((point_a[1] - point_b[1]) ** 2) \
                + ((point_a[2] - point_b[2]) ** 2))
        return result


    # Calculates the shortest distance from a boid to a boundary line
    # From http://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
    def distance_from_point_to_line(self, line_start, line_end, point):
        part_one = ((line_end[1] - line_start[1]) * point[0]) - ((line_end[0] - line_start[0]) * \
                  point[1]) + (line_end[0] * line_start[1]) - (line_end[1] * line_start[0])

        part_two = ((line_end[1] - line_start[1]) ** 2) + ((line_end[0] - line_start[0]) ** 2)

        result = abs(part_one) / np.sqrt(part_two)

        return result


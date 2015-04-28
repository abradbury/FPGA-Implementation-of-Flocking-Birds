#!/usr/bin/python
# -*- coding: utf-8 -*-

# To ignore numpy errors and docstrings:
#     pylint: disable=E1101
#     pylint: disable=C0111

from boid import Boid               # Import the Boid class

import numpy as np                  # Used in various mathematical operations
import random                       # Used to randomly position the boids on initialisation
import copy                         # Used to copy lists to avoid passing by reference
import decimal as dec


# A class representing a BoidCPU. A BoidCPU is an entity that controls all the
# boids that enter its section of the simulation space. When boids exit a
# BoidCPU's space, they are transfered to a neighbouring BoidCPU.
class BoidCPU(object):

    def __init__(self, boidgpu, _sim, _boidcpu_id, _boidcpu_coords, _init_boid_count, _grid_pos):
        self.simulation = _sim
        self.boidcpu_id = _boidcpu_id
        self.boidcpu_coords = np.copy(_boidcpu_coords)
        self.logger = self.simulation.logger
        self.grid_position = _grid_pos
        self.boidgpu = boidgpu

        # True when the BoidCPU is at its minimal size
        self.boidcpu_minimal = False

        # Make the system configuration list available
        self.config = self.simulation.config

        # Define boid structures
        self.boids = []
        self.initial_boid_count = _init_boid_count
        self.boid_count = self.initial_boid_count

        # Used to hold the plotting data
        self.x_data = []
        self.y_data = []
        self.y2_data = []

        # If the boidCPU bounds are to be coloured differently, generate a random colour
        # From: http://stackoverflow.com/a/14019260
        if self.config['colourCode']:
            rnd = lambda: random.randint(0, 255)
            self.colour = str('#%02X%02X%02X' % (rnd(), rnd(), rnd()))
        else:
            self.colour = "yellow"

        # Draw the boidCPU bounds
        self.boidgpu.draw_boidcpu(self.boidcpu_coords, self.boidcpu_id, self.colour)

        # If testing, use a known boid setup, else use a random setup
        if self.config['useTestingSetup']:
            # Get the boid details from the test state
            for boid_info in self.config['testState'][self.boidcpu_id - 1]:
                if self.config['dataType'] == np.object_:
                    dec_pos = [dec.Decimal(str(i)) for i in boid_info[1]]
                    dec_vel = [dec.Decimal(str(i)) for i in boid_info[2]]

                    pos = np.array(dec_pos, dtype=self.config['dataType'])
                    vel = np.array(dec_vel, dtype=self.config['dataType'])
                else:
                    pos = np.array(boid_info[1], dtype=self.config['dataType'])
                    vel = np.array(boid_info[2], dtype=self.config['dataType'])

                # Create the boid and add to list
                boid = Boid(self.boidgpu, self, boid_info[0], pos, vel, self.colour, False)
                self.boids.append(boid)
        else:

            # width_step = (self.boidcpu_coords[2] - self.boidcpu_coords[0]) / self.boid_count
            # height_step = (self.boidcpu_coords[3] - self.boidcpu_coords[1]) / self.boid_count

            for i in range(0, self.boid_count):
                # Randomly position the boid on initialisation
                pos_x = random.randint(self.boidcpu_coords[0], self.boidcpu_coords[2])
                pos_y = random.randint(self.boidcpu_coords[1], self.boidcpu_coords[3])

                # pos_x = (width_step * i) + self.boidcpu_coords[0] + 1
                # pos_y = (height_step * i) + self.boidcpu_coords[1] + 1

                # Randomly generate the boid's initial velocity
                vel_x = random.randint(-self.config['MAX_VELOCITY'], self.config['MAX_VELOCITY'])
                vel_y = random.randint(-self.config['MAX_VELOCITY'], self.config['MAX_VELOCITY'])

                # vel_x = self.config['MAX_VELOCITY']
                # vel_y = self.config['MAX_VELOCITY']

                if self.config['dataType'] == np.object_:
                    pos_x = dec.Decimal(pos_x)
                    pos_y = dec.Decimal(pos_y)
                    vel_x = dec.Decimal(vel_x)
                    vel_y = dec.Decimal(vel_y)

                position = np.array([pos_x, pos_y], dtype=self.config['dataType'])
                velocity = np.array([vel_x, vel_y], dtype=self.config['dataType'])

                # Specify the boid's ID
                boid_id = ((self.boidcpu_id - 1) * self.boid_count) + i + 1

                # Create the boid and add to boid list
                boid = Boid(self.boidgpu, self, boid_id, position, velocity, self.colour, False)
                self.boids.append(boid)

        self.logger.info("Created BoidCPU #" + str(self.boidcpu_id) + " with " + \
            str(self.boid_count) + " boids")


    # Calculate the next positions of every boid in the BoidCPU. Then determine if any of the boids
    # need to be transferred to neighbouring boidCPUs based on their new positions.
    def update(self):
        for boid in self.boids:
            boid.update()


    # Draw the new positions of the boids.
    def draw(self):
        for i in range(0, self.boid_count):
            self.boids[i].draw(self.colour)


    # Return a list containing the boids currently controlled by this BoidCPU
    def get_boids(self):
        boids_copy = []
        for boid in self.boids:
            boids_copy.append(boid.copy())

        return boids_copy


    # Calculate the neighbouring boids for each boid
    def calculate_boid_neighbours(self):
        # Get the list of neighbouring BoidCPUs
        self.neighbouring_boid_cpus = self.simulation.get_neighbouring_boidcpus(self.boidcpu_id)

        # Initialise with the boids from the current BoidCPU
        self.possible_neighbouring_boids = self.get_boids()

        # Get the boids from the neighbouring BoidCPUs
        for boid_cpu_index in self.neighbouring_boid_cpus:
            if boid_cpu_index != 0:
                self.possible_neighbouring_boids += \
                self.simulation.get_boidcpu_boids(boid_cpu_index)

        # Calculate the neighbouring boids for each boid
        for boid in self.boids:
            boid.calculate_neighbours(self.possible_neighbouring_boids)


    # Used to return the state of the initial setup of the boids for testing purposes
    def save_state(self):
        saved_state = []

        for boid in self.boids:
            boid_state = [boid.boid_id, boid.position.tolist(), boid.velocity.tolist()]
            saved_state.append(boid_state)

        return saved_state


    ################################################################################################
    ## BoidCPU to BoidCPU Boid Transfer Functions ------------------------------------------------##
    ################################################################################################

    def determine_boid_transfer(self, boid):
        if self.neighbouring_boid_cpus:
            # If the boidCPU has a neighbour to the NORTHWEST and the boid is beyond its northern AND
            # western boundaries
            if (self.neighbouring_boid_cpus[0] != 0) and (boid.position[1] < self.boidcpu_coords[1]) \
            and (boid.position[0] < self.boidcpu_coords[0]):
                self.logger.debug("\tBoid " + str(boid.boid_id) + \
                    " is beyond the NORTH and WESTERN boundaries of boidCPU " + str(self.boidcpu_id))
                self.transfer_boid(boid, self.neighbouring_boid_cpus[0])

            # If the boidCPU has a neighbour to the NORTHEAST and the boid is beyond its northern AND
            # eastern boundaries
            elif (self.neighbouring_boid_cpus[2] != 0) and (boid.position[1] < self.boidcpu_coords[1]) \
            and (boid.position[0] > self.boidcpu_coords[2]):
                self.logger.debug("\tBoid " + str(boid.boid_id) + \
                    " is beyond the NORTH and EASTERN boundaries of boidCPU " + str(self.boidcpu_id))
                self.transfer_boid(boid, self.neighbouring_boid_cpus[2])

            # If the boidCPU has a neighbour to the SOUTHEAST and the boid is beyond its southern AND
            # eastern boundaries
            elif (self.neighbouring_boid_cpus[4] != 0) and (boid.position[1] > self.boidcpu_coords[3]) \
            and (boid.position[0] > self.boidcpu_coords[2]):
                self.logger.debug("\tBoid " + str(boid.boid_id) + \
                    " is beyond the SOUTHERN and EASTERN boundaries of boidCPU " + str(self.boidcpu_id))
                self.transfer_boid(boid, self.neighbouring_boid_cpus[4])

            # If the boidCPU has a neighbour to the SOUTHWEST and the boid is beyond its southern AND
            # western boundaries
            elif (self.neighbouring_boid_cpus[6] != 0) and (boid.position[1] > self.boidcpu_coords[3]) \
            and (boid.position[0] < self.boidcpu_coords[0]):
                self.logger.debug("\tBoid " + str(boid.boid_id) + \
                    " is beyond the SOUTHERN and WESTERN boundaries of boidCPU " + str(self.boidcpu_id))
                self.transfer_boid(boid, self.neighbouring_boid_cpus[6])

            # If the boidCPU has a neighbour to the NORTH and the boid is beyond its northern boundary
            elif (self.neighbouring_boid_cpus[1] != 0) and (boid.position[1] < self.boidcpu_coords[1]):
                self.logger.debug("\tBoid " + str(boid.boid_id) + \
                    " is beyond the NORTH boundary of boidCPU " + str(self.boidcpu_id))
                self.transfer_boid(boid, self.neighbouring_boid_cpus[1])

            # If the boidCPU has a neighbour to the EAST and the boid is beyond its eastern boundary
            elif (self.neighbouring_boid_cpus[3] != 0) and (boid.position[0] > self.boidcpu_coords[2]):
                self.logger.debug("\tBoid " + str(boid.boid_id) + \
                    " is beyond the EAST boundary of boidCPU " + str(self.boidcpu_id))
                self.transfer_boid(boid, self.neighbouring_boid_cpus[3])

            # If the boidCPU has a neighbour to the SOUTH and the boid is beyond its southern boundary
            elif (self.neighbouring_boid_cpus[5] != 0) and (boid.position[1] > self.boidcpu_coords[3]):
                self.logger.debug("\tBoid " + str(boid.boid_id) + \
                    " is beyond the SOUTH boundary of boidCPU " + str(self.boidcpu_id))
                self.transfer_boid(boid, self.neighbouring_boid_cpus[5])

            # If the boidCPU has a neighbour to the WEST and the boid is beyond its western boundary
            elif (self.neighbouring_boid_cpus[7] != 0) and (boid.position[0] < self.boidcpu_coords[0]):
                self.logger.debug("\tBoid " + str(boid.boid_id) + \
                    " is beyond the WEST boundary of boidCPU " + str(self.boidcpu_id))
                self.transfer_boid(boid, self.neighbouring_boid_cpus[7])


    # Accept a boid transferred from another boidCPU and add it to this boidCPUs boid list
    def accept_boid(self, boid, from_id):
        boid.boidCPU = self

        self.boids.append(boid)
        self.boid_count += 1

        self.logger.debug("\tBoidCPU " + str(self.boidcpu_id) + " accepted boid " + \
            str(boid.boid_id) + " from boidCPU " + str(from_id) + " and now has " + \
            str(self.boid_count) + " boids")


    # Transfer a boid from this boidCPU to another boidCPU
    def transfer_boid(self, boid, to_id):
        self.logger.debug("\tBoidCPU " + str(self.boidcpu_id) + " sent boid " + str(boid.boid_id) \
            + " to boidCPU " + str(to_id) + " and now has " + str(self.boid_count - 1) + " boids")

        self.simulation.transfer_boid(boid, to_id, self.boidcpu_id)

        # Remove the transfered boid from the current BoidCPUs boid list
        self.boids[:] = [b for b in self.boids if b.boid_id != boid.boid_id]
        self.boid_count -= 1


    ################################################################################################
    ## Load Balancing Functions ------------------------------------------------------------------##
    ################################################################################################

    def load_balance(self):
        if self.boid_count >= self.config['BOID_THRESHOLD']:
            # Analyse the distribution of the boids in this BoidCPU to determine the step
            if (self.config['loadBalanceType'] == 2) or (self.config['loadBalanceType'] == 3):
                self.create_boid_distribution()
                requested_change = self.analyse_boid_distribution()
            elif self.config['loadBalanceType'] == 1:
                requested_change = None

            self.simulation.boidcpu_overloaded(self.boidcpu_id, requested_change)


    # Creates a distribution based on the position of the boids in the current BoidCPU. The
    # distribution splits the BoidCPU area in to a grid where the size of one grid square is given
    # by the step size. Requires that the BoidCPUs only resize using a multiple of step size.
    #
    # FIXME: Works in general, but seems to be slightly off due to draw/udpdate offset
    # FIXME: Come up with a better algorithm, this one is horrendous
    def create_boid_distribution(self):
        boidcpu_width = self.boidcpu_coords[2] - self.boidcpu_coords[0]
        boidcpu_height = self.boidcpu_coords[3] - self.boidcpu_coords[1]

        width_segments = boidcpu_width / self.config['stepSize']
        height_segments = boidcpu_height / self.config['stepSize']

        # Draw a grid on for the BoidCPU
        self.boidgpu.draw_boidcpu_grid(self.boidcpu_coords, width_segments, height_segments)

        # Create a multidimensional array of the based on the number of segments
        self.distribution = np.zeros((height_segments, width_segments))

        # For every boid in the BoidCPU
        for boid in self.boids:
            boid_not_placed = True
            # For each step size segment in width
            for w_step in range(width_segments):
                # If the boid is in that width segment
                w_step_value = (((w_step + 1) * self.config['stepSize']) + self.boidcpu_coords[0])
                if boid_not_placed and boid.position[0] < w_step_value:
                    # For each step size segment in height
                    for h_step in range(height_segments):
                        # If the boid is in that height segment
                        h_step_value = (((h_step + 1) * self.config['stepSize']) + \
                            self.boidcpu_coords[1])
                        if boid_not_placed and boid.position[1] < h_step_value:
                            # Add it to the distribution array
                            self.distribution[h_step, w_step] += 1
                            boid_not_placed = False


    # Returns the coordinate index that would be affected by a change in the given edge
    def edge_to_coord(self, edge_id):
        if edge_id > 3:
            self.logger.warning("Edge ID is greater than 3")

        return (edge_id + 1) % 4


    # Returns the appropriate row or col of the distribution depending on the edge specified
    def get_distribution_row_col(self, edge_id, bound):
        dist_height = self.distribution.shape[0]
        dist_width = self.distribution.shape[1]

        if edge_id == 0:
            return np.sum(self.distribution[bound, :])
        elif edge_id == 1:
            return np.sum(self.distribution[:, dist_width - 1 - bound])
        elif edge_id == 2:
            return np.sum(self.distribution[dist_height - 1 - bound, :])
        elif edge_id == 3:
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
    def analyse_boid_distribution(self):
        current_changes = [0, 0, 0, 0]                   # The number of step changes per border
        boids_released_per_edge = [0, 0, 0, 0]           # The number of boids released per edge
        tmp_coords = copy.deepcopy(self.boidcpu_coords)  # Make a copy of the BoidCPU coordinates
        no_of_boids_released = 0                         # The number of boids currently released

        # Initialise to be True for edges that cannot be changed e.g. top edge of BoidCPU #1
        edge_at_minimum = [True if not self.valid_edge(edge) else False for edge in range(0, 4)]

        while (no_of_boids_released < self.config['boidsToRelease']) and not self.boidcpu_minimal:
            for edge in range(0, 4):
                if self.valid_edge(edge) and (no_of_boids_released < self.config['boidsToRelease']):
                    proposed_change = current_changes[edge] + 1

                    if self.min_size_enforced(edge, proposed_change, tmp_coords):
                        boids_released_per_edge[edge] = \
                            self.get_distribution_row_col(edge, proposed_change - 1)

                        tmp_coords = self.move_edge(edge, True, proposed_change, tmp_coords)

                        current_changes[edge] += 1   # FIXME: Seems to work without this, strange...
                        no_of_boids_released += boids_released_per_edge[edge]
                    else:
                        edge_at_minimum[edge] = True

            # If all the edges for the BoidCPU are at their minimum, raise a flag and exit.
            if all(edge_at_minimum):
                self.boidcpu_minimal = True
                self.logger.debug("Minimum size reached for BoidCPU #" + str(self.boidcpu_id))

        # Remove the Boidgpu grid lines
        self.boidgpu.remove_object("gridLines")

        return current_changes


    # Used to check that the proposed change doesn't violate the minimum size of the BoidCPU. The
    # minimum BoidCPU size is defined as the boid vision radius. This ensures that a boid's
    # neighbours would always be in an adjacent BoidCPU and not one that is further away.
    def min_size_enforced(self, edge_id, proposed_change, tmp_coords=None):
        change = (self.config['stepSize'] * proposed_change)
        result = False
        if tmp_coords == None:
            tmp_coords = self.boidcpu_coords

        # If the edge to change is the top or the bottom edge, check the proposed BoidCPU height
        if (edge_id == 0) or (edge_id == 2):
            size = (tmp_coords[3] - tmp_coords[1]) - change
            change_type = "height"

        # If the edge to change is the right or the left edge, check the propsed BoidCPU width
        elif (edge_id == 1) or (edge_id == 3):
            size = (tmp_coords[2] - tmp_coords[0]) - change
            change_type = "width"

        # Log the change
        self.logger.debug("(Edge " + str(edge_id) + ") Change of " + str(change) + " gives new " + \
            change_type + " of BoidCPU #" + str(self.boidcpu_id) + " as " + str(size))

        # Check if the new width/height is less than the constraint
        if size < self.config['minBoidCPUSize']:
            result = False      # The new size is too small
            self.logger.debug("Requested change of " + str(proposed_change) + " for edge " + \
                str(edge_id) + " for BoidCPU #" + str(self.boidcpu_id) + " rejected")
        else:
            result = True       # The new size is ok

        return result


    # Determines if the edge represented by the given edge_id is valid for the current BoidCPU. For
    # example, a BoidCPU on the top of the simulation area would not have its top edge as valid.
    def valid_edge(self, edge_id):
        [row, col] = self.grid_position

        # Top left position
        if col == 0 and row == 0:
            valid_edges = [False, True, True, False]

        # Bottom right position
        elif (col == self.config['widthInBoidCPUs'] - 1) and \
        (row == self.config['widthInBoidCPUs'] - 1):
            valid_edges = [True, False, False, True]

        # Bottom left position
        elif col == 0 and (row == self.config['widthInBoidCPUs'] - 1):
            valid_edges = [True, True, False, False]

        # Top right position
        elif (col == self.config['widthInBoidCPUs'] - 1) and row == 0:
            valid_edges = [False, False, True, True]

         # Left column
        elif col == 0:
            valid_edges = [True, True, True, False]

        # Top row
        elif row == 0:
            valid_edges = [False, True, True, True]

        # Right column
        elif col == (self.config['widthInBoidCPUs'] - 1):
            valid_edges = [True, False, True, True]

        # Bottom row
        elif row == (self.config['widthInBoidCPUs'] - 1):
            valid_edges = [True, True, False, True]

        # Middle
        else:
            valid_edges = [True, True, True, True]

        return valid_edges[edge_id]


    # Evaluates the requested edge adjustment to determine what the state of the current BoidCPU
    # would be if the change is made. Returns the predicted number of boids.
    #
    # Gets the boids of neighbouring BoidCPUs that would be affected by the change, uses this, and
    # its own boids to calculate the number of boids the BoidCPU will have if the requested edge
    # changes are implemented by iterating over the boids and checking their position.
    #
    # An alternative would be to use a distribution, but this has to be created and maintained.
    #
    # FIXME: The reported numbers are not always correct, possibly due to a draw happening
    def evaluate_boundary_change(self, change_requests, overloaded_boidcpu_grid_pos):
        # Print out request
        for i in change_requests:
            self.logger.debug("\tRequest to change edge " + str(i[0]) + " of BoidCPU " + \
                str(self.boidcpu_id) + " by " + str(i[1]) + " step")

        # Make a list of the edges that are requested to change
        edge_changes = [False, False, False, False]
        for edge in change_requests:
            edge_changes[edge[0]] = True

        # Get a list of the neighbouring BoidCPUs that would be affected by the edge changes
        neighbour_ids = set()
        if edge_changes[0] and edge_changes[3]:                   # If top and left edges change
            neighbour_ids.add(self.neighbouring_boid_cpus[0])     # Get BoidCPU to the northwest
        if edge_changes[0] and edge_changes[1]:                   # If top and right edges change
            neighbour_ids.add(self.neighbouring_boid_cpus[2])     # Get BoidCPU to the northeast
        if edge_changes[2] and edge_changes[1]:                   # If bottom and right edges change
            neighbour_ids.add(self.neighbouring_boid_cpus[4])     # Get BoidCPU to the southeast
        if edge_changes[2] and edge_changes[3]:                   # If bottom and left edges change
            neighbour_ids.add(self.neighbouring_boid_cpus[6])     # Get BoidCPU to the southwest

        if edge_changes[0]:                                       # If top edge changes
            neighbour_ids.add(self.neighbouring_boid_cpus[1])     # Get BoidCPU to the north
        if edge_changes[1]:                                       # If right edge changes
            neighbour_ids.add(self.neighbouring_boid_cpus[3])     # Get BoidCPU to the east
        if edge_changes[2]:                                       # If bottom edge changes
            neighbour_ids.add(self.neighbouring_boid_cpus[5])     # Get BoidCPU to the south
        if edge_changes[3]:                                       # If left edge changes
            neighbour_ids.add(self.neighbouring_boid_cpus[7])     # Get BoidCPU to the west

        # Get the boids of the neighbouring BoidCPUs that would be affected by the change
        boids_to_check = np.copy(self.boids)
        for neighbour_id in neighbour_ids:
            extra_boids = self.simulation.get_boidcpu_boids(neighbour_id)
            boids_to_check = np.concatenate((boids_to_check, extra_boids))

        # Determine the new bounds, but not by changing the actual bounds
        temp_bounds = copy.deepcopy(self.boidcpu_coords)

        for edge in change_requests:
            # If this BoidCPU is on the same row as the overloaded BoidCPU, constrict edges
            if overloaded_boidcpu_grid_pos[0] == self.grid_position[0]:
                if (edge[0] == 0) or (edge[0] == 2):
                    temp_bounds = self.move_edge(edge[0], True, edge[1], temp_bounds)
            else:
                if (edge[0] == 0) or (edge[0] == 2):
                    temp_bounds = self.move_edge(edge[0], False, edge[1], temp_bounds)

            # If this BoidCPU is in the same column as the overloaded BoidCPU, constrict edges
            if overloaded_boidcpu_grid_pos[1] == self.grid_position[1]:
                if (edge[0] == 1) or (edge[0] == 3):
                    temp_bounds = self.move_edge(edge[0], True, edge[1], temp_bounds)
            else:
                if (edge[0] == 1) or (edge[0] == 3):
                    temp_bounds = self.move_edge(edge[0], False, edge[1], temp_bounds)


        # Iterate over all the boids in the current BoidCPU and those gather from affected
        # neighbouring BoidCPUs to determine how many boids lie within the new bounds.
        #
        # TODO: Check '=' with other usages, e.g. when transferring boids
        counter = 0
        for boid in boids_to_check:
            if (boid.position[0] >= temp_bounds[0]) and (boid.position[0] <= temp_bounds[2]):
                if (boid.position[1] >= temp_bounds[1]) and (boid.position[1] <= temp_bounds[3]):
                    counter += 1

        self.logger.debug("\tBoidCPU " + str(self.boidcpu_id) + " would have " + str(counter) + \
            " boids (" + str(self.boid_count) + " before)")

        return [len(self.boids), counter]


    # TODO: Rename 'decrease'
    def move_edge(self, edge_id, decrease, steps, coords):
        change = self.config['stepSize'] * steps
        coords_index = self.edge_to_coord(edge_id)

        if edge_id == 0:     # Top edge
            if decrease:
                coords[coords_index] += change
            else:
                coords[coords_index] -= change

        elif edge_id == 1:   # Right edge
            if decrease:
                coords[coords_index] -= change
            else:
                coords[coords_index] += change

        elif edge_id == 2:   # Bottom edge
            if decrease:
                coords[coords_index] -= change
            else:
                coords[coords_index] += change

        elif edge_id == 3:   # Left edge
            if decrease:
                coords[coords_index] += change
            else:
                coords[coords_index] -= change

        return coords


    # Changes the bounds of the BoidCPU by the specifed step size. Used during load balancing.
    #
    # If the BoidCPU is in the same row or column as the BoidCPU that is overloaded, then the
    # BoidCPU boundaries need changing in the opporsite way than if the BoidCPU was in a different
    # row to the BoidCPU that is overloaded
    def change_bounds(self, edge_type, steps, overloaded_boidcpu_position):
        [row, col] = overloaded_boidcpu_position
        step_change = steps * self.config['stepSize']

        if row == self.grid_position[0]:
            if edge_type == 0:         # Top
                self.boidcpu_coords[1] += step_change
                self.logger.debug("\tIncreased the top edge of BoidCPU " + str(self.boidcpu_id) + \
                    " by " + str(step_change))

            elif edge_type == 2:       # Bottom
                self.boidcpu_coords[3] -= step_change
                self.logger.debug("\tDecreased the bottom edge of BoidCPU " + str(self.boidcpu_id) \
                    + " by " + str(step_change))
        else:
            if edge_type == 0:         # Top
                self.boidcpu_coords[1] -= step_change
                self.logger.debug("\tDecreased the top edge of BoidCPU " + str(self.boidcpu_id) + \
                    " by " + str(step_change))

            elif edge_type == 2:       # Bottom
                self.boidcpu_coords[3] += step_change
                self.logger.debug("\tIncreased the bottom edge of BoidCPU " + str(self.boidcpu_id) \
                    + " by " + str(step_change))

        if col == self.grid_position[1]:
            if edge_type == 1:         # Right
                self.boidcpu_coords[2] -= step_change
                self.logger.debug("\tDecreased the right edge of BoidCPU " + str(self.boidcpu_id) \
                    + " by " + str(step_change))

            elif edge_type == 3:       # Left
                self.boidcpu_coords[0] += step_change
                self.logger.debug("\tIncreased the left edge of BoidCPU " + str(self.boidcpu_id) + \
                    " by " + str(step_change))

        else:
            if edge_type == 1:         # Right
                self.boidcpu_coords[2] += step_change
                self.logger.debug("\tIncreased the right edge of BoidCPU " + str(self.boidcpu_id) \
                    + " by " + str(step_change))

            elif edge_type == 3:       # Left
                self.boidcpu_coords[0] -= step_change
                self.logger.debug("\tDecreased the left edge of BoidCPU " + str(self.boidcpu_id) + \
                    " by " + str(step_change))


        # Assume that the boid is no longer at its minimal size when its bounds are changed
        self.boidcpu_minimal = False

        self.boidgpu.update_boidcpu(self.boidcpu_id, self.boidcpu_coords)


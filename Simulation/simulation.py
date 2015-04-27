#!/usr/bin/python
# -*- coding: utf-8 -*-

# To ignore numpy errors and docstrings:
#     pylint: disable=E1101
#     pylint: disable=C0111

from boidCPU import BoidCPU         # Import the BoidCPU class
from boidGPU import BoidGPU         # Import the BoidGPU class

import logging                      # Used to handle the textual output
import numpy as np                  # Used in various mathematical operations
import time                         # Used to time stuff
import decimal as dec               # For floating point simulation
import sys                          # For sys.exit()


## MUST DOs ========================================================================================
# FIXME: Boids don't seem to be repelling each other that much, they are on top of one another
# FIXME: Now that the boidCPUs resize, a boid's vision radius could cover a boidCPU that is not a
#         direct neighbour of the current boidCPU which the boid resides in - affecting the result.

# TODO: Speed up performance when usign fixed point arithmetic
# TODO: Modify code to handle different load balancing protocols
# TODO: Enhance load balancing algorithm 1 so that BoidCPUs can be queried on proposed change
# TODO: Implement load balancing algorithm 2 - distribution already exists

## MAY DOs =========================================================================================
# TODO: Add keybinding to capture return key and simulate button press
# TODO: Calculate a boidCPUs neighbours programmatically - rather than hardcoding (and for 1 BCPU)

# TODO: Only allow boids to see in front of them when looking at neighbours
# TODO: Change simulation size to 1080p
# TODO: Experiment with rectangular BoidCPUs
# TODO: No need to transfer boids if there is only one BoidCPU
# TODO: Move original graph routines to use new graph_data structure
# TODO: As the draw routine is called first, no need to draw in GUI setup
# TODO: Draw the legends on initialising the graphs
# TODO: Don't redraw the stackplots every time step, try to update the data sources


# Load Balancing Types =============================================================================
# 1) Simple load balancing - a BoidCPU signals to the controller that it is overloaded and the
#   controller adjusts all of the BoidCPU's valid edges by one step size.
#
# 2) When a BoidCPU detects that it is overloaded, it analyses the distribution of boids in its area
#   and uses this to request specific edge changes. The controller applies the requested changes.
#
# 3) Same as type 2 but each BoidCPU is queried about the requested change. The BoidCPUs return the
#   number of boids they expect to have after the edge change and the number of current boids. The
#   controller then determines the exact change to perform and issues the command. As an example,
#   if BoidCPU 1 is overloaded and requests a change that would cause BoidCPU 2 to be overloaded,
#   the controller would adjust the request to try and avoid overloading BoidCPU 2.


# The main application class. Sets up the simulation area and the number of boidCPUs that it is to
# be divided into.
#
# Note that the simulation begins with a user pressing the pauseButton (initially labelled 'Begin').
# Because the pause flag is initally set to true, in the action function the simulation is 'resumed'
class Simulation(object):

    def __init__(self):
        # Setup the simulation parameters
        self.pause_simulation = True
        self.time_step_counter = 0

        # Define configuration parameters
        self.config = {}

        self.config['width'] = 720                  # Define window size
        self.config['height'] = 720
        self.config['boidCount'] = 90
        self.config['updateInterval'] = 10          # The interval between update calls (ms)
        self.config['widthInBoidCPUs'] = 3          # The number of BoidCPUs spanning the area width
        self.config['dataType'] = np.float_         # Integers, floating points or fixed points
        self.config['fixed_point_precision'] = 28   # The number of digits after the decimal point
        self.config['loggingLevel'] = logging.ERROR
        self.setup_logging()

        # Debugging parameters
        self.config['colourCode'] = False           # True to colour boids based on their BoidCPU
        self.config['trackBoid'] = True             # True to track the specified boid & neighbours
        self.config['boidToTrack'] = 2              # The ID of the boid to track, 0 for all boids

        # Load balancing parameters
        self.config['loadBalance'] = False          # True to enable load balancing
        self.config['loadBalanceType'] = 1          # See notes at top of file
        self.config['BOID_THRESHOLD'] = 30          # The maximum boids a BoidCPU should contain
        self.config['stepSize'] = 20                # The step size to change the boundaries

        self.config['percentageToRemove'] = 0.15
        self.config['boidsToRelease'] = self.config['BOID_THRESHOLD'] - \
            (np.floor(self.config['BOID_THRESHOLD'] * (1 - self.config["percentageToRemove"])))

        # Boid movement parameters
        self.config['MAX_VELOCITY'] = 10
        self.config['MAX_FORCE'] = 1                # Determines how sharply a boid can turn
        self.config['VISION_RADIUS'] = 80           # Currently set to the width of a BoidGPU

        self.config['ALIGNMENT_WEIGHT'] = 1
        self.config['COHESION_WEIGHT'] = 1
        self.config['REPULSION_WEIGHT'] = 1

        self.config['minBoidCPUSize'] = self.config['VISION_RADIUS']

        # Define the BoidCPU count
        allowed_boidcpu_counts = np.array([1, 2, 4, 9, 16, 25, 36])
        self.boidcpu_count = allowed_boidcpu_counts[3]

        # Define the testing state
        self.config['useTestingSetup'] = True       # True to use a known initial setup
        self.config['testingStopPoint'] = 200       # The timestep to stop the simulation for tests
        if self.config['useTestingSetup']:
            self.configure_known_initial_setup()    # Makes the known initial setup available
        self.config["showBoidIds"] = False          # To show the boid and BoidCPU IDs
        self.config['showBoidTrace'] = False        # To show a trace of each boid's path
        self.config['graphicsHarness'] = False      # True if drawing boids from file

        # Setup the fixed point precision, if using fixed point
        if self.config['dataType'] == np.object_:
            dec.getcontext().prec = self.config['fixed_point_precision']

        # Instantiate the BoidGPU
        self.boidgpu = BoidGPU(self)
        self.boidgpu.initialise_simulation()

        # Create the BoidCPUs
        self.initial_boid_count = self.config['boidCount'] / self.boidcpu_count
        boidcpu_size = round(self.config['width'] / np.sqrt(self.boidcpu_count))

        self.boidcpus = []
        self.boidcpu_coords = np.array([0, 0, 0, 0])

        for i in range(0, self.boidcpu_count):
            # Define the BoidCPU's pixel position
            self.boidcpu_coords[0] = (i % 3) * boidcpu_size
            self.boidcpu_coords[1] = int(np.floor(i / 3)) * boidcpu_size
            self.boidcpu_coords[2] = self.boidcpu_coords[0] + boidcpu_size
            self.boidcpu_coords[3] = self.boidcpu_coords[1] + boidcpu_size

            # Define the BoidCPU's position in the grid of BoidCPUs [row, col]
            boidcpu_grid_pos = [int(np.floor(i / 3)), (i % 3)]

            # Create the BoidCPU and add to list
            loc = BoidCPU(self.boidgpu, self, i + 1, self.boidcpu_coords, self.initial_boid_count, \
                    boidcpu_grid_pos)
            self.boidcpus.append(loc)

        # Setup variables to keep track of how many BoidCPUs exceed the boid threshold
        self.violation_count = 0
        self.violation_list = []

        # Print out setup info for the simulation
        if self.config['loadBalance']:
            if self.config['loadBalanceType'] == 1:
                self.logger.info("BoidCPUs will signal when overloaded")
            elif self.config['loadBalanceType'] == 2:
                self.logger.info("BoidCPUs will analyse boid distribution to request load " +  \
                    "balancing amounts")
            elif self.config['loadBalanceType'] == 3:
                self.logger.info("BoidCPUs will evaluate requested changes")
        else:
            self.logger.info("No load balancing will be performed")

        if self.config['useTestingSetup']:
            self.logger.info("Using testing setup for initialisation")
        else:
            self.logger.info("Using random setup for initialisation")

        self.logger.info("Press the 'Begin' button to start the simulation")

        # Setup the graphs
        self.setup_graph_data(5)
        self.boidgpu.setup_graphs(self.boidcpu_count)

        # Print out the state of the boids in the simulation
        # self.save_state()

        # Start everything going
        self.boidgpu.begin_main_loop()


    # The main loop that is continually called during the simulation, though if the simulation is
    # paused, it does nothing.
    def simulation_step(self):
        if self.pause_simulation == False:
            self.violation_count = 0

            self.draw_step()
            self.calc_step()
            self.load_step()
            self.transfer_step()

            # Update the counter label
            self.time_step_counter += 1
            self.boidgpu.update_time_step_label(self.time_step_counter)

            # If testing, check to see if the simulation has reached the requested number of
            # timesteps and if it has pause the simulation and update the graphs
            if self.config['useTestingSetup']:
                if self.time_step_counter == self.config['testingStopPoint']:
                    self.pause()
                    self.boidgpu.update_graphs()

            # Call self after 20ms (50 Hz)
            self.boidgpu.next_simulation_step(self.config['updateInterval'])


    # The draw method is called first to enable a the neighbours of a boid to be highlighted
    # when a boid is being followed (during debugging). As no new boid positions have been
    # calculated, the draw function isn't called on the first time step.
    def draw_step(self):
        self.logger.debug("-"*70)

        for boidcpu in self.boidcpus:
            self.logger.debug("Drawing boids at calculated positions for BoidCPU #" + \
                str(boidcpu.boidcpu_id) + "...")

            # Update the canvas with the new boid positions
            start_time = time.clock()
            boidcpu.draw()
            end_time = time.clock()

            # Store the number of boids for later plotting
            boidcpu.y2_data.append(boidcpu.boid_count)
            data = (end_time - start_time) * 1000
            self.update_graph_data(boidcpu.boidcpu_id - 1, 4, data)


    def setup_graph_data(self, no_of_data_types):
        self.graph_data = [[[] for a in range(0, no_of_data_types)] for b in \
            range(0, self.boidcpu_count)]


    def update_graph_data(self, bcpuid, dtype, data):
        self.graph_data[bcpuid][dtype].append(data)


    def calc_step(self):
        self.logger.debug("=" + str(self.time_step_counter) + "="*65)

        # Calculate the neighbours for each boid
        for boidcpu in self.boidcpus:
            self.logger.debug("Calculating neighbours for BoidCPU #" + str(boidcpu.boidcpu_id))

            start_time = time.clock()
            boidcpu.calculate_boid_neighbours()
            end_time = time.clock()

            # Store the timing information for later plotting
            data = (end_time - start_time) * 1000
            boidcpu.x_data.append(self.time_step_counter)
            boidcpu.y_data.append(data)
            self.update_graph_data(boidcpu.boidcpu_id - 1, 0, data)

        # Update each boid
        for boidcpu in self.boidcpus:
            self.logger.debug("Calculating next boid positions for boidCPU " + \
                str(boidcpu.boidcpu_id) + "...")

            # Calculate the next boid positions
            start_time = time.clock()
            boidcpu.update()
            end_time = time.clock()

            # Store the timing information for later plotting
            data = (end_time - start_time) * 1000
            boidcpu.y_data[self.time_step_counter] += (data)
            self.update_graph_data(boidcpu.boidcpu_id - 1, 1, data)

            # If the boidCPU is exceeding the threshold, increment counter
            if boidcpu.boid_count > self.config['BOID_THRESHOLD']:
                self.violation_count += 1

        # Update the violation list
        self.violation_list.append(self.violation_count)

        self.logger.debug("BoidCPU boid counts: " + " ".join(str(boidcpu.boid_count) \
            for boidcpu in self.boidcpus))


    # If the number of boids in the boidCPU are greater than a threshold, signal controller
    def load_step(self):
        if self.config['loadBalance']:
            for boidcpu in self.boidcpus:
                start_time = time.clock()
                boidcpu.load_balance()
                end_time = time.clock()

                # Store the timing information for later plotting
                data = (end_time - start_time) * 1000
                boidcpu.y_data[self.time_step_counter] += (data)
                self.update_graph_data(boidcpu.boidcpu_id - 1, 2, data)
        else:
            for boidcpu in self.boidcpus:
                self.update_graph_data(boidcpu.boidcpu_id - 1, 2, 0)


    # Determine if the new positions of the boids are outside their BoidCPU, if they are, transfer
    # the boids to neighbouring BoidCPUs
    def transfer_step(self):
        for boidcpu in self.boidcpus:
            start_time = time.clock()

            for boid in boidcpu.boids:
                boidcpu.determine_boid_transfer(boid)

            end_time = time.clock()

            # Store the timing information for later plotting
            data = (end_time - start_time) * 1000
            boidcpu.y_data[self.time_step_counter] += (data)
            self.update_graph_data(boidcpu.boidcpu_id - 1, 3, data)


    # Get the neighbouring boidCPUs of the specified boidCPU. Currently, this simply returns a
    # hard-coded list of neighbours tailored to the asking boidCPU. Ideally, the neighbours would
    # be calculated in a programmatic way. Currently supports 9 BoidCPUs with wrap-around.
    @classmethod
    def get_neighbouring_boidcpus(cls, boidcpu_id):
        if boidcpu_id == 1:
            neighbouring_boidcpus = [9, 7, 8, 2, 5, 4, 6, 3]
        elif boidcpu_id == 2:
            neighbouring_boidcpus = [7, 8, 9, 3, 6, 5, 4, 1]
        elif boidcpu_id == 3:
            neighbouring_boidcpus = [8, 9, 7, 1, 4, 6, 5, 2]
        elif boidcpu_id == 4:
            neighbouring_boidcpus = [3, 1, 2, 5, 8, 7, 9, 6]
        elif boidcpu_id == 5:
            neighbouring_boidcpus = [1, 2, 3, 6, 9, 8, 7, 4]
        elif boidcpu_id == 6:
            neighbouring_boidcpus = [2, 3, 1, 4, 7, 9, 8, 5]
        elif boidcpu_id == 7:
            neighbouring_boidcpus = [6, 4, 5, 8, 2, 1, 3, 9]
        elif boidcpu_id == 8:
            neighbouring_boidcpus = [4, 5, 6, 9, 3, 2, 1, 7]
        elif boidcpu_id == 9:
            neighbouring_boidcpus = [5, 6, 4, 7, 1, 3, 2, 8]

        return neighbouring_boidcpus


    # Return a list of the boids for a specified boidCPU
    def get_boidcpu_boids(self, boidcpu_id):
        return self.boidcpus[boidcpu_id - 1].get_boids()


    # Transfer a boid from one boidCPU to another
    def transfer_boid(self, boid, to_id, from_id):
        self.boidcpus[to_id - 1].accept_boid(boid, from_id)


    # Manual timestep increment
    def next_step_button(self):
        self.pause_simulation = False
        self.simulation_step()
        self.pause_simulation = True


    # Sets a flag that is used to pause and resume the simulation
    def pause(self):
        if self.pause_simulation == False:
            self.pause_simulation = True
            self.boidgpu.toggle_pause_button(False)
        else:
            self.pause_simulation = False
            self.boidgpu.toggle_pause_button(True)
            self.simulation_step()


    # Change the boid aligment weighting, value is of type string
    def change_boid_alignment(self, value):
        if self.config['dataType'] == np.object_:
            self.config['ALIGNMENT_WEIGHT'] = dec.Decimal(value)
        else:
            self.config['ALIGNMENT_WEIGHT'] = float(value)
        self.logger.debug("Boid alignment changed to " + str(value))


    # Change the boid cohesion weighting
    def change_boid_cohesion(self, value):
        if self.config['dataType'] == np.object_:
            self.config['COHESION_WEIGHT'] = dec.Decimal(value)
        else:
            self.config['COHESION_WEIGHT'] = float(value)
        self.logger.debug("Boid cohesion changed to " + str(value))


    # Change the boid separation weighting
    def change_boid_separation(self, value):
        if self.config['dataType'] == np.object_:
            self.config['REPULSION_WEIGHT'] = dec.Decimal(value)
        else:
            self.config['REPULSION_WEIGHT'] = float(value)
        self.logger.debug("Boid separation changed to " + str(value))


    # Change the boid vision radius
    def change_vision_radius(self, value):
        self.config['VISION_RADIUS'] = int(value)
        self.logger.debug("Boid vision radius changed to " + str(value))


    # Change the maximum boid velocity
    def change_max_velocity(self, value):
        self.config['MAX_VELOCITY'] = int(value)
        self.logger.debug("Boid maximum velocity changed to " + str(value))


    # Change the maximum boid force
    def change_max_force(self, value):
        self.config['MAX_FORCE'] = float(value)
        self.logger.debug("Boid maximum force changed to " + str(value))

    # Toggle the track boid status
    def toggle_track_boid(self):
        self.config['trackBoid'] = not self.config['trackBoid']
        self.boidgpu.remove_object("boidCircle" + str(self.config['boidToTrack']))
        self.logger.debug("Tracking boid changed to " + str(self.config['trackBoid']))


    # Setup logging
    def setup_logging(self):
        self.logger = logging.getLogger('boidSimulation')
        self.logger.setLevel(self.config['loggingLevel'])

        self.channel = logging.StreamHandler()
        self.channel.setLevel(self.config['loggingLevel'])

        self.formatter = logging.Formatter("[%(levelname)8s] --- %(message)s")
        self.channel.setFormatter(self.formatter)
        self.logger.addHandler(self.channel)


    ################################################################################################
    ## Load Balancing Functions ------------------------------------------------------------------##
    ################################################################################################

    # Called when a BoidCPU is overloaded with boids. If the BoidCPU supplies a requested load
    # balancing option for each of its valid edges then these are used outright. If the BoidCPU
    # does not supply a request, then a step size of 1 is assumed for all of the BoidCPU's valid
    # edges.
    #
    # The other BoidCPUs that will be affected by the changes are identified and then the edges of
    # all the affected BoidCPUs are adjusted in the requested manner.
    #
    # TODO:  Utilise better method for determining change after querying affected BoidCPUs
    # FIXME: Cannot currently handle when multiple boidCPUs are overloaded
    def boidcpu_overloaded(self, boidcpu_id, requested_change):
        self.logger.debug("BoidCPU #" + str(boidcpu_id) + " is overloaded")
        allow_change = True

        # Determine the row and column of the boidCPU that is requesting load balancing
        [row, col] = self.boidcpus[boidcpu_id - 1].grid_position

        # Determine which other BoidCPUs would be affected by this change
        [boidcpus_to_change, alt] = self.identify_affected_boidcpus(boidcpu_id, row, col, \
            requested_change)

        # Query the affected BoidCPUs to determine what the effect of the change would be
        if self.config['loadBalanceType'] == 3:
            new_boid_counts = []
            old_boid_counts = []
            for boidcpu_index, boidcpu_edges in enumerate(alt):
                [old, new] = self.boidcpus[boidcpu_index].evaluate_boundary_change(boidcpu_edges, \
                    [row, col])
                new_boid_counts.append(new)
                old_boid_counts.append(old)

            self.logger.debug("Old boid counts: " + str(old_boid_counts) + " - total: " + \
                str(sum(old_boid_counts)))
            self.logger.debug("New boid counts: " + str(new_boid_counts) + " - total: " + \
                str(sum(new_boid_counts)) + " (possible calculation error)")

            # If, after the proposed change, any BoidCPUs contain 110% of the boid threshold then
            # do not make the proposed change
            if sum(i > (self.config['BOID_THRESHOLD'] * 1.1) for i in new_boid_counts):
                allow_change = False
                self.logger.debug("Requested change not allowed: overloads other BoidCPU")

        if allow_change:
            # Update the edges of the affected BoidCPUs by the specified step size
            for edge_index, edge_boidcpus in enumerate(boidcpus_to_change):
                for boidcpu in edge_boidcpus:
                    self.boidcpus[boidcpu[0] - 1].change_bounds(edge_index, boidcpu[1], [row, col])


            for boidcpu in self.boidcpus:
                boidcpu_width = boidcpu.boidcpu_coords[2] - boidcpu.boidcpu_coords[0]
                boidcpu_height = boidcpu.boidcpu_coords[3] - boidcpu.boidcpu_coords[1]

                self.logger.debug("New BoidCPU #" + str(boidcpu.boidcpu_id) + " width = " + \
                    str(boidcpu_width) + ", height = " + str(boidcpu_height))


    # Prints out the requested edge changes
    def print_requested_bound_change(self, boidcpu_id, requested_change):
        string_start = "BoidCPU #" + str(boidcpu_id) + " requests changing "
        string_parts = []

        for edge_id, change in enumerate(requested_change):
            string_parts.append("edge " + str(edge_id) + " by " + str(change) + " steps")

        self.logger.debug(string_start + ", ".join(string_parts))


    # Based on the position of the boidCPU in the simulation grid, determine which of the sides of
    # the boidCPU would need changing (sides on simulation edge cannot be changed)
    def identify_affected_boidcpus(self, boidcpu_id, row, col, requested_change):
        # If the BoidCPU doesn't specify a request for changing the edge, use 1 step for all valid
        # edges and check that the new size is greater than the minimum needed
        if not requested_change:
            requested_change = [0, 0, 0, 0]
            change = 1

            # If the BoidCPU has a top edge and if change within constraints, change top edge
            if self.boidcpus[boidcpu_id - 1].valid_edge(0):
                if self.boidcpus[boidcpu_id - 1].min_size_enforced(0, change):
                    requested_change[0] = change

            # If the BoidCPU has a right edge and if change within constraints, change right edge
            if self.boidcpus[boidcpu_id - 1].valid_edge(1):
                if self.boidcpus[boidcpu_id - 1].min_size_enforced(1, change):
                    requested_change[1] = change

            # If the BoidCPU has a bottom edge and if change within constraints, change bottom edge
            if self.boidcpus[boidcpu_id - 1].valid_edge(2):
                if self.boidcpus[boidcpu_id - 1].min_size_enforced(2, change):
                    requested_change[2] = change

            # If the BoidCPU has a left edge and if change within constraints, change left edge
            if self.boidcpus[boidcpu_id - 1].valid_edge(3):
                if self.boidcpus[boidcpu_id - 1].min_size_enforced(3, change):
                    requested_change[3] = change

        self.print_requested_bound_change(boidcpu_id, requested_change)

        # Create a list that is true if a change is requested for that edge and false otherwise
        edge_changes = [True if v else False for v in requested_change]

        # A structure to hold the BoidCPU edges to change. Used as follows:
        #  boidcpus_to_change[edge_index][boidCPUs][index 0 = boidcpu_id, index 1 = stepChange]
        boidcpus_to_change = [[], [], [], []]

        boidcpus_to_change_b = [[] for b in range(self.boidcpu_count)]

        # Iterate over the boidCPUs to determine the IDs of those boidCPUs that would be affected
        # by the requested change and which edges of the boidCPUs would need changing
        for boidcpu in self.boidcpus:
            if edge_changes[0] and (boidcpu.grid_position[0] == row - 1):
                boidcpus_to_change[2].append([boidcpu.boidcpu_id, requested_change[0]])
                boidcpus_to_change_b[boidcpu.boidcpu_id - 1].append([2, requested_change[0]])

            if edge_changes[1] and (boidcpu.grid_position[1] == col + 1):
                boidcpus_to_change[3].append([boidcpu.boidcpu_id, requested_change[1]])
                boidcpus_to_change_b[boidcpu.boidcpu_id - 1].append([3, requested_change[1]])

            if edge_changes[2] and (boidcpu.grid_position[0] == row + 1):
                boidcpus_to_change[0].append([boidcpu.boidcpu_id, requested_change[2]])
                boidcpus_to_change_b[boidcpu.boidcpu_id - 1].append([0, requested_change[2]])

            if edge_changes[3] and (boidcpu.grid_position[1] == col - 1):
                boidcpus_to_change[1].append([boidcpu.boidcpu_id, requested_change[3]])
                boidcpus_to_change_b[boidcpu.boidcpu_id - 1].append([1, requested_change[3]])

            if edge_changes[0] and (boidcpu.grid_position[0] == row):
                boidcpus_to_change[0].append([boidcpu.boidcpu_id, requested_change[0]])
                boidcpus_to_change_b[boidcpu.boidcpu_id - 1].append([0, requested_change[0]])

            if edge_changes[1] and (boidcpu.grid_position[1] == col):
                boidcpus_to_change[1].append([boidcpu.boidcpu_id, requested_change[1]])
                boidcpus_to_change_b[boidcpu.boidcpu_id - 1].append([1, requested_change[1]])

            if edge_changes[2] and (boidcpu.grid_position[0] == row):
                boidcpus_to_change[2].append([boidcpu.boidcpu_id, requested_change[2]])
                boidcpus_to_change_b[boidcpu.boidcpu_id - 1].append([2, requested_change[2]])

            if edge_changes[3] and (boidcpu.grid_position[1] == col):
                boidcpus_to_change[3].append([boidcpu.boidcpu_id, requested_change[3]])
                boidcpus_to_change_b[boidcpu.boidcpu_id - 1].append([3, requested_change[3]])

        return [boidcpus_to_change, boidcpus_to_change_b]


    ################################################################################################
    ## Testing Functions -------------------------------------------------------------------------##
    ################################################################################################

    # Used to print out a string to the commandline which can be used to recreate the starting
    # conditions of the simulation.
    def save_state(self):
        saved_state = [0 for i in range(0, self.boidcpu_count)]
        for boidcpu in self.boidcpus:
            saved_state[boidcpu.boidcpu_id - 1] = boidcpu.save_state()
        print saved_state


    # Makes the known test state available in the configuration list
    def configure_known_initial_setup(self):
        # Single BoidCPU with 10 boids
        # self.config['testState'] = ([[[51, [12, 11], [5, 0]], \
        #     [52, [19, 35], [-5, 1]],  \
        #     [53, [12, 31], [-4, -2]], \
        #     [54, [35, 22], [0, -3]], \
        #     [55, [4, 9], [-1, 0]], \
        #     [56, [19, 18], [2, -3]], \
        #     [57, [38, 19], [4, -4]], \
        #     [58, [18, 5], [-1, 2]], \
        #     [59, [15, 33], [2, -2]], \
        #     [60, [3, 8], [-2, 0]]]])

        if (self.boidcpu_count == 1) and (self.config['boidCount'] == 90):
            self.config['testState'] = ( \
                [[[1, [106.0, 5.0], [2.0, -5.0]], [2, [227.0, 76.0], [-6.0, 2.0]], \
                [3, [106.0, 44.0], [-7.0, 0.0]], [4, [7.0, 152.0], [-4.0, 10.0]], \
                [5, [168.0, 7.0], [1.0, 5.0]], [6, [158.0, 199.0], [10.0, -9.0]], \
                [7, [136.0, 79.0], [-1.0, 5.0]], [8, [224.0, 127.0], [6.0, -2.0]], \
                [9, [155.0, 70.0], [-4.0, -5.0]], [10, [129.0, 81.0], [0.0, 2.0]],\
                [11, [406.0, 193.0], [8.0, -3.0]], [12, [357.0, 111.0], [8.0, 4.0]], \
                [13, [293.0, 46.0], [6.0, 7.0]], [14, [476.0, 99.0], [-7.0, 0.0]], \
                [15, [294.0, 28.0], [10.0, 10.0]], [16, [271.0, 179.0], [9.0, 1.0]], \
                [17, [403.0, 194.0], [7.0, 9.0]], [18, [412.0, 110.0], [9.0, -6.0]], \
                [19, [391.0, 69.0], [-1.0, -1.0]], [20, [454.0, 115.0], [-8.0, 8.0]], \
                [21, [495.0, 129.0], [3.0, -10.0]], [22, [699.0, 156.0], [4.0, -9.0]], \
                [23, [651.0, 233.0], [7.0, 3.0]], [24, [657.0, 98.0], [-5.0, -7.0]], \
                [25, [571.0, 29.0], [-3.0, -4.0]], [26, [718.0, 25.0], [-6.0, -10.0]], \
                [27, [640.0, 173.0], [-4.0, 5.0]], [28, [634.0, 35.0], [3.0, -5.0]], \
                [29, [594.0, 193.0], [2.0, -10.0]], [30, [657.0, 151.0], [6.0, -9.0]],\
                [31, [180.0, 273.0], [10.0, 3.0]], [32, [174.0, 397.0], [-9.0, 9.0]], \
                [33, [165.0, 383.0], [-7.0, 3.0]], [34, [224.0, 329.0], [0.0, 6.0]], \
                [35, [13.0, 267.0], [8.0, -10.0]], [36, [235.0, 361.0], [-9.0, 9.0]], \
                [37, [192.0, 283.0], [1.0, -6.0]], [38, [176.0, 478.0], [7.0, -2.0]], \
                [39, [53.0, 314.0], [1.0, -4.0]], [40, [123.0, 484.0], [-1.0, 1.0]], \
                [41, [283.0, 390.0], [8.0, 8.0]], [42, [478.0, 382.0], [7.0, 7.0]], \
                [43, [335.0, 267.0], [-3.0, 4.0]], [44, [338.0, 244.0], [-3.0, -2.0]], \
                [45, [479.0, 313.0], [4.0, -1.0]], [46, [337.0, 288.0], [7.0, 1.0]], \
                [47, [324.0, 455.0], [5.0, -2.0]], [48, [272.0, 389.0], [-9.0, 10.0]], \
                [49, [356.0, 270.0], [-2.0, -5.0]], [50, [263.0, 362.0], [-6.0, 7.0]],\
                [51, [695.0, 252.0], [-5.0, -9.0]], [52, [594.0, 404.0], [-10.0, -1.0]], \
                [53, [550.0, 350.0], [-10.0, -3.0]], [54, [661.0, 446.0], [-6.0, -4.0]], \
                [55, [539.0, 283.0], [-8.0, -2.0]], [56, [551.0, 256.0], [-5.0, 7.0]], \
                [57, [644.0, 342.0], [-1.0, -7.0]], [58, [592.0, 399.0], [-9.0, 6.0]], \
                [59, [644.0, 252.0], [-5.0, -8.0]], [60, [687.0, 478.0], [-9.0, 9.0]],\
                [61, [23.0, 565.0], [10.0, -9.0]], [62, [85.0, 550.0], [9.0, -1.0]], \
                [63, [115.0, 549.0], [7.0, 1.0]], [64, [2.0, 506.0], [-9.0, -10.0]], \
                [65, [50.0, 499.0], [-1.0, 3.0]], [66, [1.0, 524.0], [6.0, -9.0]], \
                [67, [66.0, 613.0], [-10.0, -4.0]], [68, [171.0, 520.0], [5.0, -8.0]], \
                [69, [117.0, 516.0], [4.0, 0.0]], [70, [49.0, 637.0], [-6.0, 6.0]], \
                [71, [387.0, 605.0], [5.0, -6.0]], [72, [322.0, 655.0], [4.0, 8.0]], \
                [73, [342.0, 498.0], [-6.0, -1.0]], [74, [392.0, 577.0], [3.0, -2.0]], \
                [75, [468.0, 534.0], [-4.0, 8.0]], [76, [331.0, 529.0], [5.0, -7.0]], \
                [77, [348.0, 608.0], [-9.0, 6.0]], [78, [455.0, 705.0], [-1.0, -1.0]], \
                [79, [404.0, 653.0], [10.0, -5.0]], [80, [246.0, 505.0], [-1.0, -3.0]],\
                [81, [501.0, 678.0], [5.0, 0.0]], [82, [711.0, 681.0], [-7.0, 5.0]], \
                [83, [638.0, 622.0], [-4.0, -9.0]], [84, [649.0, 628.0], [9.0, -4.0]], \
                [85, [710.0, 722.0], [-10.0, -9.0]], [86, [510.0, 603.0], [10.0, 7.0]], \
                [87, [569.0, 695.0], [4.0, 10.0]], [88, [718.0, 520.0], [-4.0, 10.0]], \
                [89, [726.0, 717.0], [4.0, 8.0]], [90, [713.0, 573.0], [-3.0, 5.0]]]])

                # [[[1, [76.0, 290.0], [-7.0, -3.0]], [2, [440.0, 315.0], [-10.0, 1.0]], \
                # [3, [675.0, 502.0], [10.0, 9.0]], [4, [689.0, 484.0], [-10.0, -3.0]], \
                # [5, [129.0, 499.0], [4.0, 4.0]], [6, [652.0, 592.0], [1.0, 3.0]], \
                # [7, [515.0, 525.0], [-2.0, 8.0]], [8, [100.0, 353.0], [5.0, -3.0]], \
                # [9, [720.0, 256.0], [2.0, 3.0]], [10, [201.0, 463.0], [-5.0, 10.0]], \
                # [11, [647.0, 473.0], [-1.0, 10.0]], [12, [9.0, 657.0], [-7.0, 4.0]], \
                # [13, [509.0, 661.0], [3.0, -1.0]], [14, [539.0, 637.0], [-1.0, 4.0]], \
                # [15, [173.0, 189.0], [6.0, 5.0]], [16, [227.0, 390.0], [-2.0, 1.0]], \
                # [17, [592.0, 472.0], [10.0, -10.0]], [18, [120.0, 495.0], [-4.0, 1.0]], \
                # [19, [340.0, 306.0], [8.0, 2.0]], [20, [336.0, 114.0], [-5.0, 5.0]], \
                # [21, [487.0, 486.0], [-6.0, 1.0]], [22, [129.0, 195.0], [10.0, 4.0]], \
                # [23, [260.0, 654.0], [-1.0, 5.0]], [24, [64.0, 555.0], [3.0, -3.0]], \
                # [25, [485.0, 388.0], [-2.0, 9.0]], [26, [492.0, 545.0], [8.0, -5.0]], \
                # [27, [206.0, 410.0], [6.0, 5.0]], [28, [466.0, 314.0], [-4.0, -2.0]], \
                # [29, [274.0, 684.0], [1.0, -9.0]], [30, [62.0, 685.0], [6.0, -3.0]], \
                # [31, [213.0, 188.0], [-2.0, 1.0]], [32, [82.0, 712.0], [-10.0, -3.0]], \
                # [33, [699.0, 402.0], [10.0, -1.0]], [34, [549.0, 165.0], [-4.0, 6.0]], \
                # [35, [683.0, 426.0], [7.0, 3.0]], [36, [471.0, 274.0], [8.0, -2.0]], \
                # [37, [242.0, 167.0], [-5.0, 7.0]], [38, [285.0, 371.0], [-2.0, -1.0]], \
                # [39, [396.0, 345.0], [-9.0, 0.0]], [40, [314.0, 534.0], [8.0, -8.0]], \
                # [41, [231.0, 638.0], [-8.0, 3.0]], [42, [119.0, 235.0], [9.0, 1.0]], \
                # [43, [583.0, 230.0], [-7.0, -3.0]], [44, [67.0, 209.0], [-1.0, 6.0]], \
                # [45, [449.0, 622.0], [-4.0, -5.0]], [46, [191.0, 475.0], [4.0, -5.0]], \
                # [47, [491.0, 452.0], [-7.0, 2.0]], [48, [215.0, 599.0], [8.0, -2.0]], \
                # [49, [345.0, 247.0], [8.0, 9.0]], [50, [370.0, 535.0], [8.0, 6.0]], \
                # [51, [680.0, 520.0], [-1.0, -8.0]], [52, [583.0, 673.0], [-7.0, 1.0]], \
                # [53, [128.0, 153.0], [3.0, -1.0]], [54, [316.0, 241.0], [-9.0, 2.0]], \
                # [55, [671.0, 494.0], [-8.0, 5.0]], [56, [569.0, 69.0], [-7.0, 2.0]], \
                # [57, [100.0, 42.0], [-3.0, -1.0]], [58, [327.0, 701.0], [9.0, 1.0]], \
                # [59, [228.0, 505.0], [-7.0, -8.0]], [60, [546.0, 504.0], [6.0, 9.0]], \
                # [61, [320.0, 708.0], [0.0, 7.0]], [62, [695.0, 534.0], [-2.0, -1.0]], \
                # [63, [210.0, 66.0], [-3.0, -3.0]], [64, [35.0, 242.0], [-3.0, -3.0]], \
                # [65, [202.0, 227.0], [6.0, 3.0]], [66, [157.0, 325.0], [10.0, 10.0]], \
                # [67, [31.0, 346.0], [-8.0, -10.0]], [68, [432.0, 716.0], [-6.0, 6.0]], \
                # [69, [429.0, 198.0], [3.0, -1.0]], [70, [696.0, 714.0], [4.0, -10.0]], \
                # [71, [717.0, 538.0], [7.0, -4.0]], [72, [574.0, 103.0], [-9.0, -6.0]], \
                # [73, [201.0, 274.0], [0.0, 7.0]], [74, [48.0, 4.0], [-3.0, -10.0]], \
                # [75, [147.0, 78.0], [-8.0, 9.0]], [76, [380.0, 415.0], [-10.0, -6.0]], \
                # [77, [203.0, 537.0], [6.0, 5.0]], [78, [237.0, 227.0], [-3.0, -9.0]], \
                # [79, [590.0, 686.0], [-6.0, 10.0]], [80, [455.0, 683.0], [1.0, -4.0]], \
                # [81, [604.0, 476.0], [0.0, 3.0]], [82, [86.0, 336.0], [3.0, -4.0]], \
                # [83, [175.0, 560.0], [-8.0, 9.0]], [84, [466.0, 447.0], [6.0, 0.0]], \
                # [85, [201.0, 655.0], [-9.0, 7.0]], [86, [114.0, 291.0], [-5.0, -5.0]], \
                # [87, [150.0, 110.0], [-1.0, -8.0]], [88, [372.0, 334.0], [-3.0, 10.0]], \
                # [89, [491.0, 432.0], [-3.0, 7.0]], [90, [273.0, 265.0], [1.0, -3.0]]]])

        elif self.config['boidCount'] == 9:
            self.config['testState'] = ([[[1, [106.0, 5.0], [2.0, -5.0]]], \
                [[2, [406.0, 193.0], [8.0, -3.0]]], \
                [[3, [495.0, 129.0], [3.0, -10.0]]], \
                [[4, [180.0, 273.0], [10.0, 3.0]]], \
                [[5, [283.0, 390.0], [8.0, 8.0]]], \
                [[6, [695.0, 252.0], [-5.0, -9.0]]], \
                [[7, [23.0, 565.0], [10.0, -9.0]]], \
                [[8, [387.0, 605.0], [5.0, -6.0]]], \
                [[9, [501.0, 678.0], [5.0, 0.0]]]])

        elif self.config['boidCount'] == 90:
            self.config['testState'] = ( \
                [[[1, [106.0, 5.0], [2.0, -5.0]], [2, [227.0, 76.0], [-6.0, 2.0]], \
                [3, [106.0, 44.0], [-7.0, 0.0]], [4, [7.0, 152.0], [-4.0, 10.0]], \
                [5, [168.0, 7.0], [1.0, 5.0]], [6, [158.0, 199.0], [10.0, -9.0]], \
                [7, [136.0, 79.0], [-1.0, 5.0]], [8, [224.0, 127.0], [6.0, -2.0]], \
                [9, [155.0, 70.0], [-4.0, -5.0]], [10, [129.0, 81.0], [0.0, 2.0]]],\
                [[11, [406.0, 193.0], [8.0, -3.0]], [12, [357.0, 111.0], [8.0, 4.0]], \
                [13, [293.0, 46.0], [6.0, 7.0]], [14, [476.0, 99.0], [-7.0, 0.0]], \
                [15, [294.0, 28.0], [10.0, 10.0]], [16, [271.0, 179.0], [9.0, 1.0]], \
                [17, [403.0, 194.0], [7.0, 9.0]], [18, [412.0, 110.0], [9.0, -6.0]], \
                [19, [391.0, 69.0], [-1.0, -1.0]], [20, [454.0, 115.0], [-8.0, 8.0]]], \
                [[21, [495.0, 129.0], [3.0, -10.0]], [22, [699.0, 156.0], [4.0, -9.0]], \
                [23, [651.0, 233.0], [7.0, 3.0]], [24, [657.0, 98.0], [-5.0, -7.0]], \
                [25, [571.0, 29.0], [-3.0, -4.0]], [26, [718.0, 25.0], [-6.0, -10.0]], \
                [27, [640.0, 173.0], [-4.0, 5.0]], [28, [634.0, 35.0], [3.0, -5.0]], \
                [29, [594.0, 193.0], [2.0, -10.0]], [30, [657.0, 151.0], [6.0, -9.0]]],\
                [[31, [180.0, 273.0], [10.0, 3.0]], [32, [174.0, 397.0], [-9.0, 9.0]], \
                [33, [165.0, 383.0], [-7.0, 3.0]], [34, [224.0, 329.0], [0.0, 6.0]], \
                [35, [13.0, 267.0], [8.0, -10.0]], [36, [235.0, 361.0], [-9.0, 9.0]], \
                [37, [192.0, 283.0], [1.0, -6.0]], [38, [176.0, 478.0], [7.0, -2.0]], \
                [39, [53.0, 314.0], [1.0, -4.0]], [40, [123.0, 484.0], [-1.0, 1.0]]], \
                [[41, [283.0, 390.0], [8.0, 8.0]], [42, [478.0, 382.0], [7.0, 7.0]], \
                [43, [335.0, 267.0], [-3.0, 4.0]], [44, [338.0, 244.0], [-3.0, -2.0]], \
                [45, [479.0, 313.0], [4.0, -1.0]], [46, [337.0, 288.0], [7.0, 1.0]], \
                [47, [324.0, 455.0], [5.0, -2.0]], [48, [272.0, 389.0], [-9.0, 10.0]], \
                [49, [356.0, 270.0], [-2.0, -5.0]], [50, [263.0, 362.0], [-6.0, 7.0]]],\
                [[51, [695.0, 252.0], [-5.0, -9.0]], [52, [594.0, 404.0], [-10.0, -1.0]], \
                [53, [550.0, 350.0], [-10.0, -3.0]], [54, [661.0, 446.0], [-6.0, -4.0]], \
                [55, [539.0, 283.0], [-8.0, -2.0]], [56, [551.0, 256.0], [-5.0, 7.0]], \
                [57, [644.0, 342.0], [-1.0, -7.0]], [58, [592.0, 399.0], [-9.0, 6.0]], \
                [59, [644.0, 252.0], [-5.0, -8.0]], [60, [687.0, 478.0], [-9.0, 9.0]]],\
                [[61, [23.0, 565.0], [10.0, -9.0]], [62, [85.0, 550.0], [9.0, -1.0]], \
                [63, [115.0, 549.0], [7.0, 1.0]], [64, [2.0, 506.0], [-9.0, -10.0]], \
                [65, [50.0, 499.0], [-1.0, 3.0]], [66, [1.0, 524.0], [6.0, -9.0]], \
                [67, [66.0, 613.0], [-10.0, -4.0]], [68, [171.0, 520.0], [5.0, -8.0]], \
                [69, [117.0, 516.0], [4.0, 0.0]], [70, [49.0, 637.0], [-6.0, 6.0]]], \
                [[71, [387.0, 605.0], [5.0, -6.0]], [72, [322.0, 655.0], [4.0, 8.0]], \
                [73, [342.0, 498.0], [-6.0, -1.0]], [74, [392.0, 577.0], [3.0, -2.0]], \
                [75, [468.0, 534.0], [-4.0, 8.0]], [76, [331.0, 529.0], [5.0, -7.0]], \
                [77, [348.0, 608.0], [-9.0, 6.0]], [78, [455.0, 705.0], [-1.0, -1.0]], \
                [79, [404.0, 653.0], [10.0, -5.0]], [80, [246.0, 505.0], [-1.0, -3.0]]],\
                [[81, [501.0, 678.0], [5.0, 0.0]], [82, [711.0, 681.0], [-7.0, 5.0]], \
                [83, [638.0, 622.0], [-4.0, -9.0]], [84, [649.0, 628.0], [9.0, -4.0]], \
                [85, [710.0, 722.0], [-10.0, -9.0]], [86, [510.0, 603.0], [10.0, 7.0]], \
                [87, [569.0, 695.0], [4.0, 10.0]], [88, [718.0, 520.0], [-4.0, 10.0]], \
                [89, [726.0, 717.0], [4.0, 8.0]], [90, [713.0, 573.0], [-3.0, 5.0]]]])

        elif self.config['boidCount'] == 120:
            self.config['testState'] = ( \
                [[[1, [50.0, 198.0], [8.0, 6.0]], [2, [52.0, 191.0], [-8.0, -9.0]], \
                [3, [167.0, 151.0], [-10.0, 9.0]], [4, [109.0, 99.0], [-6.0, 10.0]], \
                [5, [12.0, 208.0], [6.0, -5.0]], [6, [63.0, 185.0], [3.0, -5.0]], \
                [7, [46.0, 124.0], [6.0, 7.0]], [8, [2.0, 139.0], [6.0, -5.0]], \
                [9, [236.0, 125.0], [-2.0, -3.0]], [10, [33.0, 217.0], [-5.0, -5.0]], \
                [11, [184.0, 109.0], [1.0, -1.0]], [12, [48.0, 57.0], [-5.0, 6.0]], \
                [13, [128.0, 85.0], [4.0, 10.0]]], [[14, [269.0, 89.0], [-2.0, 5.0]], \
                [15, [316.0, 191.0], [-3.0, 10.0]], [16, [479.0, 7.0], [3.0, -8.0]], \
                [17, [251.0, 62.0], [5.0, -10.0]], [18, [443.0, 95.0], [-4.0, -6.0]], \
                [19, [415.0, 143.0], [-6.0, 9.0]], [20, [408.0, 45.0], [-5.0, -9.0]], \
                [21, [388.0, 141.0], [5.0, 10.0]], [22, [347.0, 192.0], [2.0, -7.0]], \
                [23, [414.0, 174.0], [0.0, 3.0]], [24, [351.0, 30.0], [10.0, 0.0]], \
                [25, [287.0, 46.0], [5.0, -3.0]], [26, [333.0, 182.0], [-9.0, -10.0]]], \
                [[27, [640.0, 50.0], [-2.0, 8.0]], [28, [664.0, 13.0], [7.0, 4.0]], \
                [29, [677.0, 104.0], [-9.0, 6.0]], [30, [702.0, 168.0], [2.0, 4.0]], \
                [31, [522.0, 7.0], [10.0, 0.0]], [32, [589.0, 49.0], [6.0, -7.0]], \
                [33, [596.0, 64.0], [-6.0, 10.0]], [34, [553.0, 35.0], [9.0, 1.0]], \
                [35, [499.0, 17.0], [-8.0, 0.0]], [36, [496.0, 4.0], [8.0, 3.0]], \
                [37, [563.0, 156.0], [-4.0, -7.0]], [38, [631.0, 14.0], [5.0, 4.0]], \
                [39, [644.0, 19.0], [-10.0, 1.0]]], [[40, [112.0, 265.0], [2.0, -7.0]], \
                [41, [91.0, 389.0], [-7.0, 5.0]], [42, [110.0, 324.0], [-7.0, 5.0]], \
                [43, [60.0, 475.0], [-9.0, 5.0]], [44, [3.0, 423.0], [8.0, 6.0]], \
                [45, [234.0, 262.0], [3.0, -9.0]], [46, [16.0, 406.0], [-8.0, -7.0]], \
                [47, [10.0, 284.0], [0.0, -10.0]], [48, [147.0, 418.0], [-7.0, 1.0]], \
                [49, [173.0, 293.0], [-9.0, 9.0]], [50, [217.0, 364.0], [-3.0, 9.0]], \
                [51, [171.0, 437.0], [-8.0, -7.0]], [52, [183.0, 332.0], [0.0, -2.0]]], \
                [[53, [450.0, 417.0], [-3.0, -2.0]], [54, [366.0, 381.0], [3.0, -6.0]], \
                [55, [461.0, 270.0], [-9.0, 4.0]], [56, [400.0, 252.0], [3.0, 1.0]], \
                [57, [396.0, 286.0], [-2.0, 5.0]], [58, [255.0, 439.0], [2.0, -5.0]], \
                [59, [282.0, 422.0], [-10.0, -2.0]], [60, [270.0, 437.0], [8.0, -8.0]], \
                [61, [271.0, 243.0], [4.0, 2.0]], [62, [369.0, 480.0], [6.0, 9.0]], \
                [63, [456.0, 390.0], [-7.0, -8.0]], [64, [303.0, 246.0], [-2.0, -4.0]], \
                [65, [245.0, 246.0], [0.0, -5.0]]], [[66, [575.0, 389.0], [-1.0, 2.0]], \
                [67, [708.0, 447.0], [9.0, 5.0]], [68, [635.0, 396.0], [-3.0, -5.0]], \
                [69, [638.0, 439.0], [-2.0, -3.0]], [70, [684.0, 453.0], [-3.0, -1.0]], \
                [71, [603.0, 474.0], [10.0, -9.0]], [72, [680.0, 275.0], [3.0, 7.0]], \
                [73, [593.0, 289.0], [8.0, 3.0]], [74, [667.0, 255.0], [-2.0, 7.0]], \
                [75, [716.0, 423.0], [-10.0, -9.0]], [76, [516.0, 404.0], [5.0, 7.0]], \
                [77, [582.0, 290.0], [-7.0, 6.0]], [78, [679.0, 448.0], [-9.0, -7.0]]], \
                [[79, [60.0, 534.0], [-3.0, 1.0]], [80, [54.0, 572.0], [-2.0, -7.0]], \
                [81, [118.0, 717.0], [-3.0, 8.0]], [82, [144.0, 695.0], [9.0, 1.0]], \
                [83, [71.0, 637.0], [8.0, -10.0]], [84, [221.0, 617.0], [6.0, 4.0]], \
                [85, [136.0, 491.0], [-4.0, 0.0]], [86, [28.0, 659.0], [-5.0, 1.0]], \
                [87, [102.0, 504.0], [7.0, 3.0]], [88, [186.0, 534.0], [1.0, -9.0]], \
                [89, [126.0, 649.0], [3.0, 0.0]], [90, [7.0, 563.0], [-6.0, -10.0]], \
                [91, [60.0, 588.0], [-6.0, -7.0]]], [[92, [249.0, 494.0], [-7.0, -2.0]], \
                [93, [374.0, 498.0], [-1.0, 2.0]], [94, [313.0, 528.0], [-6.0, 5.0]], \
                [95, [310.0, 561.0], [-3.0, 4.0]], [96, [398.0, 591.0], [-2.0, -8.0]], \
                [97, [462.0, 708.0], [9.0, -4.0]], [98, [247.0, 524.0], [2.0, -7.0]], \
                [99, [297.0, 504.0], [1.0, -7.0]], [100, [283.0, 492.0], [-8.0, -10.0]], \
                [101, [479.0, 664.0], [5.0, 3.0]], [102, [341.0, 671.0], [5.0, -4.0]], \
                [103, [450.0, 486.0], [4.0, 6.0]], [104, [321.0, 581.0], [9.0, 8.0]]], \
                [[105, [524.0, 643.0], [2.0, -9.0]], [106, [619.0, 641.0], [6.0, -4.0]], \
                [107, [600.0, 531.0], [8.0, 8.0]], [108, [636.0, 627.0], [-4.0, -9.0]], \
                [109, [609.0, 691.0], [9.0, 3.0]], [110, [624.0, 653.0], [-10.0, -1.0]], \
                [111, [540.0, 708.0], [10.0, 2.0]], [112, [632.0, 539.0], [-2.0, -1.0]], \
                [113, [504.0, 648.0], [-10.0, -5.0]], [114, [687.0, 622.0], [-2.0, -5.0]], \
                [115, [638.0, 713.0], [-9.0, 10.0]], [116, [590.0, 549.0], [-8.0, -4.0]], \
                [117, [516.0, 698.0], [-6.0, 8.0]]]])

        else:
            self.logger.error("No test case for defined a boid count of " + \
                str(self.config['boidCount']))
            sys.exit(1)


if __name__ == '__main__':
    # Start everything off
    BOID_SIMULATION = Simulation()

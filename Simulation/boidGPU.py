#!/usr/bin/python
# -*- coding: utf-8 -*-

# To ignore numpy errors and docstrings:
#     pylint: disable=E1101
#     pylint: disable=C0111

from Tkinter import Frame, Canvas, Label, Button, Checkbutton, Scale
from Tkinter import Tk, TOP, E, W, HORIZONTAL, BOTH

import ttk                          # Used for the tabs
import numpy as np                  # Used in various mathematical operations
import decimal as dec
import matplotlib
matplotlib.use('TkAgg')

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg
import matplotlib.pyplot as plt     # Used to plot the graphs
from matplotlib.figure import Figure

# Handles all the simulation's graphic components and output
class BoidGPU(object):

    def __init__(self, _simulation):
        # Link back to simulation
        self.simulation = _simulation

        # Make the system configuration list available
        self.config = self.simulation.config

        # Create the window
        self.root = Tk()
        self.root.wm_title("Boid Simulation")

        # Row counter
        row = 1

        # Create the master frame for the window
        master = Frame(self.root, name="master")
        master.pack(fill=BOTH)

        # Notebook tabs ---------------------------------------------------------------------------#
        # Create the tabs/notebook
        self.notebook = ttk.Notebook(master, name="notebook")
        # self.notebook.enable_traversal()
        self.notebook.pack(fill=BOTH)

        # Add the simulation frame to the notebook
        frame = Frame(self.notebook, name='simulation')
        frame.grid(row=row, rowspan=20, column=3)
        self.notebook.add(frame, text="Simulation")

        # Add the graph frame to the notebook
        self.graph_frame = Frame(self.notebook, name='graphs')
        self.notebook.add(self.graph_frame, text="BoidCPU Timing")

        # Add the graph frame to the notebook
        self.graph_frame_b = Frame(self.notebook, name='graphs2')
        self.notebook.add(self.graph_frame_b, text="Overloaded BoidCPUs")

        # Add the graph frame to the notebook
        self.graph_frame_c = Frame(self.notebook, name='graphs3')
        self.notebook.add(self.graph_frame_c, text="Timings")

        # Setup the simulation area ---------------------------------------------------------------#
        self.canvas = Canvas(frame, bg="black", width=self.config['width'], \
            height=self.config['height'])
        self.canvas.grid(row=row, rowspan=20, column=3)

        # Place the title
        self.title = Label(frame, text="Flocking Boid Simulator", \
            font="Helvetica 16 bold underline")
        self.title.grid(row=row, column=1, columnspan=2)

        # Simulation buttons ----------------------------------------------------------------------#
        row += 1
        self.time_button = Button(frame, text="Next Time Step", \
            command=self.simulation.next_step_button)
        self.time_button.grid(row=row, column=1, columnspan=2)

        row += 1
        self.pause_button = Button(frame, text="Begin Simulation", command=self.simulation.pause)
        self.pause_button.grid(row=row, column=1, columnspan=2)

        row += 1
        self.graph_button = Button(frame, text="Update Graphs", command=self.update_graphs)
        self.graph_button.grid(row=row, column=1, columnspan=2)

        # Boid sliders ----------------------------------------------------------------------------#
        # Create the boid rule weighting sliders
        row += 1
        self.heading1 = Label(frame, text="Boid Rule Weightings", \
            font="Helvetica 14 bold underline")
        self.heading1.grid(row=row, column=1, columnspan=2)

        min_rule_val = 1
        max_rule_val = 5
        resolution = 0.1

        row += 1
        self.alignment_label = Label(frame, text="Alignment: ")
        self.alignment_label.grid(row=row, column=1, sticky=E)
        self.alignment_scale = Scale(frame, from_=min_rule_val, to=max_rule_val, orient=\
            HORIZONTAL, resolution=resolution, command=self.simulation.change_boid_alignment)
        self.alignment_scale.grid(row=row, column=2, sticky=W)
        self.alignment_scale.set(self.config['ALIGNMENT_WEIGHT'])

        row += 1
        self.cohesion_label = Label(frame, text="Cohesion: ")
        self.cohesion_label.grid(row=row, column=1, sticky=E)
        self.cohesion_scale = Scale(frame, from_=min_rule_val, to=max_rule_val, orient=\
            HORIZONTAL, resolution=resolution, command=self.simulation.change_boid_cohesion)
        self.cohesion_scale.grid(row=row, column=2, sticky=W)
        self.cohesion_scale.set(self.config['COHESION_WEIGHT'])

        row += 1
        self.separation_label = Label(frame, text="Separation: ")
        self.separation_label.grid(row=row, column=1, sticky=E)
        self.separation_scale = Scale(frame, from_=min_rule_val, to=max_rule_val, orient=\
            HORIZONTAL, resolution=resolution, command=self.simulation.change_boid_separation)
        self.separation_scale.grid(row=row, column=2, sticky=W)
        self.separation_scale.set(self.config['REPULSION_WEIGHT'])

        # Other boid parameters -------------------------------------------------------------------#
        row += 1
        self.heading1 = Label(frame, text="Other Boid Parameters", \
            font="Helvetica 14 bold underline")
        self.heading1.grid(row=row, column=1, columnspan=2)

        row += 1
        min_vision_radius = 10
        max_vision_radius = 200
        resolution = self.config['stepSize']
        self.vision_radius_label = Label(frame, text="Vision Radius: ")
        self.vision_radius_label.grid(row=row, column=1, sticky=E)
        self.vision_radius = Scale(frame, from_=min_vision_radius, to=max_vision_radius, orient=\
            HORIZONTAL, resolution=resolution, command=self.simulation.change_vision_radius)
        self.vision_radius.grid(row=row, column=2, sticky=W)
        self.vision_radius.set(self.config['VISION_RADIUS'])

        row += 1
        min_velocity = 1
        max_velocity = 100
        self.max_velocity_label = Label(frame, text="Max Velocity: ")
        self.max_velocity_label.grid(row=row, column=1, sticky=E)
        self.max_velocity_scale = Scale(frame, from_=min_velocity, to=max_velocity, orient=\
            HORIZONTAL, command=self.simulation.change_max_velocity)
        self.max_velocity_scale.grid(row=row, column=2, sticky=W)
        self.max_velocity_scale.set(self.config['MAX_VELOCITY'])

        row += 1
        min_force = 0
        max_force = 2
        resolution = 0.01
        self.max_force_label = Label(frame, text="Max Force: ")
        self.max_force_label.grid(row=row, column=1, sticky=E)
        self.max_force_scale = Scale(frame, from_=min_force, to=max_force, orient=\
            HORIZONTAL, resolution=resolution, command=self.simulation.change_max_force)
        self.max_force_scale.grid(row=row, column=2, sticky=W)
        self.max_force_scale.set(self.config['MAX_FORCE'])

        row += 1
        self.track_boid_label = Label(frame, text="Track Boid: ")
        self.track_boid_label.grid(row=row, column=1, sticky=E)
        self.track_boid_check = Checkbutton(frame, onvalue=True, offvalue=False, \
            command=self.simulation.toggle_track_boid)
        self.track_boid_check.grid(row=row, column=2, sticky=W)
        if self.config['trackBoid']:
            self.track_boid_check.select()
        else:
            self.track_boid_check.deselect()

        # Add the time step counter
        row += 1
        self.counter_label = Label(frame, text="Time step: ")
        self.counter_label.grid(row=19, column=1, sticky=E)
        self.counter = Label(frame, text=0, width=6)
        self.counter.grid(row=19, column=2, sticky=W)

        # And finally add the quit button
        row += 1
        self.quit_button = Button(frame, text="Quit", command=frame.quit)
        self.quit_button.grid(row=20, column=1, columnspan=2)

        # Needed so that the canvas sizes can be used later
        self.root.update()


    ################################################################################################
    ## User Input Functions ----------------------------------------------------------------------##
    ################################################################################################

    def update_time_step_label(self, new_time_step):
        self.counter.config(text=new_time_step)


    def toggle_pause_button(self, resume):
        if resume:
            self.pause_button.config(text="Pause Simulation")
        else:
            self.pause_button.config(text="Resume Simulation")


    ################################################################################################
    ## Setup Functions ---------------------------------------------------------------------------##
    ################################################################################################

    def initialise_simulation(self):
        self.bearing = np.pi
        self.step = 10


    # Creates a boidCPU object and draws on the screen
    def draw_boidcpu(self, coords, boid_cpu_id, colour):
        self.canvas.create_rectangle(coords[0], coords[1], coords[2], coords[3], outline=colour, \
            tags="L" + str(boid_cpu_id))

        # Show BoidCPU IDs
        if self.config["showBoidIds"]:
            x_pos = coords[0] + ((coords[2] - coords[0]) / 2)
            y_pos = coords[1] + ((coords[3] - coords[1]) / 2)
            self.canvas.create_text(x_pos, y_pos, fill="white", text=str(boid_cpu_id), \
                tags=("TL" + str(boid_cpu_id)))


    # Redraws the bounds of the boidCPU
    def update_boidcpu(self, boid_cpu_id, coords):
        self.canvas.coords("L" + str(boid_cpu_id), coords[0], coords[1], coords[2], coords[3])

        # Update the BoidCPU's ID
        if self.config["showBoidIds"]:
            x_pos = coords[0] + ((coords[2] - coords[0]) / 2)
            y_pos = coords[1] + ((coords[3] - coords[1]) / 2)
            self.canvas.coords("TL" + str(boid_cpu_id), x_pos, y_pos)


    def select_tab(self, tab_id):
        self.notebook.select(tab_id)


    def begin_main_loop(self):
        # Start everything going
        self.root.mainloop()


    ################################################################################################
    ## Boid Update Functions ---------------------------------------------------------------------##
    ################################################################################################

    def create_boid(self, position, velocity, _colour, boid_id):
        points = self.calc_boid_points(position)
        points = self.rotate_boid(velocity, position, points)

        # Colour the boid based on their original boidCPU if debugging
        if self.config['colourCode']:
            colour = _colour
            outline_colour = _colour
            boid_width = 2
        else:
            colour = "red"
            outline_colour = "white"
            boid_width = 1

        # Draw the boid
        self.canvas.create_polygon(points[0], points[1], points[2], points[3], points[4], \
            points[5], points[6], points[7], fill=colour, outline=outline_colour, \
            width=boid_width, tags=("B" + str(boid_id)))

        # Show boid IDs
        if self.config["showBoidIds"]:
            self.canvas.create_text(position[0], position[1] - 15, fill="white", \
                text=str(boid_id), tags=("T" + str(boid_id)))


    def next_simulation_step(self, milliseconds):
        self.canvas.after(milliseconds, self.simulation.simulation_step)


    # Calculate the posisitions of the boid polygon points
    @classmethod
    def calc_boid_points(cls, position):
        step_size = 6
        points = []

        points.append(position[0] - step_size)
        points.append(position[1] - step_size)
        points.append(position[0])
        points.append(position[1] + step_size)
        points.append(position[0] + step_size)
        points.append(position[1] - step_size)
        points.append(position[0])
        points.append(position[1] - (step_size / 2))

        return points


    # Move the boid on the screen based on the new velocity and position
    def update_boid(self, position, velocity, _colour, boid_id):
        points = self.calc_boid_points(position)
        points = self.rotate_boid(velocity, position, points)

        # Specify the boid fill colour based on the debug flag
        if self.config['colourCode']:
            colour = _colour
        else:
            colour = "red"

        # Update boid
        self.canvas.coords("B" + str(boid_id), points[0], points[1], points[2], points[3], \
            points[4], points[5], points[6], points[7])
        self.canvas.itemconfig("B" + str(boid_id), fill=colour)

        # Update the boid's ID
        if self.config["showBoidIds"]:
            self.canvas.coords("T" + str(boid_id), position[0], position[1] - 15)

        # Debugging method - follow a specific boid
        if self.config['trackBoid'] and (boid_id == self.config['boidToTrack']):
            self.follow_boid(position, boid_id)
        # If boidToTrack is 0, track all boids
        elif self.config['trackBoid'] and not self.config['boidToTrack']:
            self.follow_boid(position, boid_id)


    # Rotate the specified boid based on its velocity / orientation
    # Rotation definition based on an answer from StackOverflow: http://stackoverflow.com/a/3409039
    def rotate_boid(self, velocity, position, points):
        if self.config['dataType'] == np.object_:
            radians = np.arctan2(float(velocity[0]), float(velocity[1]))
        else:
            radians = np.arctan2(velocity[0], velocity[1])

        # Convert to radians for trig functions
        # radians = degrees * np.pi / 180
        self.bearing -= radians
        self.bearing = self.bearing % (2 * np.pi)

        def _rot(x_val, y_val):
            # Note: the rotation is done in the opposite fashion from for a right-handed coordinate
            #   system due to the left-handedness of computer coordinates
            x_val -= position[0]
            y_val -= position[1]

            if self.config['dataType'] == np.object_:
                _x_val = x_val * dec.Decimal(str(np.cos(radians))) + y_val * \
                    dec.Decimal(str(np.sin(radians)))
                _y_val = -x_val * dec.Decimal(str(np.sin(radians))) + y_val * \
                    dec.Decimal(str(np.cos(radians)))
            else:
                _x_val = x_val * np.cos(radians) + y_val * np.sin(radians)
                _y_val = -x_val * np.sin(radians) + y_val * np.cos(radians)

            return _x_val + position[0], _y_val + position[1]

        points[0], points[1] = _rot(points[0], points[1])
        points[2], points[3] = _rot(points[2], points[3])
        points[4], points[5] = _rot(points[4], points[5])
        points[6], points[7] = _rot(points[6], points[7])

        return points


    # Follows a boid as it moves around the area. The boid has its vision circle shown and is
    # coloured blue. Any neighbouring boid is coloured green.
    def follow_boid(self, position, boid_id):
        self.canvas.delete(("boidCircle" + str(boid_id)))
        self.canvas.itemconfig("B" + str(boid_id), fill="blue")

        self.canvas.create_oval(position[0] - self.config['VISION_RADIUS'], \
            position[1] - self.config['VISION_RADIUS'], position[0] + \
            self.config['VISION_RADIUS'], position[1] + self.config['VISION_RADIUS'], \
            outline="yellow", tags=("boidCircle" + str(boid_id)))


    # Highlights or de-highlights a specific boid based on the given boid_id
    def highlight_boid(self, enable, boid_id):
        if enable:
            self.canvas.itemconfig("B" + str(boid_id), fill="green")
        else:
            self.canvas.itemconfig("B" + str(boid_id), fill="red")


    # Draws or updates a line on the screen, depending on if the tag can be found
    def draw_line(self, start_points, end_points, tag):
        handle = self.canvas.find_withtag(tag)

        if handle:
            self.canvas.coords(tag, start_points[0], start_points[1], end_points[0], end_points[1])
        else:
            self.canvas.create_line([start_points[0], start_points[1], end_points[0], \
                end_points[1]], fill="red", tags=tag)


    def draw_circle(self, centre, radius, tag):
        self.canvas.delete(tag)

        self.canvas.create_oval(centre[0] - radius, centre[1] - radius, centre[0] + radius, \
            centre[1] + radius, outline="white", fill="white", tags=tag)


    def draw_boidcpu_grid(self, boid_cpu_coords, segment_width, segment_height):
        for i in range(segment_width):
            x_val = boid_cpu_coords[0] + (i * self.config['stepSize'])
            self.canvas.create_line([x_val, boid_cpu_coords[1], x_val, boid_cpu_coords[3]], \
                fill="green", tags="gridLines")

        for i in range(segment_height):
            y_val = boid_cpu_coords[1] + (i * self.config['stepSize'])
            self.canvas.create_line([boid_cpu_coords[0], y_val, boid_cpu_coords[2], y_val], \
                fill="green", tags="gridLines")


    def remove_object(self, tag):
        self.canvas.delete(tag)


    ################################################################################################
    ## Graphical Functions -----------------------------------------------------------------------##
    ################################################################################################

    def setup_graphs(self, boid_cpu_count):
        self.setup_timing_graphs(boid_cpu_count)
        self.setup_overloaded_graph(boid_cpu_count)
        self.setup_other_graphs(boid_cpu_count)

    # Create the graphs
    def setup_timing_graphs(self, boid_cpu_count):
        y_data = []

        # Need to hold a list of lines for each graph, surely there must be a better way to do this?
        self.lines = []
        self.lines2 = []
        self.threshold_lines = []
        self.andall_lines = []

        self.axes = []
        self.axes2 = []

        self.graph_figure = Figure()

        # Add the graph to the UI tab
        canvas = FigureCanvasTkAgg(self.graph_figure, master=self.graph_frame)
        canvas.show()
        canvas.get_tk_widget().pack(side=TOP, fill=BOTH, expand=1)

        # Add the toolbar
        toolbar = NavigationToolbar2TkAgg(canvas, self.graph_frame)
        toolbar.update()
        canvas._tkcanvas.pack(side=TOP, fill=BOTH, expand=1)

        def on_key_event(event):
            key_press_handler(event, canvas, toolbar)

        canvas.mpl_connect('key_press_event', on_key_event)

        for i in range(0, boid_cpu_count):
            # Create the subplots and sync the axes
            if i != 0:
                axis = self.graph_figure.add_subplot(3, 3, i + 1, sharex=self.axes[0], \
                    sharey=self.axes[0])
            else:
                axis = self.graph_figure.add_subplot(3, 3, i + 1)

            axis.set_title("BoidCPU #%d" % (i + 1))
            axis.grid(True)

            # Setup the first y axis
            for tick_label in axis.get_yticklabels():
                tick_label.set_color('b')

            # Setup the second y axis
            axis2 = axis.twinx()
            for tick_label in axis2.get_yticklabels():
                tick_label.set_color('r')
            axis2.set_ylim([0, self.config['boidCount']])

            # Plot each line, returns a list of lines - the comma is needed
            # If the x data set is not specified, it is assumed to be the number
            # of element contained in the y data set - which is fine here.
            line1, = axis.plot(y_data, color='b')
            line2, = axis2.plot(y_data, color='r')

            # Add the Andall(sp?) and max boid threshold lines
            threshold_line, = axis2.plot([], [], color='m')
            andall_line, = axis2.plot([], [], color='g')

            # Customise the suplot grid so that axes don't overlap
            if (i % 3) != 0:
                plt.setp(axis.get_yticklabels(), visible=False)
            else:
                axis.set_ylabel("Computation time, ms", color='b')

            if (i % 3) != (3 - 1):
                plt.setp(axis2.get_yticklabels(), visible=False)
            else:
                axis2.set_ylabel("Number of boids", color='r')

            if int(np.floor(i / 3)) != 2:
                plt.setp(axis.get_xticklabels(), visible=False)
            else:
                axis.set_xlabel("Number of timesteps")

            # Add the lines to the line lists
            self.lines.append(line1)
            self.lines2.append(line2)
            self.andall_lines.append(andall_line)
            self.threshold_lines.append(threshold_line)

            # Add the axes to the axes lists
            self.axes.append(axis)
            self.axes2.append(axis2)

        # Show the subplots in interactive mode (doesn't block)
        plt.ion()
        plt.show()


    def setup_other_graphs(self, boid_cpu_count):
        # fnx = lambda : np.random.randint(5, 50, 10)
        # y = np.row_stack((fnx(), fnx(), fnx()))
        # x = np.arange(10)

        fig = Figure()
        # ax = fig.add_subplot(111)

        y_data = []
        self.olines = []
        self.olines2 = []
        # threshold_lines = []
        # andall_lines = []
        self.oaxes = []
        self.oaxes2 = []

        for i in range(0, boid_cpu_count):
            # Create the subplots and sync the axes
            if i != 0:
                axis = fig.add_subplot(3, 3, i + 1, sharex=self.oaxes[0], sharey=self.oaxes[0])
            else:
                axis = fig.add_subplot(3, 3, i + 1)

            axis.set_title("BoidCPU #%d" % (i + 1))
            axis.grid(True)

            # Setup the first y axis
            for tick_label in axis.get_yticklabels():
                tick_label.set_color('b')

            # Setup the second y axis
            axis2 = axis.twinx()
            for tick_label in axis2.get_yticklabels():
                tick_label.set_color('r')
            axis2.set_ylim([0, self.config['boidCount']])

            # Plot each line, returns a list of lines - the comma is needed
            # If the x data set is not specified, it is assumed to be the number
            # of element contained in the y data set - which is fine here.
            line1, = axis.plot(y_data, color='b')
            line2, = axis2.plot(y_data, color='r')

            # Add the Andall(sp?) and max boid threshold lines
            # threshold_line, = axis2.plot([], [], color='m')
            # andall_line, = axis2.plot([], [], color='g')

            # Customise the suplot grid so that axes don't overlap
            if (i % 3) != 0:
                plt.setp(axis.get_yticklabels(), visible=False)
            else:
                axis.set_ylabel("Computation time, ms", color='b')

            if (i % 3) != (3 - 1):
                plt.setp(axis2.get_yticklabels(), visible=False)
            else:
                axis2.set_ylabel("Number of boids", color='r')

            if int(np.floor(i / 3)) != 2:
                plt.setp(axis.get_xticklabels(), visible=False)
            else:
                axis.set_xlabel("Number of timesteps")

            # Add the lines to the line lists
            self.olines.append(line1)
            self.olines2.append(line2)
            # andall_lines.append(andall_line)
            # threshold_lines.append(threshold_line)

            # Add the axes to the axes lists
            self.oaxes.append(axis)
            self.oaxes2.append(axis2)

        # Show the subplots in interactive mode (doesn't block)
        plt.ion()
        plt.show()



        # ax.stackplot(x, y)
        # plt.show()

        canvas = FigureCanvasTkAgg(fig, master=self.graph_frame_c)
        canvas.show()
        canvas.get_tk_widget().pack(side=TOP, fill=BOTH, expand=1)

        # Add the toolbar
        toolbar = NavigationToolbar2TkAgg(canvas, self.graph_frame_c)
        toolbar.update()
        canvas._tkcanvas.pack(side=TOP, fill=BOTH, expand=1)


    # Create a sumary graph showing the number of boidCPUs exceeding the boid threshold over time
    def setup_overloaded_graph(self, boid_cpu_count):
        plt.ion()

        self.summary_figure = Figure()
        self.summary_axis = self.summary_figure.add_subplot(111)

        self.summary_axis.plot([], [])
        self.summary_axis.fill_between([], 0, [])

        canvas = FigureCanvasTkAgg(self.summary_figure, master=self.graph_frame_b)
        canvas.show()
        canvas.get_tk_widget().pack(side=TOP, fill=BOTH, expand=1)

        # Add the toolbar
        toolbar = NavigationToolbar2TkAgg(canvas, self.graph_frame_b)
        toolbar.update()
        canvas._tkcanvas.pack(side=TOP, fill=BOTH, expand=1)

        def on_key_event(event):
            key_press_handler(event, canvas, toolbar)

        canvas.mpl_connect('key_press_event', on_key_event)

        self.summary_axis.set_ylim([0, boid_cpu_count])

        self.summary_axis.set_title("Graph showing number of boidCPUs exceeding the boid threshold")
        self.summary_axis.grid(True)

        self.summary_axis.set_xlabel("Number of time steps")
        self.summary_axis.set_ylabel("BoidCPUs over threshold")

        plt.show()


    # For each boidCPU, get the graph lines and set the data to the current data
    # plus the new data. Then reformat both axes to accommodate the new data.
    def update_graphs(self):

        for i in range(0, self.simulation.boidcpu_count):
            # Update the boidCPU boid calculation time lines
            self.lines[i].set_xdata(self.simulation.boidcpus[i].x_data)
            self.lines[i].set_ydata(self.simulation.boidcpus[i].y_data)

            # Update the boidCPU boid count lines. Because the draw routine is called first, but
            # not on time step 1, it has one less data element in than the values for the update
            # stage. Therefore, [0:-1] is used to ignore the last x value.
            self.lines2[i].set_xdata(self.simulation.boidcpus[i].x_data[0:-1])
            self.lines2[i].set_ydata(self.simulation.boidcpus[i].y2_data)

            # Update the Andall (sp?) lines
            self.andall_lines[i].set_xdata([0, len(self.simulation.boidcpus[i].x_data)])
            self.andall_lines[i].set_ydata([self.simulation.initial_boid_count, \
                self.simulation.initial_boid_count])

            # Update the max boid threshold lines
            self.threshold_lines[i].set_xdata([0, len(self.simulation.boidcpus[i].x_data)])
            self.threshold_lines[i].set_ydata([self.config['BOID_THRESHOLD'], \
                self.config['BOID_THRESHOLD']])

            # Adjust the axes accordingly
            self.axes[i].relim()
            self.axes[i].autoscale_view()

            self.axes2[i].relim()
            self.axes2[i].autoscale_view()

            # Re-draw the graphs
            self.graph_figure.canvas.draw()



            # print len(self.simulation.boidcpus[i].x_data)
            # print len(self.simulation.graphData[i][0])

            # a = np.row_stack(self.simulation.graphData[i])
            # print len(a)
            # print len(self.simulation.graphData[i])

            # self.olines[i].set_xdata(self.simulation.boidcpus[i].x_data)
            # self.olines[i].set_ydata(np.row_stack(self.simulation.graphData[i]))

            # self.olines2[i].set_xdata(self.simulation.boidcpus[i].x_data[0:-1])
            # self.olines2[i].set_ydata(self.simulation.boidcpus[i].y2_data)

            # # Adjust the axes accordingly
            # self.oaxes[i].relim()
            # self.oaxes[i].autoscale_view()

            # self.oaxes2[i].relim()
            # self.oaxes2[i].autoscale_view()
            # y = np.row_stack(self.simulation.graphData[i])
            # print y
            # x = np.arange(10)

        # Update summary graph
        self.summary_axis.lines[0].set_xdata(range(0, self.simulation.time_step_counter))
        self.summary_axis.lines[0].set_ydata(self.simulation.violation_list)
        self.summary_axis.fill_between(range(0, self.simulation.time_step_counter), 0, \
            self.simulation.violation_list)

        self.summary_axis.relim()
        self.summary_axis.autoscale_view()

        self.summary_figure.canvas.draw()

        # Switch to the first graphs tab
        self.select_tab(1)


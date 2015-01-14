#!/usr/bin/python
# -*- coding: utf-8 -*-


from Tkinter import *               # Used to draw shapes for the simulation
import ttk                          # Used for the tabs
import numpy as np                  # Used in various mathematical operations
import matplotlib
matplotlib.use('TkAgg')

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg
import matplotlib.pyplot as plt     # Used to plot the graphs

# Handles all the simulation's graphic components and output
class BoidGPU:

    def __init__(self, _simulation):
        # Link back to simulation
        self.simulation = _simulation

        # Make the system configuration list available
        self.config = self.simulation.config

        # Create the window
        self.root = Tk()
        self.root.wm_title("Boid Simulation")

        # Row counter
        r = 1

        # Create the master frame for the window
        master = Frame(self.root, name = "master")
        master.pack(fill = BOTH)

        # Create the tabs/notebook
        self.notebook = ttk.Notebook(master, name = "notebook")
        self.notebook.enable_traversal()
        self.notebook.pack(fill = BOTH)

        # Add the simulation frame to the notebook
        frame = Frame(self.notebook, name = 'simulation')
        frame.grid(row = r, rowspan = 20, column = 3)
        self.notebook.add(frame, text = "Simulation")

        # Add the graph frame to the notebook
        self.graphFrame = Frame(self.notebook, name = 'graphs')
        self.notebook.add(self.graphFrame, text = "BoidCPU Timing")

        # Add the graph frame to the notebook
        self.graph2Frame = Frame(self.notebook, name = 'graphs2')
        self.notebook.add(self.graph2Frame, text = "Overloaded BoidCPUs")

        self.canvas = Canvas(frame, bg = "black", width = self.config['width'], 
            height = self.config['height'])
        self.canvas.grid(row = r, rowspan = 20, column = 3);
        
        # Place the title
        self.title = Label(frame, text = "Flocking Boid Simulator", font = "Helvetica 16 bold underline")
        self.title.grid(row = r, column = 1, columnspan = 2)

        # Create the simulation buttons
        r += 1
        self.timeButton = Button(frame, text = "Next Time Step", 
            command = self.simulation.nextStepButton)
        self.timeButton.grid(row = r, column = 1, columnspan = 2)

        r += 1
        self.pauseButton = Button(frame, text = "Begin Simulation", command = self.simulation.pause)
        self.pauseButton.grid(row = r, column = 1, columnspan = 2)

        r += 1
        self.graphButton = Button(frame, text = "Update Graphs", 
            command = self.updateGraphs)
        self.graphButton.grid(row = r, column  = 1, columnspan = 2)

        # Create the boid rule weighting sliders
        r += 1
        self.heading1 = Label(frame, text = "Boid Rule Weightings", font = "Helvetica 14 bold underline")
        self.heading1.grid(row = r, column = 1, columnspan = 2)

        minRuleValue = 1
        maxRuleValue = 5
        resolution = 0.1

        r += 1
        self.alignmentLabel = Label(frame, text = "Alignment: ")
        self.alignmentLabel.grid(row = r, column = 1, sticky = E)
        self.alignmentScale = Scale(frame, from_ = minRuleValue, to = maxRuleValue, orient = 
            HORIZONTAL, resolution = resolution, command = self.simulation.changeBoidAlignment)
        self.alignmentScale.grid(row = r, column = 2, sticky = W)
        self.alignmentScale.set(self.config['ALIGNMENT_WEIGHT'])

        r += 1
        self.cohesionLabel = Label(frame, text = "Cohesion: ")
        self.cohesionLabel.grid(row = r, column = 1, sticky = E)
        self.cohesionScale = Scale(frame, from_ = minRuleValue, to = maxRuleValue, orient = 
            HORIZONTAL, resolution = resolution, command = self.simulation.changeBoidCohesion)
        self.cohesionScale.grid(row = r, column = 2, sticky = W)
        self.cohesionScale.set(self.config['COHESION_WEIGHT'])

        r += 1
        self.separationLabel = Label(frame, text = "Separation: ")
        self.separationLabel.grid(row = r, column = 1, sticky = E)
        self.separationScale = Scale(frame, from_ = minRuleValue, to = maxRuleValue, orient = 
            HORIZONTAL, resolution = resolution, command = self.simulation.changeBoidSeparation)
        self.separationScale.grid(row = r, column = 2, sticky = W)
        self.separationScale.set(self.config['REPULSION_WEIGHT'])

        # Other boid parameters
        r += 1
        self.heading1 = Label(frame, text = "Other Boid Parameters", font = "Helvetica 14 bold underline")
        self.heading1.grid(row = r, column = 1, columnspan = 2)

        r += 1
        minVisionRadius = 10
        maxVisionRadius = 200
        resolution = self.config['stepSize']
        self.visionRadiusLabel = Label(frame, text = "Vision Radius: ")
        self.visionRadiusLabel.grid(row = r, column = 1, sticky = E)
        self.visionRadius = Scale(frame, from_ = minVisionRadius, to = maxVisionRadius, orient = 
            HORIZONTAL, resolution = resolution, command = self.simulation.changeVisionRadius)
        self.visionRadius.grid(row = r, column = 2, sticky = W)
        self.visionRadius.set(self.config['VISION_RADIUS'])

        r += 1
        minVelocity = 1
        maxVelocity = 100
        self.maxVelocityLabel = Label(frame, text = "Max Velocity: ")
        self.maxVelocityLabel.grid(row = r, column = 1, sticky = E)
        self.maxVelocityScale = Scale(frame, from_ = minVelocity, to = maxVelocity, orient = 
            HORIZONTAL, command = self.simulation.changeMaxVelocity)
        self.maxVelocityScale.grid(row = r, column = 2, sticky = W)
        self.maxVelocityScale.set(self.config['MAX_VELOCITY'])

        r += 1
        minForce = 0
        maxForce = 2
        resolution = 0.01
        self.maxForceLabel = Label(frame, text = "Max Force: ")
        self.maxForceLabel.grid(row = r, column = 1, sticky = E)
        self.maxForceScale = Scale(frame, from_ = minForce, to = maxForce, orient = 
            HORIZONTAL, resolution = resolution, command = self.simulation.changeMaxForce)
        self.maxForceScale.grid(row = r, column = 2, sticky = W)
        self.maxForceScale.set(self.config['MAX_FORCE'])

        r += 1
        self.trackBoidLabel = Label(frame, text = "Track Boid: ")
        self.trackBoidLabel.grid(row = r, column = 1, sticky = E)
        self.trackBoidCheck = Checkbutton(frame, onvalue = True, offvalue = False, command = self.simulation.toggleTrackBoid)
        self.trackBoidCheck.grid(row = r, column = 2, sticky = W)
        if self.config['trackBoid']:
            self.trackBoidCheck.select()
        else:
            self.trackBoidCheck.deselect()

        # Add the time step counter
        r += 1
        self.counterLabel = Label(frame, text = "Time step: ")
        self.counterLabel.grid(row = 19, column = 1, sticky = E)
        self.counter = Label(frame, text = 0, width = 6)
        self.counter.grid(row = 19, column = 2, sticky = W)

        # And finally add the quit button
        r += 1
        self.quitButton = Button(frame, text = "Quit", command = frame.quit)
        self.quitButton.grid(row = 20, column = 1, columnspan = 2)

        # Needed so that the canvas sizes can be used later
        self.root.update()


    ################################################################################################
    ## User Input Functions ----------------------------------------------------------------------##
    ################################################################################################

    def updateTimeStepLabel(self, newTimeStep):
        self.counter.config(text = newTimeStep)


    def togglePauseButton(self, resume):
        if resume:
            self.pauseButton.config(text = "Pause Simulation")
        else:
            self.pauseButton.config(text = "Resume Simulation")


    ################################################################################################
    ## Setup Functions ---------------------------------------------------------------------------##
    ################################################################################################

    def initialiseSimulation(self):
        self.bearing = np.pi
        self.step = 10


    # Creates a boidCPU object and draws on the screen
    def drawBoidCPU(self, coords, locID, colour):
        self.canvas.create_rectangle(coords[0], coords[1], coords[2], coords[3], outline = colour, 
            tags = "L" + str(locID))

        # Show BoidCPU IDs
        if self.config["showBoidIds"]:
            xPos = coords[0] + ((coords[2] - coords[0]) / 2)
            yPos = coords[1] + ((coords[3] - coords[1]) / 2)
            self.canvas.create_text(xPos, yPos, fill = "white", text = str(locID), 
                tags = ("TL" + str(locID)))


    # Redraws the bounds of the boidCPU
    def updateBoidCPU(self, locID, coords):
        self.canvas.coords("L" + str(locID), coords[0], coords[1], coords[2], coords[3])

        # Update the BoidCPU's ID
        if self.config["showBoidIds"]:
            xPos = coords[0] + ((coords[2] - coords[0]) / 2)
            yPos = coords[1] + ((coords[3] - coords[1]) / 2)
            self.canvas.coords("TL" + str(locID), xPos, yPos)


    def getWindowSize(self):
        return [self.canvas.winfo_width(), self.canvas.winfo_height()]


    def selectTab(self, tabID):
        self.notebook.select(tabID)


    def beginMainLoop(self):
        # Start everything going
        self.root.mainloop()


    ################################################################################################
    ## Graphical Functions -----------------------------------------------------------------------##
    ################################################################################################

    # Create the graphs
    def setupGraphs(self, boidCPUCount):
        self.yData = []

        # Need to hold a list of lines for each graph, surely there must be a better way to do this?
        self.lines = []
        self.lines2 = []
        self.thresholdLines = []
        self.andallLines = []

        self.axes = []
        self.axes2 = []

        self.graphFigure = plt.figure()

        # Add the graph to the UI tab
        canvas = FigureCanvasTkAgg(self.graphFigure, master = self.graphFrame)
        canvas.show()
        canvas.get_tk_widget().pack(side = TOP, fill = BOTH, expand=1)

        # Add the toolbar
        toolbar = NavigationToolbar2TkAgg(canvas, self.graphFrame)
        toolbar.update()
        canvas._tkcanvas.pack(side=TOP, fill=BOTH, expand=1)

        def on_key_event(event):
            key_press_handler(event, canvas, toolbar)

        canvas.mpl_connect('key_press_event', on_key_event)

        for i in range(0, boidCPUCount):
            # Create the subplots and sync the axes
            if i != 0:
                axis = self.graphFigure.add_subplot(3, 3, i + 1, sharex = self.axes[0], 
                    sharey = self.axes[0])
            else:
                axis = self.graphFigure.add_subplot(3, 3, i + 1)

            axis.set_title("BoidCPU %d" % (i + 1))
            axis.grid(True)
            
            # Setup the first y axis
            for tl in axis.get_yticklabels():
                tl.set_color('b')
    
            # Setup the second y axis
            axis2 = axis.twinx()
            for tl in axis2.get_yticklabels():
                tl.set_color('r')
            axis2.set_ylim([0, self.config['boidCount']])

            # Plot each line, returns a list of lines - the comma is needed
            # If the x data set is not specified, it is assumed to be the number 
            # of element contained in the y data set - which is fine here.
            line1, = axis.plot(self.yData, color = 'b')
            line2, = axis2.plot(self.yData, color = 'r')

            # Add the Andall(sp?) and max boid threshold lines
            thresholdLine, = axis2.plot([], [], color = 'm')
            andallLine, = axis2.plot([], [], color = 'g')
            
            # Customise the suplot grid so that axes don't overlap
            if (i % 3) != 0:
                plt.setp(axis.get_yticklabels(), visible = False)
            else:
                axis.set_ylabel("Computation time, ms", color = 'b')

            if (i % 3) != (3 - 1):
                plt.setp(axis2.get_yticklabels(), visible = False)
            else:
                axis2.set_ylabel("Number of boids", color = 'r')

            if int(np.floor(i / 3)) != 2:
                plt.setp(axis.get_xticklabels(), visible = False)
            else:
                axis.set_xlabel("Number of timesteps")

            # Add the lines to the line lists
            self.lines.append(line1)
            self.lines2.append(line2)
            self.andallLines.append(andallLine)
            self.thresholdLines.append(thresholdLine)

            # Add the axes to the axes lists
            self.axes.append(axis)
            self.axes2.append(axis2)

        # Show the subplots in interactive mode (doesn't block)
        # plt.ion()
        # plt.show()

        # plt.close()


    # Create a sumary graph showing the number of boidCPUs exceeding the boid threshold over time
    def setupSummaryGraph(self, boidCPUCount):
        plt.ion()

        self.summaryFigure, self.summaryAxis = plt.subplots(1,1)
        self.summaryAxis.plot([], [])
        self.summaryAxis.fill_between([], 0, [])

        canvas = FigureCanvasTkAgg(self.summaryFigure, master = self.graph2Frame)
        canvas.show()
        canvas.get_tk_widget().pack(side = TOP, fill = BOTH, expand=1)

        # Add the toolbar
        toolbar = NavigationToolbar2TkAgg(canvas, self.graph2Frame)
        toolbar.update()
        canvas._tkcanvas.pack(side=TOP, fill=BOTH, expand=1)

        def on_key_event(event):
            key_press_handler(event, canvas, toolbar)

        canvas.mpl_connect('key_press_event', on_key_event)

        self.summaryAxis.set_ylim([0, boidCPUCount])
        
        self.summaryAxis.set_title("Graph showing number of boidCPUs exceeding the boid threshold")
        self.summaryAxis.grid(True)

        self.summaryAxis.set_xlabel("Number of time steps")
        self.summaryAxis.set_ylabel("BoidCPUs over threshold")

        plt.show()


    # For each boidCPU, get the graph lines and set the data to the current data  
    # plus the new data. Then reformat both axes to accommodate the new data.
    def updateGraphs(self):

        for i in range(0, self.simulation.boidCPUCount):
            # Update the boidCPU boid calculation time lines
            self.lines[i].set_xdata(self.simulation.boidCPUs[i].xData)
            self.lines[i].set_ydata(self.simulation.boidCPUs[i].yData)

            # Update the boidCPU boid count lines. Because the draw routine is called first, but 
            # not on time step 1, it has one less data element in than the values for the update 
            # stage. Therefore, [0:-1] is used to ignore the last x value.
            self.lines2[i].set_xdata(self.simulation.boidCPUs[i].xData[0:-1])
            self.lines2[i].set_ydata(self.simulation.boidCPUs[i].y2Data)

            # Update the Andall (sp?) lines
            self.andallLines[i].set_xdata([0, len(self.simulation.boidCPUs[i].xData)])
            self.andallLines[i].set_ydata([self.simulation.initialBoidCount, self.simulation.initialBoidCount])

            # Update the max boid threshold lines
            self.thresholdLines[i].set_xdata([0, len(self.simulation.boidCPUs[i].xData)])
            self.thresholdLines[i].set_ydata([self.config['BOID_THRESHOLD'], self.config['BOID_THRESHOLD']])

            # Adjust the axes accordingly
            self.axes[i].relim()
            self.axes[i].autoscale_view()

            self.axes2[i].relim()
            self.axes2[i].autoscale_view()

            # Re-draw the graphs
            self.graphFigure.canvas.draw()

        # Update summary graph
        self.summaryAxis.lines[0].set_xdata(range(0, self.simulation.timeStepCounter))
        self.summaryAxis.lines[0].set_ydata(self.simulation.violationList)
        self.summaryAxis.fill_between(range(0, self.simulation.timeStepCounter), 0, self.simulation.violationList)

        self.summaryAxis.relim()
        self.summaryAxis.autoscale_view()

        self.summaryFigure.canvas.draw()

        # Switch to the graphs tab
        self.selectTab(1)


    ################################################################################################
    ## Boid Update Functions ---------------------------------------------------------------------##
    ################################################################################################

    def createBoid(self, position, velocity, _colour, BOID_ID):
        points = self.calcBoidPoints(position)
        points = self.rotateBoid(velocity, position, points)

        # Colour the boid based on their original boidCPU if debugging
        if self.config['colourCode']:
            colour = _colour
            outlineColour = _colour
            boidWidth = 2
        else:
            colour = "red"
            outlineColour = "white"
            boidWidth = 1

        # Draw the boid
        self.canvas.create_polygon(points[0], points[1], points[2], points[3], points[4], points[5], 
            points[6], points[7], fill = colour, outline = outlineColour, width = boidWidth, 
            tags = ("B" + str(BOID_ID)))

        # Show boid IDs
        if self.config["showBoidIds"]:
            self.canvas.create_text(position[0], position[1] - 15, fill = "white", text = str(BOID_ID), tags = ("T" + str(BOID_ID)))


    def nextSimulationStep(self, milliseconds):
        self.canvas.after(milliseconds, self.simulation.simulationStep)


    # Calculate the posisitions of the boid polygon points
    def calcBoidPoints(self, position):
        stepSize = 6
        points = []

        points.append(position[0] - stepSize)
        points.append(position[1] - stepSize)
        points.append(position[0])
        points.append(position[1] + stepSize)
        points.append(position[0] + stepSize)
        points.append(position[1] - stepSize)
        points.append(position[0])
        points.append(position[1] - (stepSize/2))

        return points


    # Move the boid on the screen based on the new velocity and position
    def updateBoid(self, position, velocity, _colour, BOID_ID):
        points = self.calcBoidPoints(position)
        points = self.rotateBoid(velocity, position, points)

        # Specify the boid fill colour based on the debug flag
        if self.config['colourCode']:
            colour = _colour
        else:
            colour = "red"

        # Update boid
        self.canvas.coords("B" + str(BOID_ID), points[0], points[1], points[2], points[3], points[4], 
            points[5], points[6], points[7])
        self.canvas.itemconfig("B" + str(BOID_ID), fill = colour) 

        # Update the boid's ID
        if self.config["showBoidIds"]:
            self.canvas.coords("T" + str(BOID_ID), position[0], position[1] - 15)

        # Debugging method - follow a specific boid
        if self.config['trackBoid'] and (BOID_ID == self.config['boidToTrack']):  
            self.followBoid(position, BOID_ID)
        # If boidToTrack is 0, track all boids
        elif self.config['trackBoid'] and not self.config['boidToTrack']:
            self.followBoid(position, BOID_ID)


    # Rotate the specified boid based on its velocity / orientation
    # Rotation definition based on an answer from StackOverflow: http://stackoverflow.com/a/3409039
    def rotateBoid(self, velocity, position, points):
        radians = np.arctan2(velocity[0], velocity[1])

        # Convert to radians for trig functions
        # radians = degrees * np.pi / 180
        self.bearing -= radians
        self.bearing = self.bearing % (2 * np.pi)

        def _rot(x, y):
            # Note: the rotation is done in the opposite fashion from for a right-handed coordinate 
            #   system due to the left-handedness of computer coordinates
            x -= position[0]
            y -= position[1]
            _x = x * np.cos(radians) + y * np.sin(radians)
            _y = -x * np.sin(radians) + y * np.cos(radians)
            return _x + position[0], _y + position[1]

        points[0], points[1] = _rot(points[0], points[1])
        points[2], points[3] = _rot(points[2], points[3])
        points[4], points[5] = _rot(points[4], points[5])
        points[6], points[7] = _rot(points[6], points[7])

        return points


    # Follows a boid as it moves around the area. The boid has its vision circle shown and is 
    # coloured blue. Any neighbouring boid is coloured green. 
    def followBoid(self, position, BOID_ID):
        self.canvas.delete(("boidCircle" + str(BOID_ID)))
        self.canvas.itemconfig("B" + str(BOID_ID), fill = "blue")

        self.canvas.create_oval(position[0] - self.config['VISION_RADIUS'], 
            position[1] - self.config['VISION_RADIUS'], position[0] + self.config['VISION_RADIUS'], 
            position[1] + self.config['VISION_RADIUS'], outline = "yellow", tags = ("boidCircle" + str(BOID_ID)))


    # Highlights or de-highlights a specific boid based on the given BOID_ID
    def highlightBoid(self, on, BOID_ID):
        if on:
            self.canvas.itemconfig("B" + str(BOID_ID), fill = "green")
        else:
            self.canvas.itemconfig("B" + str(BOID_ID), fill = "red")


    # Draws or updates a line on the screen, depending on if the tag can be found
    def drawLine(self, startPoints, endPoints, tag):
        handle = self.canvas.find_withtag(tag)

        if handle:
            self.canvas.coords(tag, startPoints[0], startPoints[1], endPoints[0], endPoints[1])
        else:
            self.canvas.create_line([startPoints[0], startPoints[1], endPoints[0], endPoints[1]], 
                fill = "red", tags = tag)


    def drawCircle(self, centre, radius, tag):
        self.canvas.delete(tag)
        
        self.canvas.create_oval(centre[0] - radius, centre[1] - radius, centre[0] + radius, 
            centre[1] + radius, outline = "white", fill = "white", tags = tag)


    def drawBoidCPUGrid(self, boidCPUCoords, segmentWidth, segmentHieght):
        for i in range(segmentWidth):
            x = boidCPUCoords[0] + (i * self.config['stepSize'])
            self.canvas.create_line([x, boidCPUCoords[1], x, boidCPUCoords[3]], fill = "green", 
                tags = "gridLines")

        for i in range(segmentHieght):
            y = boidCPUCoords[1] + (i * self.config['stepSize'])
            self.canvas.create_line([boidCPUCoords[0], y, boidCPUCoords[2], y], fill = "green", 
                tags = "gridLines")


    def removeObject(self, tag):
        self.canvas.delete(tag)



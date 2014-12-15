#!/usr/bin/python
# -*- coding: utf-8 -*-


# from boid import Boid               # Import the Boid class

# import numpy as np                  # Used in various mathematical operations
# import logging                      # Used to handle the textual output
# import random                       # Used to randomly position the boids on initialisation
from Tkinter import *               # Used to draw shapes for the simulation


# 
class BoidGPU:

    def __init__(self, _simulation):
        # Define window size
        self.width = 700
        self.height = 700

        # Link back to simulation
        self.simulation = _simulation

        # Create the window
        self.root = Tk()
        self.root.wm_title("Boid Simulation")

        frame = Frame(self.root)
        frame.pack()
        
        self.canvas = Canvas(frame, bg = "black", width = self.width, height = self.height)
        self.canvas.pack();
        
        # Create the buttons
        self.timeButton = Button(frame, text = "Next Time Step", command = self.simulation.nextStepButton)
        self.timeButton.pack(side = LEFT)
        self.pauseButton = Button(frame, text = "Begin", command = self.simulation.pause)
        self.pauseButton.pack(side = LEFT)
        self.graphButton = Button(frame, text = "Update Graphs", command = self.simulation.updateGraphs)
        self.graphButton.pack(side = LEFT)
        self.quitButton = Button(frame, text = "Quit", command = frame.quit)
        self.quitButton.pack(side = LEFT)

        self.counterLabel = Label(frame, text = 0, width = 6)
        self.counterLabel.pack(side = RIGHT)

        # Needed so that the canvas sizes can be used later
        self.root.update()

    # Temporary method
    def getCanvas(self):
        return self.canvas


    def beginMainLoop(self):
        # Start everything going
        self.root.mainloop()


    def getWindowSize(self):
        return [self.canvas.winfo_width(), self.canvas.winfo_height()]


    def updateTimeStepLabel(self, newTimeStep):
        self.counterLabel.config(text = newTimeStep)


    def togglePauseButton(self, resume):
        if resume:
            self.pauseButton.config(text = "Pause")
        else:
            self.pauseButton.config(text = "Resume")


    def initialiseSimulation(self):
        print

    def updateBoids(self, locationID, boids):
        print 


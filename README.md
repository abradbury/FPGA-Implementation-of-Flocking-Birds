FPGA Flocking Birds Simulator
=============================

A final-year university project to produce a scalable flocking birds simulator implemented across multiple FPGAs outputting to a HD monitor. The simulation is based on the flocking birds algorithm proposed by [Craig Reynolds](http://www.red3d.com/cwr/boids/).

Python Simulator
----------------
Initially, a Python simulation was created to investigate any issues that may occur during formal development and to act as a test-bed for new techniques. 

Vivado HLS (VHLS) Prototype
---------------------------
The next stage was to develop a VHLS prototype of the system. VHLS was chosen due to relative familiarity.

Bluespec SystemVerilog (BSV) Final Implementation
-------------------------------------------------
The initial intention was to develop the final implementation using [Bluespec SystemVerilog](http://wiki.bluespec.com/), a relatively new hardware description language that would enable greater customisation than a VHLS solution. 

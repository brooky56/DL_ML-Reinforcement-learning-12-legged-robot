from pyrep import PyRep
from pyrep import PyRep, objects

import time

pr = PyRep()
# Launch the application with a scene file in headless mode
pr.launch('strirus.ttt', headless=False) 
pr.start()  # Start the simulation

# Task related initialisations in Simulator
robot = objects.dummy.Dummy("Strirus")

print(robot)

pr.stop()  # Stop the simulation
pr.shutdown()  # Close the application

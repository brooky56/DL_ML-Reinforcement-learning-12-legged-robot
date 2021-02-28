from pyrep import PyRep
import time

pr = PyRep()
# Launch the application with a scene file in headless mode
pr.launch('strirus.ttt', headless=False) 
pr.start()  # Start the simulation

time.sleep(60)

pr.stop()  # Stop the simulation
pr.shutdown()  # Close the application

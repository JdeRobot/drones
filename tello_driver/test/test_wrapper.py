import time
from drone_wrapper import DroneWrapper

print("Test started.")
drone = DroneWrapper()
drone.takeoff(h=0.2)
time.sleep(5)

print("Drone at {0}".format(drone.get_position()[-1]))

drone.set_cmd_pos(z=0.5)
time.sleep(5)
print("Drone at {0}".format(drone.get_position()[-1]))

# drone.set_cmd_pos(z=1.0)
# time.sleep(5)
# print("Drone at {0}".format(drone.get_position()[-1]))

# drone.set_cmd_vel(az=0.5)
# time.sleep(5)

drone.land()
time.sleep(3)
print("Test finished.")

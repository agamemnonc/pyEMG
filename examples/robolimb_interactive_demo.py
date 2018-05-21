from pyEMG.robolimb import RoboLimb
import time
import sys


r = RoboLimb()
r.start()
time.sleep(2)

movement_dict = {1:"cylindrical", 2:"lateral", 3:"tridigit_ext", 4:"bidigit_ext", 5:"pointer", 6:"thumbs_up"}

# Support Python 2 and 3 input
get_input = input
if sys.version_info[:2] <= (2, 7):
    get_input = raw_input

while(True):
	grasp_input = get_input('Grasp (1:cylindrical, 2:lateral, 3:tridigit_ext, 4:bidigit_ext, 5:pointer, 6:thumbs_up, 0:exit)\n') # Wait for input after object placement
	if int(grasp_input) not in (list(movement_dict.keys()) + [0]):
		print("Movement not understood, pleas try again.")
		pass

	elif grasp_input == str(0):
		print("Exiting...")
		r.open_all()
		time.sleep(1)
		r.close_finger(1)
		time.sleep(1)
		r.stop()
		sys.exit()

	else:
		r.grasp(movement_dict[int(grasp_input)])
		time.sleep(1.5)
		_ = get_input('Press to open')
		r.open_fingers()
		time.sleep(1.5)

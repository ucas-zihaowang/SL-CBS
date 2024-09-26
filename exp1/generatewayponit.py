# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

import numpy as np
import math

if __name__ == '__main__':
	num = 14
	radius = 5
	theta = np.linspace(0, 360, num + 1).tolist()
	theta.pop()
	print( theta )
	i = 0;
	for ta in theta:
		hudu = math.pi * ta / 180
		x_index = round( 5 + radius * math.cos(hudu), 2)
		y_index = round( 0 + radius * math.sin(hudu), 2)

		print(f'- uav_id: {i}')
		print(f'  start_pos: [{x_index},{y_index}]')
		print(f'  goal_pos: [{x_index},{y_index}]')
		i+=1
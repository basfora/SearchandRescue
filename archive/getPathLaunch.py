import csv
import numpy as np
import sys

graphFile = 'SchoolGraph.txt'
x_off, y_off = -34.85, -23

# -------- helper functions -------- #
def getCoordinates(node, lookUp): 
	row = lookUp[node - 1]
	transformed_point = change_coordinates(np.array((row[1], row[2])))
	print(transformed_point)
	return '{},{},{}'.format(transformed_point[0], transformed_point[1], 0)

def getAngles(node, lookUp):
	# row = lookUp[node - 1]
	# return '{}'.format(row[4])
	return '0'

def getPathCoordinates(path, lookUp):
	string = ''
	for i in path:
		temp = getCoordinates(i, lookUp) + ', '
		string += temp
	return string[:-2]

def getPathAngles(path, lookUp): 
	string = ''
	for i in path:
		temp = getAngles(i, lookUp) + ', '
		string += temp
	return string[:-2]

def change_coordinates(points):
	x, y = points
	theta = np.radians(0)
	c, s = np.cos(theta), np.sin(theta)
	R = np.array(((c, -s, x_off), (s, c, y_off), (0, 0, 1)))

	temp = np.array((x, y, 1))
	output = np.dot(R, temp)
	return [round(i, 3) for i in output]
# ---------------------------------- #

if __name__ == "__main__": 
	temp = (r for r in open(graphFile) if not r[0] == '#')
	lookUp = np.genfromtxt(temp, delimiter=' ', names = True)
	lookUp = sorted(lookUp, key = lambda x: x[0])

	# user input section
	# print('There are %s vertices in this graph.' len(lookUp))
	print('Navigation goal, separate nodes with commas. (e.g. 1, 2, 3): ')
	valid = False
	while (not valid):
		try: 
			path = [int(s) for s in raw_input().split(',')]
			if all (i <= len(lookUp) for i in path) and all (i > 0 for i in path): 
				valid = True
			else: 
				print('Invalid node, try again.')
		except (KeyboardInterrupt, SystemExit): 
			print ('quitting program. ')
			sys.exit(0)
		except: 
			print ('format error, try again. ')



	# open files for reading and writing
	f = open('move_base_seq.launch', 'w+')

	lines = ['<?xml version="1.0"?>\n',
		'\t<launch>\n', 
		'\t<arg name="ns"\tdefault="jackal0"/>\n',
		'\t\t<node pkg="simple_navigation_goals" type="move_base_seq.py" name="move_base_seq" ns="$(arg ns)" output="screen">\n',
		'\t\t\t<rosparam param="p_seq">[', getPathCoordinates(path, lookUp), ']</rosparam>\n',
		'\t\t\t<rosparam param="yea_seq">[', getPathAngles(path, lookUp), ']</rosparam>\n', 
		'\t\t</node>\n', 
		'\t</launch>']


	f.writelines(lines)
	f.close()
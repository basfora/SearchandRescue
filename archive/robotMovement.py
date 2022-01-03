import csv
from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt

# importing data from csv
with open('School_CamsLeft.csv', 'r') as f: 
	csv_reader = csv.reader(f)
	left_data = list(csv_reader)

with open('School_CamsRight.csv', 'r') as f: 
	csv_reader = csv.reader(f)
	right_data = list(csv_reader)

left_x = []
left_y = []
left_z = []

right_x = []
right_y = []
right_z = []
for i in range(len(left_data)):
	if (i != 0):
		left_x.append((float)(left_data[i][1]))
		left_y.append((float)(left_data[i][2]))
		left_z.append((float)(left_data[i][3]))
		right_x.append((float)(right_data[i][1]))
		right_y.append((float)(right_data[i][2]))
		right_z.append((float)(right_data[i][3]))


# plot
fig = plt.figure()
ax1 = fig.add_subplot(211, projection = '3d')
ax1.plot(left_z, left_x, left_y, label = 'robot path')
ax1.legend()
ax1.set_xlabel('x')
ax1.set_ylabel('y')
ax1.set_zlabel('z')
ax1.set_ylim([30, -60])
ax1.set_zlim([1, 5])

ax2 = fig.add_subplot(212)
ax2.plot(left_z, left_x)
ax2.set_xlabel('x')
ax2.set_ylabel('y')
ax2.set_ylim([30, -60])
# ax2 = fig.add_subplot(212, projection = '3d')
# ax2.scatter(right_x, right_y, right_z)
plt.show()

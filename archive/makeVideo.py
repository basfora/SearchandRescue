import re
import os 
import cv2
from PIL import Image  
import fnmatch
import glob

def makeVid(title, img_array):
	out = cv2.VideoWriter(title,cv2.VideoWriter_fourcc(*'XVID'), 30, (width, height))
 	for i in range(len(img_array)):
		out.write(img_array[i][1])
	out.release()



os.chdir("/home/samhong/Desktop/Research/Normal")   
path = os.getcwd()

dirFiles = os.listdir('.')
dirFiles.sort(key = lambda f: int(filter(str.isdigit, f)))
img_array =[]
count = 0
video_count = 0

for file in dirFiles: 
	if not file.startswith('.') and file != None:
		count = count + 1
		img = cv2.imread(file)
		height, width, layers = img.shape
		size = (width,height)
		img_array.append((file, img))
		print(len(img_array))
		if count == 600:
			makeVid('project' + str(video_count) + '.avi', img_array)
			img_array = []
			count = 0
			video_count = video_count + 1

makeVid('project' + str(video_count) + '.avi', img_array)
print ('finish')


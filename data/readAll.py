import pickle, dill, sys, os
import numpy as np
import pandas as pd


class Info: pass

sims = [file for file in os.listdir() if 'sim' in file]
columns = ['sim', 'belief', 'missionOutcome', 'timeStep', 'finalPose', 'visitedVertices', 
	'simTime', 'robotIds', 'robotDeathLocation', 'robotDeathTime', 'robotDeathCause']
df = pd.DataFrame(columns=columns)

for i, sim in enumerate(sims): 
    folderPath = os.path.join(os.getcwd(), sim)
    pickleFiles = os.listdir(folderPath)
    for j, file in enumerate(pickleFiles): 
        pickleFilePath = os.path.join(folderPath, file, 'output.pickle')
        if not os.path.isfile(pickleFilePath): 
            continue
        file = open(pickleFilePath, 'rb')
        info = dill.load(file)

        if 'pose' in vars(info): 
            pose = info.pose
        else: 
            pose = 'nan'

        df = df.append({'sim': sim, 
    					'belief': pickleFiles[j].split('_')[1], 
    					'missionOutcome': info.missionStatus, 
    					'timeStep': info.time_step, 
    					'finalPose': pose, 
    					'visitedVertices': info.visited, 
    					'simTime': info.time, 
    					'robotIds': info.robotIDs, 
    					'robotDeathLocation': [info.died[i][0] for i in range(len(info.robotIDs))],
    					'robotDeathTime': [info.died[i][1] for i in range(len(info.robotIDs))], 
    					'robotDeathCause': [info.died[i][2] for i in range(len(info.robotIDs))]}, ignore_index=True)


df.to_csv('out.csv', index=False)

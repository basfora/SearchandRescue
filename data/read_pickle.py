import pickle
import sys


class Info: pass

# path to output.picke file
name_folder = sys.argv[1]

file = open(str(name_folder) + "/output.pickle", 'rb')
info = pickle.load(file)

print('mission outcome:', info.missionStatus)
print('time step:', info.time_step)
if info.missionStatus=="TARGET FOUND":
    print('final pose:', info.pose)
print('visited vertices:', info.visited)
print('simulation time:', info.time)
print('robot IDs:',info.robotIDs)
print('robot death location:', [info.died[i][0] for i in range(len(info.robotIDs))])
print('robot death time:', [info.died[i][1] for i in range(len(info.robotIDs))])
print('robot death cause:', [info.died[i][2] for i in range(len(info.robotIDs))])

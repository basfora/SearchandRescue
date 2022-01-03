import sys
import os
import ast
from collections import deque
this_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
abspath = os.path.dirname(os.path.abspath(__file__))
sys.path.append(this_path)
# -------------------
# Packages from Beatriz' code
from milp_sim.risk.classes.gazebo import MyGazeboSim
"""
Reads the current belief vector and position from text file
"""
def read_inputs():
    # TODO check this works
    f = open(abspath + '/inputs.txt', 'r')

    belief_vector = ast.literal_eval(f.readline())
    num_robots = int(f.readline())

    print('num_robots: ', num_robots)
    points = []

    for i in range(num_robots):
        points.append(int(f.readline()))

    visited_vertices = ast.literal_eval(f.readline())
    time_step = int(f.readline())

    f.close()

    return belief_vector, points, visited_vertices, time_step

"""
Writes the output belief vector and list of goals into a text file
"""
def write_outputs(belief_vector, vertices, visited, time_step=1):
    line1 = str(belief_vector) + '\n'

    f = open(abspath + '/outputs.txt', 'w')
    f.write(line1)
    for i in range(len(vertices)):
        f.write(str(vertices[i]) + '\n')

    f.write(str(visited) + '\n')
    f.write(str(time_step))

    f.close()
    return

"""
Given a dictionary, outputs a list
"""
def return_as_list(my_dict):
    my_list = []

    for k in my_dict.keys():
        my_list.append(my_dict[k])

    return my_list

"""
Given a list of lists, get rid of first element on each list
"""
def remove_first(my_lists):
    for my_list in my_lists:
        del my_list[0]

    return my_lists

def process_belief(belief):
    mapping = {0.1667: 0.16666666}
    belief_new = [mapping[v] if v in mapping else v for v in belief]
    return belief_new

"""
Run the planner after initializing and call necessary functions to pass the output to simulator
"""
def run_planner():
    # get new info
    belief_t, positions, visited_vertices, time_step = read_inputs()

    print('Belief received: ', belief_t)
    print('Positions: ', positions)

    sim_op = 1
    my_sim = MyGazeboSim(positions, belief_t, visited_vertices, time_step, sim_op)
    my_path, belief_vector, visited = my_sim.output_results()

    path_list = return_as_list(my_path)

    print('Belief generated: ', belief_vector)
    # write txt file
    write_outputs(belief_vector, remove_first(path_list), visited, time_step + 1)

    return


if __name__ == "__main__":
    run_planner()







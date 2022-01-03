import sys
import os
import ast
from collections import deque
this_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
abspath = os.path.dirname(os.path.abspath(__file__))
sys.path.append(this_path)
# -------------------
# Packages from Beatriz' code
from milp_mespp.core import extract_info as ext
from milp_mespp.core import sim_fun as sf
from milp_mespp.core import plan_fun as pln
from milp_mespp.classes.inputs import MyInputs
from milp_mespp.classes.belief import MyBelief
from milp_mespp.classes.searcher import MySearcher
from milp_mespp.core import create_parameters as cp

"""
Reads the current belief vector and position from text file
"""
def read_inputs():
    f = open(abspath + '/inputs.txt', 'r')
    belief_vector = ast.literal_eval(f.readline())
    num_robots = int(f.readline())
    print('num_robots: ', num_robots)
    points = []
    for i in range(num_robots):
        points.append(int(f.readline()))
    time_step = int(f.readline())
    f.close()
    return belief_vector, points, time_step

"""
Writes the output belief vector and list of goals into a text file
"""
def write_outputs(belief_vector, vertices):
    line1 = str(belief_vector) + '\n'

    f = open(abspath + '/outputs.txt', 'w')
    f.write(line1)
    for i in range(len(vertices)):
        f.write(str(vertices[i]) + '\n')

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
Initialize the planner the pre-set parameters
    If needed, change parameters here
"""
def initialize_planner(num_robots):
    # ---------------------------------------
    # fixed parameters
    # re-planning frequency (each time step)
    theta = 1
    # planning horizon
    horizon = 10
    # solver timeout (decrease for a faster computer)
    timeout = 120
    # ---------------------------------------
    # initialize default inputs
    exp_inputs = MyInputs()
    exp_inputs.set_graph(8)

    # solver parameter: central x distributed
    exp_inputs.set_solver_type('distributed')
    # target motion
    exp_inputs.set_target_motion('static')

    # searchers
    m = num_robots
    exp_inputs.set_size_team(m)
    exp_inputs.set_capture_range(0)

    # time stuff
    # time-step stuff: deadline mission (tau), planning horizon (h), re-plan frequency (theta)
    exp_inputs.set_all_times(horizon)
    exp_inputs.set_theta(theta)
    # solver timeout (in sec)
    exp_inputs.set_timeout(timeout)

    return exp_inputs

"""
Run the planner after initializing and call necessary functions to pass the output to simulator
"""
def run_planner():
    # get new info
    belief_t, positions, time_step = read_inputs()
    # get specs
    specs = initialize_planner(len(positions))
    print('Belief received: ', belief_t)
    print('Positions: ', positions)
    # update belief
    specs.set_start_searchers(positions)
    # update number of searchers
    specs.set_b0(belief_t)

    output_solver_data = True
    my_path, solver_data = pln.run_planner(specs, output_solver_data)

    # to retrieve belief computed by the solver
    # we want b(t+1) - next time step
    t_next = 1
    my_belief_vec = process_belief(solver_data.retrieve_solver_belief(0, t_next))

    path_list = return_as_list(my_path)
    # if time_step == 2:
    #     path_list[2][1] = -1

    print('Belief generated: ', my_belief_vec)
    # write txt file
    write_outputs(my_belief_vec, remove_first(path_list))

    return


if __name__ == "__main__":
    run_planner()







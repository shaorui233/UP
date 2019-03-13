import os 
dir_path = os.path.dirname(os.path.realpath(__file__))

import sys
sys.path.append(dir_path + '/python_bridge')
sys.path.append(dir_path + '/../build/machine_learning')

import pycheetah as cheetah

def step_sim(input):
    cheetah.step(input)

step_sim(3.)

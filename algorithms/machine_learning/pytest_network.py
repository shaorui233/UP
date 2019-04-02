import random
import gym
import numpy as np
import math
from collections import deque
from keras.models import Model
from keras.models import load_model
from keras.layers import Input
from keras.layers import Dense
from keras.optimizers import Adam

#from scores.score_logger import ScoreLogger

import os
dir_path = os.path.dirname(os.path.realpath(__file__))

import sys
sys.path.append(dir_path + '/python_bridge')
sys.path.append(dir_path + '/../../build/algorithms/machine_learning')
import pycheetah as cheetah

ENV_NAME = "Mini-Cheetah"

# Learning parameters
GAMMA = 0.95
LEARNING_RATE = 0.001

MEMORY_SIZE = 1000000
BATCH_SIZE = 20

EXPLORATION_MAX = 1.0
EXPLORATION_MIN = 0.01
EXPLORATION_DECAY = 0.995

T_sampling = 10 # sampling iteration

class DQNSolver:

    def __init__(self, observation_space, action_space):
        self.exploration_rate = EXPLORATION_MAX

        self.action_space = action_space
        self.memory = deque(maxlen=MEMORY_SIZE) # define a data container

#        self.model = Sequential() # create a neural network using Keras models
#        self.model.add(Dense(24, input_shape=(observation_space,), activation="relu"))
#        self.model.add(Dense(24, activation="relu"))
#        self.model.add(Dense(self.action_space, activation="linear"))
        inputs = Input(shape=(observation_space,)) # N is the width of any input element, say you have 50000 data points, and each one is a vector of 3 elements, then N is 3

        x = Dense(24, activation='relu')(inputs) # this is your network, let's say you have 2 hidden layers of 64 nodes each (don't know if that's enough for you)
        x = Dense(24, activation='relu')(x)

        outputs = Dense(action_space, activation='tanh')(x)
#        outputs = Dense(action_space, activation='softmax')(x)

        self.model = Model(input=inputs, output=outputs)

        self.model.compile(loss="mse", optimizer=Adam(lr=LEARNING_RATE))

    def remember(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done)) # accumulate data
        # append(x)=Add x to the right side of the deque.

    def act(self, state):
#        random_action = np.zeros(12)
#        if np.random.rand() < self.exploration_rate: # take a random action with a probability of exploration_rate
#            for i in range(12):
#                random_action[i] = state[0,i+6]+np.random.rand(1)/10
#            return random_action
        q_values = self.model.predict(state)
        print('action from the network')
        return np.argmax(q_values[0]) # otherwise use the network to take action

    def experience_replay(self):
        if len(self.memory) < BATCH_SIZE:
            return
        batch = random.sample(self.memory, BATCH_SIZE)
        for state, action, reward, state_next, terminal in batch:
            q_update = reward
            if not terminal:
                q_update = (reward + GAMMA * np.amax(self.model.predict(state_next)[0]))
            q_values = self.model.predict(state)
#            q_values[0][action] = q_update #??? Need to update this
            self.model.fit(state, q_values, verbose=0)
        # update exploration rate for the act() function
        # exploration_rate => probability of taking a random action
        self.exploration_rate *= EXPLORATION_DECAY
        self.exploration_rate = max(EXPLORATION_MIN, self.exploration_rate)

def reward_function(state):
    z_com = state[5]
    z_com_d = 0.3
    reward = -(z_com-z_com_d)**2
    return reward

def termination_flag(state,time_step):
    z_com = state[5]
    done = 0
    if time_step>100 and z_com < 0.05:
        done = 1

    if z_com > 0.45:
        done = 1
#    print('z_com')
#    print z_com
    return done

def reset_cheetah_simulation(): # need a better way to reset simulation
    for i in range(100):
#        print i
        in_jpos = cheetah.doubleArray(12)
        in_jvel = cheetah.doubleArray(12)

        out_config = cheetah.doubleArray(18)
        out_config_vel = cheetah.doubleArray(18)

        for j in range(12):
            in_jpos[j] = 0.
            in_jvel[j] = 0.

        in_jpos[0] = -0.03
        in_jpos[1] = -0.79
        in_jpos[2] = 1.715

        in_jpos[3] = 0.03
        in_jpos[4] = -0.72
        in_jpos[5] = 1.715

        in_jpos[6] = -0.03
        in_jpos[7] = -0.79
        in_jpos[8] = 1.715

        in_jpos[9] = 0.03
        in_jpos[10] = -0.72
        in_jpos[11] = 1.715

        cheetah.step(in_jpos, in_jvel, out_config, out_config_vel)

    state = np.zeros(36)

    for i in range(18):
        state[i] = out_config[i]
        state[i+18] = out_config_vel[i]

    return state

def env_cheetah_step(action,time_step):
    for i in range(T_sampling):
#        print i
        in_jpos = cheetah.doubleArray(12)
        in_jvel = cheetah.doubleArray(12)

        out_config = cheetah.doubleArray(18)
        out_config_vel = cheetah.doubleArray(18)

        for j in range(12):
            in_jpos[j] = action[j]
            in_jvel[j] = 0.

        cheetah.step(in_jpos, in_jvel, out_config, out_config_vel)

    state = np.zeros(36)

    for i in range(18):
        state[i] = out_config[i]
        state[i+18] = out_config_vel[i]

    reward = reward_function(state)
    done = termination_flag(state, time_step)
    return state, reward, done

def cheetah_reinforcement_learning():
    observation_space = 36
    action_space = 12
    dqn_solver = DQNSolver(observation_space, action_space)
    run = 0
    state = reset_cheetah_simulation()
    state = np.reshape(state, [1, observation_space])
    action = dqn_solver.act(state)
    print('action:')
    print action


if __name__ == "__main__":
    cheetah_reinforcement_learning()

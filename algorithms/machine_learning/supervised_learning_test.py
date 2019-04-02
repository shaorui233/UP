#This code tries to use simulation data to learn
#a network that get z_com command and output q_knee_d to achieve the command

import os 
dir_path = os.path.dirname(os.path.realpath(__file__))

import sys
sys.path.append(dir_path + '/python_bridge')
sys.path.append(dir_path + '/../../build/algorithms/machine_learning')

import pycheetah as cheetah
import numpy as np
from numpy import *
import tensorflow as tf
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.patches as mpatches

def tf_reset():
    try:
        sess.close()
    except:
        pass
    tf.reset_default_graph()
    return tf.Session()

# set up parameters
N = 5000 # number of simulated iterations
T = 100 # data collection period (T/1000 s)
N_data = round(N/T)
N_training = 5000
i_data = 0

# initialize the data
inputs = np.linspace(-2*np.pi, 2*np.pi, N_data)[:, None]
outputs = np.sin(inputs)# + 0.05 * np.random.normal(size=[len(inputs),1])

# ============= Run Simulation ==========================
for i in range(N):#(10000):
#    print i
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


    a = 0.5;
    in_jpos[2] += a*np.sin(i*0.001)
    in_jvel[2] = a*np.cos(i*0.001)

    in_jpos[5] += a*np.sin(i*0.001)
    in_jvel[5] = a*np.cos(i*0.001)

    in_jpos[8] += a*np.sin(i*0.001)
    in_jvel[8] = a*np.cos(i*0.001)

    in_jpos[11] += a*np.sin(i*0.001)
    in_jvel[11] = a*np.cos(i*0.001)

    cheetah.step(in_jpos, in_jvel, out_config, out_config_vel)

#    for j in range(18):
#        print out_config[j] # rpy, xyz, q

    # update data from cheetah simulation
    if i % T == 0:
        print i
        inputs[i_data] = out_config[5] # z_com
        outputs[i_data] = in_jpos[2] # desired knee joint angle of leg 0
        i_data = i_data + 1

#========================== Implementation of Supervised Learning =================================

#print('Data: Inputs:')
#print inputs
#print('Data: Outputs:')
#print outputs

#print('Plot data')
#plt.scatter(inputs[:, 0], outputs[:, 0], s=0.1, color='k', marker='o')

sess = tf_reset()

def create_model():
    # create inputs
    input_ph = tf.placeholder(dtype=tf.float32, shape=[None, 1])
    output_ph = tf.placeholder(dtype=tf.float32, shape=[None, 1])

    # create variables
    W0 = tf.get_variable(name='W0', shape=[1, 20], initializer=tf.contrib.layers.xavier_initializer())
    W1 = tf.get_variable(name='W1', shape=[20, 20], initializer=tf.contrib.layers.xavier_initializer())
    W2 = tf.get_variable(name='W2', shape=[20, 1], initializer=tf.contrib.layers.xavier_initializer())

    b0 = tf.get_variable(name='b0', shape=[20], initializer=tf.constant_initializer(0.))
    b1 = tf.get_variable(name='b1', shape=[20], initializer=tf.constant_initializer(0.))
    b2 = tf.get_variable(name='b2', shape=[1], initializer=tf.constant_initializer(0.))

    weights = [W0, W1, W2]
    biases = [b0, b1, b2]
    activations = [tf.nn.relu, tf.nn.relu, None]

    # create computation graph
    layer = input_ph
    for W, b, activation in zip(weights, biases, activations):
        layer = tf.matmul(layer, W) + b
        if activation is not None:
            layer = activation(layer)
    output_pred = layer

    return input_ph, output_ph, output_pred

input_ph, output_ph, output_pred = create_model()

# create loss
mse = tf.reduce_mean(0.5 * tf.square(output_pred - output_ph))

# create optimizer
opt = tf.train.AdamOptimizer().minimize(mse)

# initialize variables
sess.run(tf.global_variables_initializer())
# create saver to save model variables
saver = tf.train.Saver()

# run training
batch_size = 32
for training_step in range(N_training):#(10000):
    # get a random subset of the training data
    indices = np.random.randint(low=0, high=len(inputs), size=batch_size)
    input_batch = inputs[indices]
    output_batch = outputs[indices]

    # run the optimizer and get the mse
    _, mse_run = sess.run([opt, mse], feed_dict={input_ph: input_batch, output_ph: output_batch})

    # print the mse every so often
    if training_step % 1000 == 0:
        print('{0:04d} mse: {1:.3f}'.format(training_step, mse_run))

    if training_step == N_training - 1:
        print('Done training!!! Save trained model.')
        saver.save(sess, 'model.ckpt')

# Evaluate the trained network
sess = tf_reset()

# create the model
input_ph, output_ph, output_pred = create_model()

# restore the saved model
saver = tf.train.Saver()
saver.restore(sess, "model.ckpt")

output_pred_run = sess.run(output_pred, feed_dict={input_ph: inputs})

plt.scatter(inputs[:, 0], outputs[:, 0], c='k', marker='o', s=0.1)
plt.scatter(inputs[:, 0], output_pred_run[:, 0], c='r', marker='o', s=0.1)
print('output_pred_run:')
print output_pred_run
print('output: (knee joint angle of leg 0)')
print outputs
print('input: (z_com)')
print inputs

print ('============= Use the Trained Network to command z_com ==============')
z_com_d = 0.3
print('z_com_d')
print z_com_d

inputs[0] = z_com_d
output_pred_run = sess.run(output_pred, feed_dict={input_ph: inputs})
q_knee_d = double(output_pred_run[0,0])
print('q_knee_d:')
print q_knee_d

# Run simulation with z_com command from the network
# ============= Run Simulation ==========================
for i in range(1000):#(10000):

    for j in range(12):
        in_jvel[j] = 0.

    in_jpos[0] = -0.03
    in_jpos[1] = -0.79
    in_jpos[2] = q_knee_d

    in_jpos[3] = 0.03
    in_jpos[4] = -0.72
    in_jpos[5] = q_knee_d

    in_jpos[6] = -0.03
    in_jpos[7] = -0.79
    in_jpos[8] = q_knee_d

    in_jpos[9] = 0.03
    in_jpos[10] = -0.72
    in_jpos[11] = q_knee_d

    cheetah.step(in_jpos, in_jvel, out_config, out_config_vel)

z_com_final = out_config[5]
print('COM height:')
print z_com_final
print('Done !!!')

#    x_train[i] = in_jpos[2]
#    y_train[i] = out_config[8]

#    x_test[i] = in_jpos[5]
#    y_test[i] = out_config[11]

#model = tf.keras.models.Sequential([
#  #tf.keras.layers.Flatten(input_shape=(28, 28)),
#  tf.keras.layers.Dense(512, activation=tf.nn.relu),
#  tf.keras.layers.Dropout(0.2),
#  tf.keras.layers.Dense(10, activation=tf.nn.softmax)
#])
#model.compile(optimizer='adam',
#              loss='sparse_categorical_crossentropy',
#              metrics=['accuracy'])

#model.fit(x_train, y_train, epochs=5)
#model.evaluate(x_test, y_test)

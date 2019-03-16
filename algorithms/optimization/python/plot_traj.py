import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import os

# Plot configuration
PLOT_VERTICALLY = 0
PLOT_HORIZONTALLY = 1

# number of figures in this plot

def create_figures(subfigure_width=480, subfigure_height=600, starting_figure_no=1, starting_col_index = 0, starting_row_index=0, plot_configuration=PLOT_HORIZONTALLY):

    figure_number = starting_figure_no
    col_index = starting_col_index
    row_index = starting_row_index

    ## read files --------------------------------------------------------------------
    file_path = os.getcwd() + "/../../experiment_data/"

    pos = \
            np.genfromtxt(file_path+'pos.txt', delimiter=None, dtype=(float))
    vel = \
            np.genfromtxt(file_path+'vel.txt', delimiter=None, dtype=(float))
    acc = \
            np.genfromtxt(file_path+'acc.txt', delimiter=None, dtype=(float))
    time = np.genfromtxt(file_path+'time.txt', delimiter='\n', dtype=(float))
    
    st_idx = 0
    end_idx = len(time)
    time = time[st_idx:end_idx]
    dim = 3
    # Plot Figure --------------------------------------------------------------------
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))
    fig.canvas.set_window_title('pos')
    for i in range(0,dim,1):
        ax1 = plt.subplot(dim, 1, i+1)
        plt.plot(time, pos[st_idx:end_idx,i])
    plt.xlabel('time (sec)')

    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1
    #----------------------------------------------------------------------------------

    # Plot Figure --------------------------------------------------------------------
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))
    fig.canvas.set_window_title('vel')
    for i in range(0,dim,1):
        ax1 = plt.subplot(dim, 1, i+1)
        plt.plot(time, vel[st_idx:end_idx,i])
    plt.xlabel('time (sec)')

    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1
    #----------------------------------------------------------------------------------

    # Plot Figure --------------------------------------------------------------------
    ## plot command/jpos
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))
    fig.canvas.set_window_title('acc')
    for i in range(0,dim,1):
        ax1 = plt.subplot(dim, 1, i+1)
        plt.plot(time, acc[st_idx:end_idx,i])
    plt.xlabel('time (sec)')

    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1
    #----------------------------------------------------------------------------------

    # Plot Figure --------------------------------------------------------------------
    ## plot command/jpos
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))
    fig.canvas.set_window_title('trj')
    plt.plot(pos[st_idx:end_idx, 0], pos[st_idx:end_idx,1])
    plt.xlabel('X')

    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1
    #----------------------------------------------------------------------------------


if __name__ == "__main__":
    create_figures()
    plt.show()


import rps.robotarium as robotarium
from rps.utilities.transformations import *
from rps.utilities.barrier_certificates import *
from rps.utilities.misc import *
from rps.utilities.controllers import *
from gp_prediction import GP_Model

import numpy as np
import time
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import random
from datetime import datetime
import math
from matplotlib import cm
from numpy import unravel_index




# Instantiate Robotarium object
N = 3
# Robot colors

ROBOT_COLOR = {0: "#1E7311", 1: "#FCFF23", 2: "#FC0000"}
# initial_conditions = np.array(np.mat('1 0.5 -0.5 0 0.28; 0.8 -0.3 -0.75 0.1 0.34; 0 0 0 0 0'))
initial_conditions = np.array(np.mat('0 -1.5 1.5; 0.8 -0.8 -0.8; 0 0 0'))
initial_positions = np.array(np.mat('0 -1.5 1.5; 0.8 -0.8 -0.8; 0 0 0'))
dimensions ={"global":((-1.6,1.6),(-1,1)),"robot1":((-1.8,1.8),(-2.2,0.2)),"robot2":((-0.2,3.4),(-0.2,2.2)),"robot3":((-3.4,0.2),(-0.2,2.2))}

r = robotarium.Robotarium(number_of_robots=N, show_figure=True, initial_conditions=initial_conditions,sim_in_real_time=True)
for i in range(N):
        r.chassis_patches[i].set_facecolor(ROBOT_COLOR[i])

# Function to convert global positions to local positions
def global_to_local(global_position, reference_position):
    local_positions = (global_position - reference_position[:2])
    return local_positions

# Function to convert global positions to local positions
def local_to_global(local_position, reference_position):
    global_position = (local_position + reference_position[:2])
    return global_position

source_positions ={"global":(0,0),"robot1":(0,-9),"robot2":(14,7),"robot3":(-16,8)}
global_grid_size = (3.2,2)
local_grid_size = (3.6,2.4)
# Define the boundary for random walk
x_min, x_max = -1.6, 1.6
y_min, y_max = -1, 1
resolution = 0.1
# def pos_to_grid(pos):
#     for i, x_pos in enumerate(np.arange(x_min,x_max+resolution,resolution)):
#         for j, y_pos in enumerate(np.arange(y_max,y_min-resolution,-resolution)):
#             if pos[0]==x_pos and pos[1]==y_pos
#                 return i,j


# distance Calculation
def dist(x, y, pos):
    return math.sqrt(((pos[0]-x)**2) + ((pos[1]-y)**2))
# RSSI signal generation at pos(x,y) using path-loos model
def gen_wifi(freq=2.4, power=20, trans_gain=0, recv_gain=0, size=global_grid_size, pos=(0,0),grid_pos=(0,0),source_position=(0,0), shadow_dev=2, n=3,noise=2):
    if pos is None:
        pos = (random.randrange(size[0]), random.randrange(size[1]))

    # random.seed(datetime.now())
    rss0 = power + trans_gain + recv_gain + 20 * math.log10(3 / (4 * math.pi * freq * 10))
    rss0=rss0-noise*random.random()
    normal_dist = np.random.normal(0, shadow_dev, size=[int(size[1]/resolution)+2, int(size[0]/resolution)+2])
    # random.seed(datetime.now())
    # print(source_position)

    distance = dist(source_position[0], source_position[1], (pos[0],pos[1]))
    if distance < 0.0005:
        return rss0
    val =rss0 - ((10 * n * math.log10(distance)) + normal_dist[grid_pos[0]][grid_pos[1]])
    rss = val-noise*random.random()
    # if rss >  rss0:
    #     return rss0
    return rss

# Create unicycle position controller
unicycle_position_controller = create_clf_unicycle_position_controller()

# Create barrier certificates to avoid collision
uni_barrier_cert = create_unicycle_barrier_certificate()


# Define tolerance to each goal
tolerance = 0.05

# Plotting Parameters
goal_marker_size_m = 0.1
marker_size_goal = determine_marker_size(r,goal_marker_size_m)
font_size = determine_font_size(r,0.04)
line_width = 2

#Plot text for caption
r.axes.text(source_positions["global"][0], source_positions["global"][1], "Source", fontsize=font_size, color='k',fontweight='bold',horizontalalignment='center',verticalalignment='center',zorder=-2)

r.axes.scatter(source_positions["global"][0],source_positions["global"][1], s=marker_size_goal, marker='o', facecolors='none',edgecolors='r',linewidth=line_width,zorder=-2)

### Load Pre-trained GP models
robot_gp_models = []
for i in range(N):
    robot_gp_model = GP_Model()
    robot_gp_model.load_model("robot"+str(i+1)+"_60")
    robot_gp_models.append(robot_gp_model)
global_gp_model = GP_Model()
global_gp_model.load_model("global_60")


x_global_values = np.arange(x_min,x_max+resolution,resolution)
y_global_values = np.arange(y_min,y_max+resolution,resolution)

def calculate_global_rssi():
    rssi_values = np.zeros((int(global_grid_size[1]/resolution)+1, int(global_grid_size[0]/resolution)+1))
    for j, y_pos in enumerate(y_global_values):
        for i, x_pos in enumerate(x_global_values):
            rssi_values[j][i],_= global_gp_model.predict(np.array([[x_pos, y_pos]]))
            # rssi_values[j][i]=gen_wifi(size=global_grid_size,pos=((x_pos*10),(y_pos*10)),grid_pos=(j,i),source_position=source_positions["global"])
    return rssi_values
#Robot1 values
x_local_values = [np.arange(x_min-0.2,x_max+0.2+resolution,resolution)]
y_local_values = [np.arange(y_min*2-0.2,0.2,resolution)]
#Robot2 values
x_local_values.append(np.arange(-0.2,x_max*2+0.2+resolution,resolution))
y_local_values.append(np.arange(-0.2,y_max*2+0.2,resolution))
#Robot3 values
x_local_values.append(np.arange(x_min*2-0.2,0.2+resolution,resolution))
y_local_values.append(np.arange(-0.2,y_max*2+0.2,resolution))
def calculate_local_rssi(robot):
    rssi_values = np.zeros((int(local_grid_size[1]/resolution)+2, int(local_grid_size[0]/resolution)+2))
    for j, y_pos in enumerate(y_local_values[robot]):
        for i, x_pos in enumerate(x_local_values[robot]):
            rssi_values[j][i],_=robot_gp_models[robot].predict(np.array([[x_pos, y_pos]]))
            # rssi_values[j][i]=gen_wifi(size=local_grid_size,pos=((x_pos*10),(y_pos*10)),grid_pos=(j,i),source_position=source_positions["robot"+str(robot+1)])
    return rssi_values

    
    

# Initialize arrays to store the trajectories of all robots
global_trajectories = np.zeros((2, 0, N))
local_trajectories = [np.zeros((2, 0, N)) for i in range(N)]
# Set up the live trajectory plotting with custom layout
fig = plt.figure(constrained_layout=True)
spec = gridspec.GridSpec(ncols=2, nrows=2, figure=fig)


# Create a large axis for global trajectories
ax_global = fig.add_subplot(spec[0, 0])
ax_global.set_xlabel('x')
ax_global.set_ylabel('y')
ax_global.set(xlim=(x_min, x_max), ylim=(y_min, y_max))
ax_global.set_xticks(np.arange(x_min, x_max, 0.2))
ax_global.set_yticks(np.arange(y_min, y_max, 0.2)) 
ax_global.set_title('Global GP of Robots')
# ax_global.grid(True)
z_values=calculate_global_rssi()
c = ax_global.pcolormesh(x_global_values, y_global_values, z_values, cmap="RdBu_r", vmin=z_values.min(), vmax=z_values.max())
fig.colorbar(c, ax=ax_global)

# Create three small axes for local trajectories
ax_local = [fig.add_subplot(spec[0,1])]
ax_local.append(fig.add_subplot(spec[1,0]))
ax_local.append(fig.add_subplot(spec[1,1]))

for i, ax in enumerate(ax_local):
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    if i == 0:
        ax.set(xlim=(x_min-0.2, x_max+0.2), ylim=(y_min*2-0.2, 0.2))
        ax.set_xticks(np.arange(x_min-0.2, x_max+0.2, 0.2))
        ax.set_yticks(np.arange(y_min*2-0.2, 0.2, 0.2)) 
    elif i == 1:
        ax.set(xlim=(-0.2, x_max*2-0.2), ylim=(-0.2,y_max*2-0.2))
        ax.set_xticks(np.arange(-0.2, x_max*2-0.2, 0.2))
        ax.set_yticks(np.arange(-0.2, y_max*2-0.2, 0.2)) 
    else:
        ax.set(xlim=(x_min*2-0.2,0.2), ylim=(-0.2,y_max*2-0.2))
        ax.set_xticks(np.arange(x_min*2-0.2, 0.2, 0.2))
        ax.set_yticks(np.arange(-0.2, y_max*2-0.2, 0.2))
    ax.set_title(f'Local GP of Robot {i+1}')
    z_values=calculate_local_rssi(i)[:,:-1]
    c = ax_global.pcolormesh(x_local_values[i], y_local_values[i], z_values, cmap="RdBu_r", vmin=z_values.min(), vmax=z_values.max())
    fig.colorbar(c, ax=ax)
    # ax.grid(True)

global_lines = [ax_global.plot([], [], color=ROBOT_COLOR[i],linewidth=2, label=f'Robot {i+1}')[0] for i in range(N)]
# local_lines =[[ax.plot([], [], linewidth=2, color=ROBOT_COLOR[i])[0] for i in range(N)] for ax in ax_local]
local_marker_lines = [[ax.plot([], [], 'o', markersize=2, color=ROBOT_COLOR[i], label=f'Robot {i+1}')[0] for i in range(N)] for ax in ax_local]
for ax in ax_local:
    ax.legend()

ax_global.legend()

def update_trajectory_plot( global_trajectories, local_trajectories,rssi_vals):
    for i, g_line in enumerate(global_lines):
        # print(i)
        g_line.set_data(global_trajectories[0, :, i], global_trajectories[1, :, i])
    ax_global.pcolormesh(x_global_values, y_global_values, rssi_vals[0], cmap="RdBu_r", vmin=rssi_vals[0].min(), vmax=rssi_vals[0].max())
    for i, local_traj in enumerate(local_trajectories):
        for j, marker_line in enumerate(local_marker_lines[i]):
            marker_line.set_data(local_traj[0, :, j], local_traj[1, :, j])

            # l_line.set_data(local_traj[0, :, j], local_traj[1, :, j])

        ax_local[i].pcolormesh(x_local_values[i], y_local_values[i], rssi_vals[i+1][:,:-1], cmap="RdBu_r", vmin=rssi_vals[i+1][:,:-1].min(), vmax=rssi_vals[i+1][:,:-1].max())
        # print("robot"+str(j+1),rssi_vals[j+1].min(),rssi_vals[j+1].max())

    for ax in [ax_global, *ax_local]:
        ax.relim()
        ax.autoscale_view()

    fig.canvas.draw()
    plt.pause(0.00001)

# define x initially
x = r.get_poses()
r.step()

iterations = 150
index =0
AP_positions = np.zeros((2,N))

# Initialize a list of lists to store the localization errors for each robot
localization_errors = [[] for _ in range(N)]
localization_error_std = [[] for _ in range(N)]

# Create separate error plots for each robot
fig_error, ax_error = plt.subplots(nrows=N, ncols=1, sharex=True, figsize=(10, 6))
for i in range(N):
    ax_error[i].set_title(f'Localization Error for Robot {i+1}')
    ax_error[i].set_ylabel('RMSE')
ax_error[-1].set_xlabel('Iteration')


# Run random walk until stopped manually

while index<iterations:

    # Get poses of agents
    # x = r.get_poses()
    # print(x)

    # Generate random goal points within the boundary
    goal_points = np.vstack((np.random.uniform(low=x_min+0.2, high=x_max-0.2, size=(N,)),np.random.uniform(low=y_min+0.2, high=y_max-0.2, size=(N,)), np.zeros(N)))
    # goal_points = np.vstack((np.add(x[0],np.random.uniform(low=x_min, high=x_max, size=(N,))),np.add(x[1],np.random.uniform(low=y_min, high=y_max, size=(N,))),np.zeros(N)))
    # print(goal_points)
    while index<iterations and np.any(np.linalg.norm(x[:2, :] - goal_points[:2, :], axis=0) > tolerance):
        # Get current position
        x = r.get_poses()
        # print(x)
        # Store the current global positions in the global_trajectories array
        global_trajectories = np.concatenate((global_trajectories, x[:2, :].reshape(2, 1, N)), axis=1)
        # print(global_trajectories)
        rssi_vals = [calculate_global_rssi()]

        # Convert the global positions to local positions and store them in the local_trajectories array
        local_positions = np.zeros((2,N))
        for i in range(N):
            local_position = global_to_local(x[:2, i], initial_positions[:2, i])
            local_positions[0][i]=local_position[0]
            local_positions[1][i]=local_position[1]
            rssi_vals.append(calculate_local_rssi(i))
            # local_trajectories[i] = np.concatenate((local_trajectories[i], local_positions.reshape(2, 1, N)), axis=1)
        # Find access point location
        for i in range(N):
            ap_pos = unravel_index(rssi_vals[i+1][:,:-1].argmax(), rssi_vals[i+1][:,:-1].shape)
            # print(ap_pos)
            AP_positions[0][i]=(ap_pos[1]*resolution)+dimensions["robot"+str(i+1)][0][0]
            AP_positions[1][i]=(ap_pos[0]*resolution)+dimensions["robot"+str(i+1)][1][0]

        # predict other robots position in local frame        
        for i in range(N):
            local_poses = np.zeros((2,N))
            local_poses[:,i] = local_positions[:,i]
            mse = []
            for j in range(N):
                if i!=j:
                    ap_diff=[0,0]
                    ap_diff[0]=AP_positions[0][i]-AP_positions[0][j]
                    ap_diff[1]=AP_positions[1][i]-AP_positions[1][j]
                    local_poses[0][j]=local_positions[0][j]+ap_diff[0]
                    local_poses[1][j]=local_positions[1][j]+ap_diff[1]
                    # Calculate the root mean squared error for each robot
                    predicted_global_position = local_to_global(local_poses[:, j], initial_positions[:2, i])
                    # print(x[:2, j],local_poses[:, j], predicted_global_position)
                    mse.append(np.sqrt(np.mean((x[:2, j] - predicted_global_position) ** 2)))           
            local_trajectories[i] = np.concatenate((local_trajectories[i], local_poses.reshape(2, 1, N)), axis=1)
            total_mse = np.mean(mse)
            localization_errors[i].append(total_mse)
            localization_error_std[i].append(np.std(mse))

            

        # Update the live trajectory plot
        update_trajectory_plot(global_trajectories, local_trajectories, rssi_vals)

       # Update the error plots for each robot
        for i in range(N):
            x_values = np.arange(len(localization_errors[i]))
            y_values = np.array(localization_errors[i])
            std_values = np.array(localization_error_std[i])
            ax_error[i].plot(x_values, y_values, color=ROBOT_COLOR[i], linewidth=2)

            if len(x_values) > 1:
                current_index = len(x_values) - 1
                current_x_values = x_values[current_index - 1: current_index + 1]
                current_y_values = y_values[current_index - 1: current_index + 1]
                current_std_values = std_values[current_index - 1: current_index + 1]

                ax_error[i].fill_between(current_x_values, current_y_values - current_std_values, current_y_values + current_std_values,
                                        color=ROBOT_COLOR[i], alpha=0.1)

            ax_error[i].relim()
            ax_error[i].autoscale_view()

        # if index%5 == 0:
        for i in range(N):
            global_gp_model.update_global_gp(x[:2, i])
            robot_gp_models[i].update_local_gp(local_positions[:,i],i)


        # Create single-integrator control inputs
        dxu = unicycle_position_controller(x, goal_points[:2])

        # Create safe control inputs (i.e., no collisions)
        dxu = uni_barrier_cert(dxu, x)

        # Set the velocities by mapping the single-integrator inputs to unicycle inputs
        r.set_velocities(np.arange(N), dxu)

        # Iterate the simulation
        r.step()
        index+=1
fig.savefig('gp_plots')
fig_error.savefig('localization_error_plots')


#Call at end of script to print debug information and for your script to run on the Robotarium server properly
r.call_at_scripts_end()
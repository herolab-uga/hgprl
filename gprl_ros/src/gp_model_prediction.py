import time
import GPy
import numpy as np
import random
import math
import pickle
import os

from numpy import unravel_index



class GP_Model:

    grid_size = (13,10)
    dimensions ={"robot1":((-4,6),(-1,12)),"robot2":((-1,9),(-5,8)),"robot3":((-1,9),(-8,5)),"gp_grid":((-1,9),(-8,5))}
    # initial_positions = {"robot1":(3,-7),"robot2":(0,-3),"robot3":(0,0)}

    resolution=0.1
    model = None

    

    def update_local_gp(self,pos,rssi,robot):
        x_min,x_max = self.dimensions["robot" + str(robot + 1)][0]
        y_min,y_max = self.dimensions["robot" + str(robot + 1)][1]
        x = int(abs(x_min - pos[0]) / self.resolution)
        y = int(abs(y_min - pos[1]) / self.resolution)
        self.model.set_XY(np.vstack([self.model.X, [pos]]), np.vstack([self.model.Y, [rssi]]))
        self.model.optimize()
    def initialize_local_gp(self,X,y):
        # Initialize kernel and training data
        sigma_f, l = np.var(y), 2
        kernel = GPy.kern.RBF(2, sigma_f, l)  # Change input_dim to 2

        # Train the GP model
        self.model = GPy.models.GPRegression(X, y, kernel)
        self.model.optimize()
    def predict(self,feature):
        # Make a prediction for a new 2D feature (e.g., location)
        mean,variance =self.model.predict(feature)
        return mean,variance
    def save_model(self,name):
        with open(name+'_pretrained_gp.pkl', 'wb') as file:
            pickle.dump(self.model, file)
    def load_model(self,name):
        absolute_path = os.path.dirname(os.path.abspath(__file__))
        with open(absolute_path+'/'+name+'_pretrained_gp.pkl', 'rb') as file:
            self.model = pickle.load(file)
    def get_ap_pos(self,robot_number):
        x_local_values = np.arange(self.dimensions["robot"+str(robot_number+1)][0][0],self.dimensions["robot"+str(robot_number+1)][0][1],self.resolution)
        y_local_values = np.arange(self.dimensions["robot"+str(robot_number+1)][1][0],self.dimensions["robot"+str(robot_number+1)][1][1],self.resolution)
        predicted_rssi_values = np.zeros((len(y_local_values),len(x_local_values)))
        predicted_rssi_variance = np.zeros((len(y_local_values),len(x_local_values)))
        for j, y_pos in enumerate(y_local_values):
            for i, x_pos in enumerate(x_local_values):
                predicted_rssi_values[j][i],predicted_rssi_variance[j][i]=self.model.predict(np.array([[x_pos, y_pos]]))
        ap_pos = unravel_index(predicted_rssi_values.argmax(), predicted_rssi_values.shape)
        print(predicted_rssi_values[ap_pos[0]][ap_pos[1]])
        AP_position = [0,0]
        AP_position[0]=((ap_pos[1]*self.resolution)+self.dimensions["robot"+str(robot_number+1)][0][0])#- self.initial_positions["robot"+str(robot_number+1)][0]
        AP_position[1]=((ap_pos[0]*self.resolution)+self.dimensions["robot"+str(robot_number+1)][1][0])#- self.initial_positions["robot"+str(robot_number+1)][1]

        return AP_position

# for i in range(3):
#     robot_gp_model = GP_Model()
#     robot_gp_model.initialize_local_gp(i)
#     robot_gp_model.save_model("robot"+str(i+1)+"_40")

# global_gp_model = GP_Model()
# global_gp_model.initialize_global_gp()
# global_gp_model.save_model("global_40")

### Testing
# mean, variance = robot_gp_model.predict(np.array([[0,-0.9]]))
# print("Before", mean, variance)
# robot_gp_model.update_local_gp_with_samples(0)
# mean, variance = robot_gp_model.predict(np.array([[0,-0.9]]))
# print("After", mean, variance)
# x_local_values = [np.arange(-1.8,1.9,0.1)]
# y_local_values = [np.arange(-2.2,0.3,0.1)]
# def calculate_local_rssi():
#     rssi_values = np.zeros((int(robot1_gp_model.local_grid_size[1]/robot1_gp_model.resolution)+2, int(robot1_gp_model.local_grid_size[0]/robot1_gp_model.resolution)+2))
#     for j, y_pos in enumerate(y_local_values[0]):
#         for i, x_pos in enumerate(x_local_values[0]):
#             rssi_values[j][i],_=robot1_gp_model.predict(np.array([[x_pos, y_pos]]))
#             print(x_pos,y_pos,rssi_values[j][i])
#     return rssi_values
# calculate_local_rssi()


# # for x_pos in np.arange(-1.6,1.6,0.1):
# mean, variance = global_gp_model.predict(np.array([[0, 0]]))
# print(mean, variance)
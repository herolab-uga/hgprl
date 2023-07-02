import time
import GPy
import numpy as np
import random
import math
import pickle


class GP_Model:

    source_positions ={"global":(0,0),"robot1":(0,-9),"robot2":(14,7),"robot3":(-16,8)}
    global_grid_size = (3.2,2)
    local_grid_size = (3.6,2.4)
    dimensions ={"global":((-1.6,1.6),(-1,1)),"robot1":((-1.8,1.8),(-2.2,0.2)),"robot2":((-0.2,3.4),(-0.2,2.2)),"robot3":((-3.4,0.2),(-0.2,2.2))}
    resolution=0.4
    model = None
    # distance Calculation
    def dist(self,x, y, pos):
        return math.sqrt(((pos[0]-x)**2) + ((pos[1]-y)**2))
    # RSSI signal generation at pos(x,y) using path-loos model
    def gen_wifi(self,freq=2.4, power=20, trans_gain=0, recv_gain=0, size=global_grid_size, pos=(0,0),grid_pos=(0,0),source_position=(0,0), shadow_dev=2, n=3,noise=2):
        if pos is None:
            pos = (random.randrange(size[0]), random.randrange(size[1]))

        # random.seed(datetime.now())
        rss0 = power + trans_gain + recv_gain + 20 * math.log10(3 / (4 * math.pi * freq * 10))
        rss0=rss0-noise*random.random()
        normal_dist = np.random.normal(0, shadow_dev, size=[int(size[1]/self.resolution)+2, int(size[0]/self.resolution)+2])
        # random.seed(datetime.now())
        # print(source_position)

        distance = self.dist(source_position[0], source_position[1], (pos[0],pos[1]))
        if distance < 0.005:
            return rss0
        val =rss0 - ((10 * n * math.log10(distance)) + normal_dist[grid_pos[0]][grid_pos[1]])
        rss = val-noise*random.random()
        # if rss >  rss0:
        #     return rss0
        return rss

    # Load your RSSI data and corresponding features
    # def load_local_rssi_data(self,robot):
    #     # Load your dataset here, for example using pandas, numpy, etc.
    #     # The dataset should have three columns: feature1, feature2, and RSSI values.
    #     # For this example, let's assume you have loaded the data into two numpy arrays: X_data and y_data
    #     x_min,x_max = self.dimensions["robot"+str(robot+1)][0]
    #     y_min,y_max = self.dimensions["robot"+str(robot+1)][1]
    #     X_data = np.vstack((np.random.uniform(x_min, x_max, (1000, 1)).T,np.random.uniform(y_min, y_max, (1000, 1)).T)).T
    #     rssi_values = np.zeros((1000,1))
    #     for i, pos in enumerate(X_data):
    #         x = int(abs(x_min-pos[0])/self.resolution)
    #         y = int(abs(y_min-pos[1])/self.resolution)
    #         rssi_values[i]=self.gen_wifi(size=self.local_grid_size,pos=((pos[0]*10),(pos[1]*10)),grid_pos=(y,x),source_position=self.source_positions["robot"+str(robot+1)])
    #         # print(pos[0],pos[1],rssi_values[i])
    #     # y_data = np.random.uniform(-90, -40, (1000, 1))

    #     return X_data, rssi_values

    def load_local_rssi_data(self, robot):
        x_min, x_max = self.dimensions["robot" + str(robot + 1)][0]
        y_min, y_max = self.dimensions["robot" + str(robot + 1)][1]
        x_values = np.linspace(x_min, x_max, num=int((x_max - x_min) / self.resolution) + 1)
        y_values = np.linspace(y_min, y_max, num=int((y_max - y_min) / self.resolution) + 1)

        X_data = np.array([[x, y] for y in y_values for x in x_values])
        rssi_values = np.zeros((len(X_data), 1))

        for i, pos in enumerate(X_data):
            x = int(abs(x_min - pos[0]) / self.resolution)
            y = int(abs(y_min - pos[1]) / self.resolution)
            rssi_values[i] = self.gen_wifi(size=self.local_grid_size, pos=((pos[0] * 10), (pos[1] * 10)),
                                        grid_pos=(y, x), source_position=self.source_positions["robot" + str(robot + 1)])
            # print(pos,rssi_values[i])

        return X_data, rssi_values
    # Load your RSSI data and corresponding features
    def load_global_rssi_data(self):
        # Load your dataset here, for example using pandas, numpy, etc.
        # The dataset should have three columns: feature1, feature2, and RSSI values.
        # For this example, let's assume you have loaded the data into two numpy arrays: X_data and y_data
        x_min,x_max = self.dimensions["global"][0]
        y_min,y_max = self.dimensions["global"][1]
        x_values = np.linspace(x_min, x_max, num=int((x_max - x_min) / self.resolution) + 1)
        y_values = np.linspace(y_min, y_max, num=int((y_max - y_min) / self.resolution) + 1)

        X_data = np.array([[x, y] for y in y_values for x in x_values])
        rssi_values = np.zeros((len(X_data), 1))

        for i, pos in enumerate(X_data):
            x = int(abs(x_min - pos[0]) / self.resolution)
            y = int(abs(y_min - pos[1]) / self.resolution)
            rssi_values[i] = self.gen_wifi(size=self.local_grid_size, pos=((pos[0] * 10), (pos[1] * 10)),
                                        grid_pos=(y, x), source_position=self.source_positions["global"])

        return X_data, rssi_values
    def update_local_gp_with_samples(self, robot, num_samples=10):
        X_data, rssi_values = self.load_local_rssi_data(robot)
        sample_indices = np.random.choice(len(X_data), num_samples, replace=False)

        for i in sample_indices:
            x_sample = X_data[i].reshape(1, -1)
            y_sample = rssi_values[i].reshape(1, -1)
            print(x_sample,y_sample)

            # Update the GP model with the new sample
            self.model.set_XY(np.vstack([self.model.X, x_sample]), np.vstack([self.model.Y, y_sample]))
            self.model.optimize()
    def update_global_gp(self,pos):
        x_min,x_max = self.dimensions["global"][0]
        y_min,y_max = self.dimensions["global"][1]
        x = int(abs(x_min - pos[0]) / self.resolution)
        y = int(abs(y_min - pos[1]) / self.resolution)
        rssi = self.gen_wifi(size=self.local_grid_size, pos=((pos[0] * 10), (pos[1] * 10)),
                                        grid_pos=(y, x), source_position=self.source_positions["global"])
        self.model.set_XY(np.vstack([self.model.X, [pos]]), np.vstack([self.model.Y, [rssi]]))
        # print(pos,"global",rssi)
        self.model.optimize()
    def update_local_gp(self,pos,robot):
        x_min,x_max = self.dimensions["robot" + str(robot + 1)][0]
        y_min,y_max = self.dimensions["robot" + str(robot + 1)][1]
        x = int(abs(x_min - pos[0]) / self.resolution)
        y = int(abs(y_min - pos[1]) / self.resolution)
        rssi = self.gen_wifi(size=self.local_grid_size, pos=((pos[0] * 10), (pos[1] * 10)),
                                        grid_pos=(y, x), source_position=self.source_positions["robot" + str(robot + 1)])
        self.model.set_XY(np.vstack([self.model.X, [pos]]), np.vstack([self.model.Y, [rssi]]))
        # print(pos,robot,rssi)
        self.model.optimize()
    def initialize_local_gp(self,robot):
        X, y = self.load_local_rssi_data(robot)
        # Initialize kernel and training data
        sigma_f, l = np.var(y), 2
        kernel = GPy.kern.RBF(2, sigma_f, l)  # Change input_dim to 2

        # Train the GP model
        self.model = GPy.models.GPRegression(X, y, kernel)
        self.model.optimize()
    def initialize_global_gp(self):
        X, y = self.load_global_rssi_data()
        # Initialize kernel and training data
        sigma_f, l = np.var(y), 2
        kernel = GPy.kern.RBF(2, sigma_f, l)  # Change input_dim to 2

        # Train the GP model
        self.model = GPy.models.GPRegression(X, y, kernel)
        self.model.optimize()
    def predict(self,feature):
        # Make a prediction for a new 2D feature (e.g., location)
        mean, variance =self.model.predict(feature)
        return mean,variance
    def save_model(self,name):
        with open('models/'+name+'_pretrained_gp.pkl', 'wb') as file:
            pickle.dump(self.model, file)
    def load_model(self,name):
        with open('models/'+name+'_pretrained_gp.pkl', 'rb') as file:
            self.model = pickle.load(file)

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
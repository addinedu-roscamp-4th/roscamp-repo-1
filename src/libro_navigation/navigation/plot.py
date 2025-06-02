import matplotlib.pyplot as plt
import numpy as np


class Graph():
    def __init__(self):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(1,1,1)
        # self.ax.plot(, np.zeros(n), 'r', label='reference')
        self.x_list = []
        self.y_list = []

    def add_point(self, x, y):
        self.x_list.append(x)
        self.y_list.append(y)


    def show(self, ref_theta):
        self.ax.plot(self.x_list, self.y_list, 'b', label='PID controller')
        n = len(self.y_list)
        self.ax.plot(self.x_list, np.full(n, ref_theta), 'r', label='reference')
        plt.show()

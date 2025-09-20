import numpy as np
import matplotlib.pyplot as plt


class BasicVisualizer:
    def __init__(self, t0=0, t1=4, samples=1000):
        self.t = np.linspace(t0, t1, samples)
        self.fig, self.ax = plt.subplots()
        self.ax.legend()
        self.ax.grid()

    def lam(self, t):
        return 5 * np.sin( 2 * np.pi * 1 * t )
    
    def h(self, t):
        return 3 * np.pi * np.exp(-self.lam ( t ) ) 

    def plot_basic(self):
        self.ax.plot(self.t, self.h(self.t), label='h(t)')
        plt.show()


class StaticVisualizer(BasicVisualizer):
    pass


class LiveVisualizer(BasicVisualizer):
    def __init__(self):
        super().__init__() 
        self.datapoints = []
        # Create a single line (Line2D) to then update with set_data
        # Inspired by https://www.geeksforgeeks.org/python/dynamically-updating-plot-in-matplotlib/
        self.x = []
        self.y = []
        self.line = self.ax.plot(self.x, self.y, label='h(t)')[0]

    def plot_live(self, t_k):
        self.datapoints.append(t_k)
        t_arr = np.asarray(self.datapoints)

        self.line.set_data(t_arr, self.h(t_arr))
        self.ax.relim()
        self.ax.autoscale_view()
        plt.pause(1/30)  # renders at ~30fps
        


def sample_live_data_points(LiveVisualizer):
    t_k = float(0)
    print(np.linspace(0,10,700))
    while True:

        t_k += 1/70
        for t_k in np.linspace(0,10,700):
            LiveVisualizer.plot_live(t_k)
        plt.show()

if __name__ == '__main__':
    basic = BasicVisualizer(0, 4, 100)
    basic.plot_basic()

    live = LiveVisualizer()
    sample_live_data_points(live)
    
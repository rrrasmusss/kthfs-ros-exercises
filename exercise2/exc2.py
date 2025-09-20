import numpy as np
import matplotlib.pyplot as plt


class BasicVisualizer:
    def __init__(self, t0=0, t1=4, samples=1000):
        self.t = np.linspace(t0, t1, samples)
        self.fig, self.ax = plt.subplots()
        # Label axes
        self.ax.grid()

    def lam(self, t):
        return 5 * np.sin(2 * np.pi * 1 * t)

    def h(self, t):
        return 3 * np.pi * np.exp(-self.lam(t))

    def plot_basic(self):
        self.ax.plot(self.t, self.h(self.t), label='h(t)')
        plt.show()


class LiveVisualizer(BasicVisualizer):
    def __init__(self):
        super().__init__()
        self.datapoints = []
        # Create a single line (Line2D) to then update with set_data
        # Inspired by
        # https://www.geeksforgeeks.org/python/dynamically-updating-plot-in-matplotlib/
        self.x = []
        self.y = []
        self.line = self.ax.plot(self.x, self.y)[0]

    def update_line(self, t_k):
        self.datapoints.append(t_k)
        t_arr = np.asarray(self.datapoints)
        # Appends new x and y to line object
        self.line.set_data(t_arr, self.h(t_arr))
        # Adjust limits and autoscales view
        self.ax.relim()
        self.ax.autoscale_view()

    def sample_run_live_loop(self):
        t_k = 0
        dt = 1/100  # delta of which t increments further
        p = 1/30  # ~30 updates per second
        # ion: Interactive mode, enables iterative addition of t_k
        # => automatically redraws when elements are changed.
        plt.ion()
        plt.show()

        # while the figure still exists
        # (from StackOverflow: https://tinyurl.com/4bczkf8s)
        while plt.fignum_exists(self.fig.number):
            t_k += dt
            self.update_line(t_k)
            plt.pause(p)

        plt.ioff()  # Interactive mode set to off.
        """
        Sidenote: probably better to use Animator,
        didn't have time to understand it
        """


class FrequencyInspector(BasicVisualizer):
    def __init__(self):
        self.Fs = 250  # sampling frequency
        self.N = 1000  # number of samples

        self._tstep = 1 / self.Fs  # Period
        self.t = np.linspace(0, (self.N-1)*self._tstep, self.N)  # time steps
        self.y = self.h(self.t)

        self._fstep = self.Fs / self.N
        self.f = np.linspace(0, (self.N-1)*self._fstep, self.N)  # freq steps

    def fourier_transform(self):
        fig, [ax1, ax2] = plt.subplots(nrows=2, ncols=1)
        X = np.fft.fft(self.y)  # Series of complex numbers
        X_mag = np.abs(X) / self.N  # Magnitude normalized by number of samples

        f_plot = self.f[0:int(self.N/2+1)]  # from 0 to Nyqvist
        X_mag_plot = 2 * X_mag[0:int(self.N/2+1)]
        # DC component doesn't need to multiply by 2
        X_mag_plot[0] = X_mag_plot[0] / 2  # otherwise wrong DC magnitude

        ax1.plot(self.t, self.y, '.-')
        ax2.plot(f_plot, X_mag_plot, '.-')
        ax1.set_xlabel('time (s)')
        ax1.set_ylabel('h(t)')
        ax2.set_xlabel('freq (Hz)')
        ax2.set_ylabel('magnitude')
        ax1.grid()
        ax2.grid()
        ax1.set_xlim(0, self.t[-1])
        ax2.set_xlim(0, f_plot[-1])
        plt.tight_layout()

        plt.show()


if __name__ == '__main__':
    basic = BasicVisualizer()
    basic.plot_basic()

    fourier = FrequencyInspector()
    fourier.fourier_transform()

    live = LiveVisualizer()
    live.sample_run_live_loop()

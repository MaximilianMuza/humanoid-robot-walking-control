import matplotlib.pyplot as plt
import math
import numpy as np
class MotionController:
    def __init__(self):
        # Constants
        self.total_step_num = 5
        self.t_total = 240
        self.t_double0 = 10
        self.t_double = 10

        # State variables
        self.current_step_num = 0
        self.walking_tick = 0
        self.t_start = 601
        self.t_last = self.total_step_num * self.t_total + self.t_start

        # Initialize temporary variables
        self.temp_lpx = 0
        self.temp_rpx = 0
        self.temp_lpy = 0
        self.temp_rpy = 0
        self.temp_lpz = 0
        self.temp_rpz = 0

        self.lx = []
        self.ly = []
        self.lz = []
        self.rx = []
        self.ry = []
        self.rz = []

    def compute_quintic_coefficients(self, start, end, T):
        # Boundary conditions
        a_matrix = np.array([
            [0, 0, 0, 0, 0, 1],      # f(0)
            [T**5, T**4, T**3, T**2, T, 1],  # f(T)
            [0, 0, 0, 0, 1, 0],      # f'(0)
            [5*T**4, 4*T**3, 3*T**2, 2*T, 1, 0],  # f'(T)
            [0, 0, 0, 2, 0, 0],      # f''(0)
            [20*T**3, 12*T**2, 6*T, 2, 0, 0]   # f''(T)
        ])

        b_matrix = np.array([0, end-start, 0, 0, 0, 0])
        coefficients = np.linalg.solve(a_matrix, b_matrix)

        return coefficients

    def compute_quintic_coefficients_z(self, x0, xT, T):
        # Define matrices based on boundary conditions
        A = np.array([
            [0, 0, 0, 0, 0, 1],
            [T**5, T**4, T**3, T**2, T, 1],
            [0, 0, 0, 0, 1, 0],
            [5*T**4, 4*T**3, 3*T**2, 2*T, 1, 0],
            [0, 0, 0, 2, 0, 0],
            [20*T**3, 12*T**2, 6*T, 2, 0, 0]
        ])
        b = np.array([x0, xT, 0, 0, 0, 0])

        # Calculate coefficients
        coeffs = np.linalg.solve(A, b)
        return coeffs
    def quintic_polynomial(self, t, coeffs):
        a, b, c, d, e, f = coeffs
        return a*t**5 + b*t**4 + c*t**3 + d*t**2 + e*t + f


    def update(self):
        print(self.walking_tick, self.current_step_num)

        if self.current_step_num == 0:
            if self.walking_tick < self.t_start:

                self.temp_lpx = 0
                self.temp_rpx = 0
                self.temp_lpy = 0
                self.temp_rpy = -0.2
                self.temp_lpz = 0
                self.temp_rpz = 0

            elif self.walking_tick < self.t_start + self.t_double0:

                self.temp_lpx = 0
                self.temp_rpx = 0
                self.temp_lpy = 0
                self.temp_rpy = -0.2
                self.temp_lpz = 0
                self.temp_rpz = 0

            elif self.walking_tick < self.t_start + self.t_total - self.t_double:
                T = self.t_total - self.t_double - self.t_double0
                time_elapsed = self.walking_tick - self.t_start - self.t_double0
                coeffs = self.compute_quintic_coefficients(0, 0.2, T)
                self.temp_rpx = self.quintic_polynomial(time_elapsed, coeffs)

                self.temp_lpx = 0
                self.temp_lpy = 0
                self.temp_rpy = -0.2
                self.temp_lpz = 0

                if 0 <= time_elapsed <= T:
                    coeffs = self.compute_quintic_coefficients_z(0, 0.05, T/2)  # since it's symmetric, we only compute for half T
                    if time_elapsed <= T/2:
                        self.temp_rpz = self.quintic_polynomial(time_elapsed, coeffs)
                    else:
                        self.temp_rpz = self.quintic_polynomial(T - time_elapsed, coeffs)
                else:
                    self.temp_rpz = 0

            else:

                self.temp_lpx = 0
                self.temp_rpx = 0.2
                self.temp_lpy = 0
                self.temp_rpy = -0.2
                self.temp_lpz = 0
                self.temp_rpz = 0


        elif self.current_step_num >= self.total_step_num and self.walking_tick > self.t_last:
            T = self.t_total - self.t_double - self.t_double0
            time_elapsed = ((self.walking_tick - self.t_start) % self.t_total) - self.t_double0
            coeffs = self.compute_quintic_coefficients(-0.2, 0, T)

            if self.walking_tick < self.t_last + self.t_total:
                self.temp_lpx = self.quintic_polynomial(time_elapsed, coeffs) - 0.2
                if 0 <= time_elapsed <= T:
                    coeffs = self.compute_quintic_coefficients_z(0, 0.05, T/2)  # since it's symmetric, we only compute for half T
                    if time_elapsed <= T/2:
                        self.temp_lpz = self.quintic_polynomial(time_elapsed, coeffs)
                    else:
                        self.temp_lpz = self.quintic_polynomial(T - time_elapsed, coeffs)
                else:
                    self.temp_lpz = 0

            else:
                self.temp_lpx = 0

            self.temp_rpx = 0
            self.temp_lpy = 0.2
            self.temp_rpy = 0
            self.temp_rpz = 0



        else:
            temp_walk = (self.walking_tick - self.t_start) % self.t_total
            if temp_walk < self.t_double0:


                if self.current_step_num % 2 == 0: # right
                    self.temp_lpx = 0
                    self.temp_rpx = -0.2
                    self.temp_lpy = 0
                    self.temp_rpy = -0.2
                    self.temp_lpz = 0
                    self.temp_rpz = 0
                else:
                    self.temp_lpx = -0.2
                    self.temp_rpx = 0
                    self.temp_lpy = 0.2
                    self.temp_rpy = 0
                    self.temp_lpz = 0
                    self.temp_rpz = 0

            elif temp_walk < self.t_total - self.t_double:

                T = self.t_total - self.t_double - self.t_double0
                time_elapsed = ((self.walking_tick - self.t_start) % self.t_total) - self.t_double0
                coeffs = self.compute_quintic_coefficients(-0.2, 0.2, T)


                if self.current_step_num % 2 == 0:
                    self.temp_lpx = 0
                    self.temp_rpx = self.quintic_polynomial(time_elapsed, coeffs) - 0.2
                    self.temp_lpy = 0
                    self.temp_rpy = -0.2
                    self.temp_lpz = 0
                    if 0 <= time_elapsed <= T:
                        coeffs = self.compute_quintic_coefficients_z(0, 0.05, T/2)  # since it's symmetric, we only compute for half T
                        if time_elapsed <= T/2:
                            self.temp_rpz = self.quintic_polynomial(time_elapsed, coeffs)
                        else:
                            self.temp_rpz = self.quintic_polynomial(T - time_elapsed, coeffs)
                    else:
                        self.temp_rpz = 0
                    
                else:
                    self.temp_lpx = self.quintic_polynomial(time_elapsed, coeffs) - 0.2
                    self.temp_rpx = 0
                    self.temp_lpy = 0.2
                    self.temp_rpy = 0
                    if 0 <= time_elapsed <= T:
                        coeffs = self.compute_quintic_coefficients_z(0, 0.05, T/2)  # since it's symmetric, we only compute for half T
                        if time_elapsed <= T/2:
                            self.temp_lpz = self.quintic_polynomial(time_elapsed, coeffs)
                        else:
                            self.temp_lpz = self.quintic_polynomial(T - time_elapsed, coeffs)
                    else:
                        self.temp_lpz = 0

                    self.temp_rpz = 0
                    

            else:
                
                if self.current_step_num % 2 == 0: # right moves
                    self.temp_lpx = 0
                    self.temp_rpx = 0.2
                    self.temp_lpy = 0
                    self.temp_rpy = -0.2
                    self.temp_lpz = 0
                    self.temp_rpz = 0
                else:
                    self.temp_lpx = 0.2
                    self.temp_rpx = 0
                    self.temp_lpy = 0.2
                    self.temp_rpy = 0
                    self.temp_lpz = 0
                    self.temp_rpz = 0


        self.walking_tick += 1
        tick_temp = max(0,self.walking_tick - self.t_start)
        self.current_step_num = round(tick_temp / self.t_total)

    def plot_data(self, data_x, data_y, title):
        ticks = list(range(2200))
        ticks = [x * 0.005 for x in ticks]

        # Plot data_x and data_y
        plt.figure(figsize=(8, 4))
        plt.plot(ticks, data_x, 'b--', label="Left foot")  # red line
        plt.plot(ticks, data_y, 'r-', label="Right foot")  # blue dotted line
        plt.xlabel('Time [sec]')
        plt.ylabel('Displacement [m]')
        plt.title(title)
        plt.legend()
        plt.show()
        plt.savefig('{}'.format(title))



if __name__ == "__main__":
    controller = MotionController()

    # Simulate the motion by updating the controller in a loop
    for tick in range(2200):
        controller.update()
        controller.lx.append(controller.temp_lpx)
        controller.ly.append(controller.temp_lpy)
        controller.lz.append(controller.temp_lpz)
        controller.rx.append(controller.temp_rpx)
        controller.ry.append(controller.temp_rpy)
        controller.rz.append(controller.temp_rpz)

controller.plot_data(controller.lx, controller.rx, 'X direction')
controller.plot_data(controller.ly, controller.ry,  'Y direction')
controller.plot_data(controller.lz, controller.rz, 'Z direction')
    # Print or use the values of temp_lpx, temp_rpx, temp_lpy, temp_rpy, temp_lpz, temp_rpz as needed.
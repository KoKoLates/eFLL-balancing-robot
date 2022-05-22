import numpy as np
import skfuzzy as fuzzy
import skfuzzy.control as fuzzy_ctrl
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class Fuzzy:
    def __init__(self):
        self.error_function = [[-20, -20, -15, 0],
                               [-15, 0, 15],
                               [0, 15, 20, 20]]

        self.power_function = [[-255, -255, -200, 0],
                               [-200, 0, 200],
                               [0, 200, 255, 255]]

        self.rate_error_function = [[-200, -200, -180, 0],
                                    [-180, 0, 180],
                                    [0, 180, 200, 200]]

        # Adjust the range after calibration and testing the mpu
        # The values below is alternative
        error_range = np.arange(-20, 20, 0.5, np.float32)
        power_range = np.arange(-255, 255, 1, np.float32)
        rate_error_range = np.arange(-200, 200, 1, np.float32)

        self.error = fuzzy_ctrl.Antecedent(error_range, 'error')
        self.power = fuzzy_ctrl.Consequent(power_range, 'power')
        self.rate_error = fuzzy_ctrl.Antecedent(rate_error_range, 'rate_error')

        self.error['N'] = fuzzy.trapmf(error_range, self.error_function[0])
        self.error['Z'] = fuzzy.trimf(error_range, self.error_function[1])
        self.error['P'] = fuzzy.trapmf(error_range, self.error_function[2])

        self.rate_error['N'] = fuzzy.trapmf(rate_error_range, self.rate_error_function[0])
        self.rate_error['Z'] = fuzzy.trimf(rate_error_range, self.rate_error_function[1])
        self.rate_error['P'] = fuzzy.trapmf(rate_error_range, self.rate_error_function[2])

        self.power['N'] = fuzzy.trapmf(power_range, self.power_function[0])
        self.power['Z'] = fuzzy.trimf(power_range, self.power_function[1])
        self.power['P'] = fuzzy.trapmf(power_range, self.power_function[2])

        n_rule = fuzzy_ctrl.Rule((self.error['N'] & self.rate_error['N']) |
                                 (self.error['Z'] & self.rate_error['N']) |
                                 (self.error['N'] & self.rate_error['Z']), self.power['N'], 'N')

        z_rule = fuzzy_ctrl.Rule((self.error['P'] & self.rate_error['N']) |
                                 (self.error['Z'] & self.rate_error['Z']) |
                                 (self.error['N'] & self.rate_error['P']), self.power['Z'], 'Z')

        p_rule = fuzzy_ctrl.Rule((self.error['P'] & self.rate_error['Z']) |
                                 (self.error['P'] & self.rate_error['P']) |
                                 (self.error['Z'] & self.rate_error['P']), self.power['P'], 'P')

        self.power.defuzzify_method = 'centroid'
        system = fuzzy_ctrl.ControlSystem([n_rule, z_rule, p_rule])
        simulation = fuzzy_ctrl.ControlSystemSimulation(system)

        def fun(x, y):
            simulation.input['error'] = x
            simulation.input['rate_error'] = y
            simulation.compute()
            z = simulation.output['power']
            return z

        fig_3D = plt.figure()
        ax = Axes3D(fig_3D)
        X, Y = np.meshgrid(error_range, rate_error_range)
        Z = fun(X, Y)
        ax.plot_surface(X, Y, Z, rstride=1, cstride=1, cmap=plt.cm.coolwarm)
        ax.set_xlabel("temperature error", color='g')
        ax.set_ylabel("moisture", color='r')
        ax.set_zlabel("time", color='b')

        plt.tight_layout()
        plt.show()


if __name__ == '__main__':
    fuzzy = Fuzzy()


import numpy as np
import skfuzzy as fuzzy
import skfuzzy.control as fuzzy_ctrl
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class System:
    def __init__(self):
        angle_function = [[-20, -20, -15, -5],
                          [-15, -0.5, -0.15],
                          [-5, 0.15, 0.15, 5],
                          [0.15, 5, 15],
                          [5, 15, 20, 20]]

        dangle_function = [[-80, -80, -50, -5],
                           [-50, -5, -0.5],
                           [-5, -0.5, 0.5, 5],
                           [0.5, 5, 50],
                           [5, 50, 80, 80]]

        power_function = [[-255, -255, -60],
                          [-160, -70, -5],
                          [-60, -5, 5, 60],
                          [5, 70, 160],
                          [60, 255, 255]]

        angle_range = np.arange(-20, 20, 0.1, np.float32)
        power_range = np.arange(-255, 255, 1, np.float32)
        dangle_range = np.arange(-80, 80, 0.5, np.float32)

        angle = fuzzy_ctrl.Antecedent(angle_range, 'angle')
        power = fuzzy_ctrl.Consequent(power_range, 'power')
        dangle = fuzzy_ctrl.Antecedent(dangle_range, 'dangle')

        angle['VN'] = fuzzy.trapmf(angle_range, angle_function[0])
        angle['LN'] = fuzzy.trimf(angle_range, angle_function[1])
        angle['Z'] = fuzzy.trapmf(angle_range, angle_function[2])
        angle['LP'] = fuzzy.trimf(angle_range, angle_function[3])
        angle['VP'] = fuzzy.trapmf(angle_range, angle_function[4])

        dangle['VN'] = fuzzy.trapmf(dangle_range, dangle_function[0])
        dangle['LN'] = fuzzy.trimf(dangle_range, dangle_function[1])
        dangle['Z'] = fuzzy.trapmf(dangle_range, dangle_function[2])
        dangle['LP'] = fuzzy.trimf(dangle_range, dangle_function[3])
        dangle['VP'] = fuzzy.trapmf(dangle_range, dangle_function[4])

        power['VN'] = fuzzy.trimf(power_range, power_function[0])
        power['LN'] = fuzzy.trimf(power_range, power_function[1])
        power['Z'] = fuzzy.trapmf(power_range, power_function[2])
        power['LP'] = fuzzy.trimf(power_range, power_function[3])
        power['VP'] = fuzzy.trimf(power_range, power_function[4])

        vn = fuzzy_ctrl.Rule((angle['VN'] & dangle['VN']) |
                             (angle['VN'] & dangle['LN']) |
                             (angle['VN'] & dangle['Z']) |
                             (angle['LN'] & dangle['VN']) |
                             (angle['LN'] & dangle['LN']) |
                             (angle['Z'] & dangle['VN']), power['VN'], 'VN')

        ln = fuzzy_ctrl.Rule((angle['VN'] & dangle['LP']) |
                             (angle['LN'] & dangle['Z']) |
                             (angle['Z'] & dangle['LN']) |
                             (angle['LP'] & dangle['VN']), power['LN'], 'LN')

        z = fuzzy_ctrl.Rule((angle['VN'] & dangle['VP']) |
                            (angle['LN'] & dangle['LP']) |
                            (angle['Z'] & dangle['Z']) |
                            (angle['LP'] & dangle['LN']) |
                            (angle['VP'] & dangle['VN']), power['Z'], 'Z')

        lp = fuzzy_ctrl.Rule((angle['LN'] & dangle['VP']) |
                             (angle['Z'] & dangle['LP']) |
                             (angle['LP'] & dangle['Z']) |
                             (angle['VP'] & dangle['LN']), power['LP'], 'LP')

        vp = fuzzy_ctrl.Rule((angle['VP'] & dangle['VP']) |
                             (angle['VP'] & dangle['LP']) |
                             (angle['VP'] & dangle['Z']) |
                             (angle['LP'] & dangle['VP']) |
                             (angle['LP'] & dangle['LP']) |
                             (angle['Z'] & dangle['VP']), power['VP'], 'VP')


        power.defuzzify_method = 'centroid'
        system = fuzzy_ctrl.ControlSystem([vn, ln, z, lp, vp])
        simulation = fuzzy_ctrl.ControlSystemSimulation(system)

        def fun(x, y):
            simulation.input['angle'] = x
            simulation.input['dangle'] = y
            simulation.compute()
            z = simulation.output['power']
            return z

        fig_3D = plt.figure()
        ax = Axes3D(fig_3D)
        X, Y = np.meshgrid(angle_range, dangle_range)
        Z = fun(X, Y)
        ax.plot_surface(X, Y, Z, rstride=1, cstride=1, cmap=plt.cm.coolwarm)
        ax.set_xlabel("angle", color='g')
        ax.set_ylabel("angle change", color='r')
        ax.set_zlabel("pwm", color='b')

        plt.tight_layout()
        plt.show()


if __name__ == '__main__':
    fuzzy = System()


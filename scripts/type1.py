import numpy as np
import skfuzzy as fuzzy
import skfuzzy.control as fuzzy_ctrl

from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class Type1:
    def __init__(self):
        self.angle_range = np.arange(-45, 66, 1, np.float32)
        self.velocity_range = np.arange(-50, 51, 1, np.float32)
        self.power_range = np.arange(-255, 256, 1, np.float32)
        self.angle_function = [[-45, -45, 45], [-45, 45, 45]]
        self.velocity_function = [[-50, -50, 50], [-50, 50, 50]]
        self.power_function = [[-255, -255, 0], [-255, 0, 255], [0, 255, 255]]

        angle = fuzzy_ctrl.Antecedent(self.angle_range, 'angle')
        power = fuzzy_ctrl.Consequent(self.power_range, 'power')
        velocity = fuzzy_ctrl.Antecedent(self.velocity_range, 'velocity')

        angle['N'] = fuzzy.trimf(self.angle_range, self.angle_function[0])
        angle['P'] = fuzzy.trimf(self.angle_range, self.angle_function[1])
        velocity['N'] = fuzzy.trimf(self.velocity_range, self.velocity_function[0])
        velocity['P'] = fuzzy.trimf(self.velocity_range, self.velocity_function[1])
        power['N'] = fuzzy.trimf(self.power_range, self.power_function[0])
        power['Z'] = fuzzy.trimf(self.power_range, self.power_function[1])
        power['P'] = fuzzy.trimf(self.power_range, self.power_function[2])

        n = fuzzy_ctrl.Rule(angle['P'] & velocity['P'], power['N'], 'N')
        z = fuzzy_ctrl.Rule((angle['P'] & velocity['N']) | (angle['N'] & velocity['P']), power['Z'], 'Z')
        p = fuzzy_ctrl.Rule(angle['N'] & velocity['N'], power['P'], 'P')

        power.defuzzify_method = 'centroid'
        system = fuzzy_ctrl.ControlSystem([n, z, p])
        self.simulation = fuzzy_ctrl.ControlSystemSimulation(system)

    def figure_3d(self):
        def fun(x_, y_):
            self.simulation.input['angle'] = x_
            self.simulation.input['velocity'] = y_
            self.simulation.compute()
            z_ = self.simulation.output['power']
            return z_

        fig = plt.figure()
        ax = Axes3D(fig, auto_add_to_figure=False)
        fig.add_axes(ax)
        x, y = np.meshgrid(self.angle_range, self.velocity_range)
        z = fun(x, y)
        ax.plot_surface(x, y, z, rstride=1, cstride=1, cmap=plt.cm.coolwarm)
        ax.set_xlabel("angle", color='g')
        ax.set_ylabel("angle change", color='r')
        ax.set_zlabel("pwm", color='b')
        plt.show()

    def figure(self):
        fig, (angle, velocity, power) = plt.subplots(nrows=3, figsize=(6, 6))
        angle.plot(self.angle_range, fuzzy.trimf(self.angle_range, self.angle_function[0]), 'cornflowerblue', label='N')
        angle.plot(self.angle_range, fuzzy.trimf(self.angle_range, self.angle_function[1]), 'mediumseagreen', label='P')
        angle.legend()

        velocity.plot(self.velocity_range, fuzzy.trimf(self.velocity_range, self.velocity_function[0]), 'cornflowerblue', label='N')
        velocity.plot(self.velocity_range, fuzzy.trimf(self.velocity_range, self.velocity_function[1]), 'mediumseagreen', label='P')
        velocity.legend()

        power.plot(self.power_range, fuzzy.trimf(self.power_range, self.power_function[0]), 'cornflowerblue', label='N')
        power.plot(self.power_range, fuzzy.trimf(self.power_range, self.power_function[1]), 'mediumseagreen', label='Z')
        power.plot(self.power_range, fuzzy.trimf(self.power_range, self.power_function[2]), 'indianred', label='P')
        power.legend()

        for i in (angle, velocity, power):
            i.spines['top'].set_visible(False)
            i.spines['right'].set_visible(False)
            i.get_xaxis().tick_bottom()
            i.get_yaxis().tick_left()

        plt.show()


if __name__ == '__main__':
    fuzz = Type1()
    fuzz.figure_3d()
    fuzz.figure()

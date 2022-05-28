import numpy as np
import skfuzzy as fuzzy
import skfuzzy.control as fuzzy_ctrl
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class System:
    def __init__(self):
        self.angle_range = np.arange(-15, 16, 1, np.float32)
        self.velocity_range = np.arange(-80, 81, 1, np.float32)
        self.power_range = np.arange(-255, 256, 1, np.float32)
        self.angle_function = [[-15, -15, -10, -5], [-12, -8, -2], [-6, -1, 1, 6], [2, 8, 12], [5, 10, 15, 15]]
        self.velocity_function = [[-80, -80, -50, -30], [-70, -45, -10], [-40, -10, 10, 40], [10, 45, 70], [30, 50, 80, 80]]
        self.power_function = [[-255, -255, -50], [-175, -100, -10], [-75, -10, 10, 75], [10, 100, 175], [50, 255, 255]]

        angle = fuzzy_ctrl.Antecedent(self.angle_range, 'angle')
        power = fuzzy_ctrl.Consequent(self.power_range, 'power')
        velocity = fuzzy_ctrl.Antecedent(self.velocity_range, 'velocity')

        angle['NN'] = fuzzy.trapmf(self.angle_range, self.angle_function[0])
        angle['ZN'] = fuzzy.trimf(self.angle_range, self.angle_function[1])
        angle['ZZ'] = fuzzy.trapmf(self.angle_range, self.angle_function[2])
        angle['ZP'] = fuzzy.trimf(self.angle_range, self.angle_function[3])
        angle['PP'] = fuzzy.trapmf(self.angle_range, self.angle_function[4])

        velocity['NN'] = fuzzy.trapmf(self.velocity_range, self.velocity_function[0])
        velocity['ZN'] = fuzzy.trimf(self.velocity_range, self.velocity_function[1])
        velocity['ZZ'] = fuzzy.trapmf(self.velocity_range, self.velocity_function[2])
        velocity['ZP'] = fuzzy.trimf(self.velocity_range, self.velocity_function[3])
        velocity['PP'] = fuzzy.trapmf(self.velocity_range, self.velocity_function[4])

        power['NN'] = fuzzy.trimf(self.power_range, self.power_function[0])
        power['ZN'] = fuzzy.trimf(self.power_range, self.power_function[1])
        power['ZZ'] = fuzzy.trapmf(self.power_range, self.power_function[2])
        power['ZP'] = fuzzy.trimf(self.power_range, self.power_function[3])
        power['PP'] = fuzzy.trimf(self.power_range, self.power_function[4])

        nn = fuzzy_ctrl.Rule((angle['NN'] & velocity['NN']) |
                             (angle['NN'] & velocity['ZN']) |
                             (angle['NN'] & velocity['ZZ']) |
                             (angle['ZN'] & velocity['NN']) |
                             (angle['ZN'] & velocity['ZN']) |
                             (angle['ZZ'] & velocity['NN']), power['NN'], 'NN')

        zn = fuzzy_ctrl.Rule((angle['NN'] & velocity['ZP']) |
                             (angle['ZN'] & velocity['ZZ']) |
                             (angle['ZZ'] & velocity['ZN']) |
                             (angle['ZP'] & velocity['NN']), power['ZN'], 'ZN')

        zz = fuzzy_ctrl.Rule((angle['NN'] & velocity['PP']) |
                             (angle['ZN'] & velocity['ZP']) |
                             (angle['ZZ'] & velocity['ZZ']) |
                             (angle['ZP'] & velocity['ZN']) |
                             (angle['PP'] & velocity['NN']), power['ZZ'], 'ZZ')

        zp = fuzzy_ctrl.Rule((angle['ZN'] & velocity['PP']) |
                             (angle['ZZ'] & velocity['ZP']) |
                             (angle['ZP'] & velocity['ZZ']) |
                             (angle['PP'] & velocity['ZN']), power['ZP'], 'ZP')

        pp = fuzzy_ctrl.Rule((angle['PP'] & velocity['PP']) |
                             (angle['PP'] & velocity['PP']) |
                             (angle['PP'] & velocity['ZZ']) |
                             (angle['ZP'] & velocity['PP']) |
                             (angle['ZP'] & velocity['ZP']) |
                             (angle['ZZ'] & velocity['PP']), power['PP'], 'PP')

        power.defuzzify_method = 'centroid'
        system = fuzzy_ctrl.ControlSystem([nn, zn, zz, zp, pp])
        self.simulation = fuzzy_ctrl.ControlSystemSimulation(system)

    def fig_3d(self):
        def fun(x_, y_):
            self.simulation.input['angle'] = x_
            self.simulation.input['velocity'] = y_
            self.simulation.compute()
            z_ = self.simulation.output['power']
            return z_
        #
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

    def fig(self):
        fig, (angle, velocity, power) = plt.subplots(nrows=3, figsize=(6, 6))
        angle.plot(self.angle_range, fuzzy.trapmf(self.angle_range, self.angle_function[0]), 'b', label='huge')
        angle.plot(self.angle_range, fuzzy.trimf(self.angle_range, self.angle_function[1]), 'g', label='large')
        angle.plot(self.angle_range, fuzzy.trapmf(self.angle_range, self.angle_function[2]), 'r', label='high')
        angle.plot(self.angle_range, fuzzy.trimf(self.angle_range, self.angle_function[3]), 'y', label='low')
        angle.plot(self.angle_range, fuzzy.trapmf(self.angle_range, self.angle_function[4]), 'k', label='few')
        angle.legend()

        velocity.plot(self.velocity_range, fuzzy.trapmf(self.velocity_range, self.velocity_function[0]), 'b', label='arid')
        velocity.plot(self.velocity_range, fuzzy.trimf(self.velocity_range, self.velocity_function[1]), 'g', label='dry')
        velocity.plot(self.velocity_range, fuzzy.trapmf(self.velocity_range, self.velocity_function[2]), 'r', label='normal')
        velocity.plot(self.velocity_range, fuzzy.trimf(self.velocity_range, self.velocity_function[3]), 'y', label='moist')
        velocity.plot(self.velocity_range, fuzzy.trapmf(self.velocity_range, self.velocity_function[4]), 'k', label='wet')
        velocity.legend()

        power.plot(self.power_range, fuzzy.trimf(self.power_range, self.power_function[0]), 'b', label='short')
        power.plot(self.power_range, fuzzy.trimf(self.power_range, self.power_function[1]), 'g', label='medium')
        power.plot(self.power_range, fuzzy.trapmf(self.power_range, self.power_function[2]), 'r', label='long')
        power.plot(self.power_range, fuzzy.trimf(self.power_range, self.power_function[3]), 'g', label='medium')
        power.plot(self.power_range, fuzzy.trimf(self.power_range, self.power_function[4]), 'r', label='long')
        power.legend()

        for i in (angle, velocity, power):
            i.spines['top'].set_visible(False)
            i.spines['right'].set_visible(False)
            i.get_xaxis().tick_bottom()
            i.get_yaxis().tick_left()

        plt.show()


if __name__ == '__main__':
    fuzz = System()
    fuzz.fig_3d()
    fuzz.fig()


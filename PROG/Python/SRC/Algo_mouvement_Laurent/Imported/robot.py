from leg import *

class Robot:
    def __init__(self):
        self.N_legs = 6
        self.minimum_legs_down = 4
        self.Legs= []

        l1 = 1.
        l2 = 1.

        l = 0.7
        L = 3.7

        margin = 0.05
        threshold = 0.1
        landing = 0.15

        alphamax = np.pi/6
        alphamin = -np.pi/6
        alpharepos = 0.
        alphadiff = alphamax - alphamin

        betamax = np.pi/4
        betamin = -np.pi/4
        betarepos = 0.

        gammamax = np.pi/2
        gammamin = 0.
        gammarepos = np.pi/8

        speed_ratio_up_down = 10.
        up_down_height_ratio = 0.1

        colors = ['r', 'b', 'y', 'm', 'g', 'c']

        legs_fix = [[L/2, -l/2], [0, -l/2], [-L/2, -l/2], [L/2, l/2], [0, l/2], [-L/2, l/2]]

        for n_leg in range(self.N_legs):
            if n_leg/(self.N_legs/2) == 0:
                side = 'right'
            else:
                side= 'left'
            self.Legs += [Leg(n_leg, 
                              legs_fix[n_leg], 
                              side, 
                              [alphamin, alphamax, alpharepos], 
                              [betamin, betamax, betarepos],
                              [gammamin, gammamax, gammarepos],
                              l1,
                              l2,
                              landing,
                              threshold,
                              margin,
                              speed_ratio_up_down,
                              up_down_height_ratio,
                              colors[n_leg])]



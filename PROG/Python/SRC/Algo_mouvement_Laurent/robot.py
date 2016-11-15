from leg import *
import numpy as np

class Robot:
    def __init__(self, N_legs = 6, l1 = 1., l2 = 1., l = 0.7, L = 3.7, minimum_legs_down = 4, frm = 0.1, srud = 7., udhr = 0.1):
        # All length units here are in arbitrary units. 
        self.N_legs = N_legs # Number of leg in this model
        self.minimum_legs_down = minimum_legs_down # Minimum number of leg on the ground at all times to ensure bearing
        self.Legs= []

        self.l = l # Width of the body
        self.L = L # Length of the body
        self.frm = frm # Rotation/Movement factor. At 0, the  robot rotates on himself then goes forward. At 1, the robot permenently rotates as it translates to the final point

        self.position = [0., 0.] # Initial position. At the initialisation, the robot is at the center of its landmark
        self.orientation = [1., 0.] # Initial orientation. At the initialisation, the robot is along the x axis

        # Variables defining the history of the robot.
        self.history = []
        self.angles_history = []
        self.feet_positions_history = []

        # Values to define the leg demands zones (see leg.py). Keep the order need < envy (< landing, to be removed).
        # The larger these values, the more safe you are about geetting close to mechanical stops, but the more power consuming this model gets. 
        need = 0.05
        envy = 0.1
        landing = 0.15

        # Definition of allowed and default leg angles. Alpha is the horizontal rotation angle, Beta is the first vertical rotation  angle and gamma is the second one.
        # Xrepos is the default value chosen.
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

        self.srud = srud # Speed ratio up/down (SRUD) defines how fast a leg travels forward when lifted compared to grounded legs. For this model to work, we need rud > N_legs/(N_legs - minimum_legs_down). 
        # It will be mostly constrained by physical capacities of actuators
        self.udhr = udhr # Height to which each leg has to be lifted when not grounded. Should be interfaced with trajectories generation.

        colors = ['r', 'b', 'y', 'm', 'g', 'c'] # Set of color for display

        legs_fix = [[L/2, -l/2], [0, -l/2], [-L/2, -l/2], [L/2, l/2], [0, l/2], [-L/2, l/2]] # Position of each leg on the body or the robot. Also used to define the 'side' of the leg
        for i in range(len(legs_fix)):
            legs_fix[i] = np.array(legs_fix[i])

        # Now we define all the legs with the set of parameters defined here
        for n_leg in range(self.N_legs):
            if legs_fix[n_leg][1] < 0:
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
                              envy,
                              need,
                              srud,
                              udhr,
                              colors[n_leg])]

    def move(self, final_position, N_points = 500):
# Function to move the robot.
# final_position : Desired position as vector [x,y] in the temporary landmark of the robot
# N_points : number of points for this move. Probably useless in the end, necessary for the current algorythm
# Returns an history of the center positions, feet positions, and angles of each leg.

        final_orientation = np.array([final_position[0]-self.position[0], final_position[1]-self.position[1]])
        final_orientation /= norm(final_orientation)

        self.history += [self.position]

        self.angles_history += [[]]
        for leg in self.Legs:
            self.angles_history[-1] += [leg.angles]

        points_center = [0,0]
        points_center[0] = np.linspace(self.position[0], final_position[0], N_points)
        points_center[1] = np.linspace(self.position[1], final_position[1], N_points)
        orientation_x = np.linspace(self.orientation[0], final_orientation[0], int(self.frm*N_points))
        orientation_y = np.linspace(self.orientation[1], final_orientation[1], int(self.frm*N_points))
        orientation_x /= np.sqrt(orientation_x**2+orientation_y**2)
        orientation_y /= np.sqrt(orientation_x**2+orientation_y**2)
        orientation = [0,0]
        orientation[0] = np.array(orientation_x.tolist() + [final_orientation[0] for i in range(N_points - len(orientation_x))])
        orientation[1] = np.array(orientation_y.tolist() + [final_orientation[1] for i in range(N_points - len(orientation_y))])

        self.feet_positions_history += [[]]
        for leg in self.Legs:
            leg.absolute_feet_position = leg.get_leg_absolute_position(self.position, self.orientation)
            print leg.absolute_feet_position
            self.feet_positions_history[-1] += [leg.absolute_feet_position]

        final_feet_positions =[]
        for leg in self.Legs:
            final_feet_positions += [leg.get_leg_absolute_position(final_position, final_orientation)]


        for time in range(1, N_points):
        # We start at time t=1 since t=0 iss the initial position.
            print "Statuses at start of cycle {0} :".format(time)
            for leg in self.Legs:
                print leg.status

            # We set the different history variables for this cycle
            self.angles_history += [[]]
            self.feet_positions_history += [[]]
            # Update position for this cycle and save it
            self.position = np.array([points_center[0][time], points_center[1][time]])
            self.history += [self.position]

            demands = []

            for leg in self.Legs:
                if leg.status == 'down': # If the leg is currently on the ground
                    leg.relative_feet_position = leg.get_leg_relative_position(self.position, self.orientation) # We first update the relative position of the grounded legs
                    leg.update_angles_from_position() # We update the new angles for this relative position
                    self.angles_history[-1] += [leg.angles] 
                    self.feet_positions_history[-1] += [leg.absolute_feet_position]

                    leg_zones = leg.zone_presence() # Now we check the presence of the leg in the different zones
                    if leg_zones['envy'] == True:
                        demands += [None]
                    elif leg_zones['envy'] == False and leg_zones['need'] == True:
                        print "Relative distance from envy to need for leg {1} : {0}".format(leg.get_ratio_distance_from_zone_to_next('envy', 'need'), leg.leg_id)
                        demands += ['envy']
                    elif leg_zones['need'] == False and leg_zones['critical'] == True:
                        print "Relative distance from need to critical for leg {1} : {0}".format(leg.get_ratio_distance_from_zone_to_next('need', 'critical'), leg.leg_id)
                        demands += ['need']
                    elif leg_zones['critical'] == False:
                        print "Leg {0} passed critical boundary !".format(leg.leg_id)
                        sys.exit('Error 2893254')
                    else:
                        print "Unknown leg_zones status. Probable definition issue"
                        sys.exit('Error 5249632')
                elif leg.status == 'up': # If the leg is currently moving in the air, towards a designed position.
                    leg.relative_feet_position = leg.flight
                    self.angles_history[-1] += [leg.angles]
                    hist_angles[-1] += [get_angles(flight_positions[n_leg][t-t_origin[n_leg]][:2], n_leg/3, np.array([points_center_x[t], points_center_y[t]])+rotate(np.array(legs_fix[n_leg]), [orientation_x[t], orientation_y[t]]), [orientation_x[t], orientation_y[t]], flight_positions[n_leg][t-t_origin[n_leg]][2])]

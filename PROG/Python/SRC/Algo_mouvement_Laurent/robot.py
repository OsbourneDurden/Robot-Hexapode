from leg import *
import numpy as np

class Robot:
    def __init__(self, N_legs = 6, l1 = 1., l2 = 1., l = 0.7, L = 3.7, minimum_legs_down = 4, max_sameside_leg_number = 4, frm = 0.1, srud =8., udhr = 0.1, flight_angle = pi/4):
        # All length units here are in arbitrary units. 
        self.N_legs = N_legs # Number of leg in this model

        # RULES DEFINITIONS
        self.minimum_legs_down = minimum_legs_down # Minimum number of leg on the ground at all cycles to ensure bearing
        self.max_sameside_leg_number = max_sameside_leg_number # Max number of leg that can be on the same side

        self.l = l # Width of the body
        self.L = L # Length of the body
        self.frm = frm # Rotation/Movement factor. At 0, the  robot rotates on himself then goes forward. At 1, the robot permenently rotates as it translates to the final point

        self.position = np.array([0., 0., 0.]) # Initial position. At the initialisation, the robot is at the center of its landmark
        self.orientation = np.array([1., 0.]) # Initial orientation. At the initialisation, the robot is along the x axis

        # Variables defining the history of the robot.
        self.history = []
        self.angles_history = []
        self.feet_positions_history = []
        self.landmark_history = [] # List of landmark changes

        self.history += [self.position]
        self.current_landmark = [self.position, self.orientation]
        self.landmark_history += [self.current_landmark]

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

        self.srud = srud # Speed ratio up/down (SRUD) defines how fast a leg travels forward when lifted compared to grounded legs. For this model to work, we need rud > N_legs/(N_legs - minimum_legs_down). False. TODO
        # It will be mostly constrained by physical capacities of actuators
        self.udhr = udhr # Up/Down Height Ratio. Height to which each leg has to be lifted when not grounded compared to the height of the robot. 
        # Should be interfaced with trajectories generation.

        colors = ['r', 'b', 'y', 'm', 'g', 'c'] # Set of color for display

        legs_fix = [[L/2, -l/2], [0, -l/2], [-L/2, -l/2], [L/2, l/2], [0, l/2], [-L/2, l/2]] # Position of each leg on the body or the robot. Also used to define the 'side' of the leg
        for i in range(len(legs_fix)):
            legs_fix[i] = np.array(legs_fix[i])

        self.Legs= []
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
                              flight_angle,
                              frm,
                              colors[n_leg])]

        self.angles_history += [[]]
        for leg in self.Legs:
            self.angles_history[-1] += [leg.angles]

        self.feet_positions_history += [[]]
        for leg in self.Legs:
            leg.absolute_feet_position = leg.get_leg_absolute_position(self.position, self.orientation)
            print leg.absolute_feet_position
            self.feet_positions_history[-1] += [leg.absolute_feet_position]

    def get_relative_point(self, final_absolute_feet_position, final_absolute_robot_position, final_absolute_robot_rotation, leg_to_raise):
	print final_absolute_feet_position, final_absolute_robot_position, final_absolute_robot_rotation, leg_to_raise
        if self.Legs[leg_to_raise].side == 'left':
            return tools.rotate(final_absolute_feet_position - final_absolute_robot_position, final_absolute_robot_rotation) - self.Legs[leg_to_raise].fix_position
        else:
            tmp = tools.rotate(final_absolute_feet_position - final_absolute_robot_position, final_absolute_robot_rotation) - self.Legs[leg_to_raise].fix_position
            return np.array([tmp[0], -tmp[1], tmp[2]])

    def get_relative_orientation(self, final_absolute_orientation, leg_to_raise):
        if self.Legs[leg_to_raise].side == 'right':
            return final_absolute_orientation
        else:
            return np.array([final_absolute_orientation[0], -final_absolute_orientation[1]])

    def get_flight_length(self, leg_to_raise, mean_alpha_move):
        return int((self.Legs[leg_to_raise].alphamax-self.Legs[leg_to_raise].alphamin)/(mean_alpha_move*self.srud))

    def move(self, final_position, N_points = 500, rotation_factor = 0.1):
        '''
        Function to move the robot.
        final_position : Desired position as vector [x,y] in the temporary landmark of the robot
        N_points : number of points for this move. Probably useless in the end, necessary for the current algorythm
        Returns an history of the center positions, feet positions, and angles of each leg.
        '''
        final_position = np.array(final_position)
        final_orientation = final_position-self.position
        final_orientation /= norm(final_orientation)

        points_center = [0,0]
        points_center[0] = np.linspace(self.position[0], final_position[0], N_points)
        points_center[1] = np.linspace(self.position[1], final_position[1], N_points)
        positions = np.array(points_center)
        orientation_x = np.linspace(self.orientation[0], final_orientation[0], int(self.frm*N_points))
        orientation_y = np.linspace(self.orientation[1], final_orientation[1], int(self.frm*N_points))
        orientation_x /= np.sqrt(orientation_x**2+orientation_y**2)
        orientation_y /= np.sqrt(orientation_x**2+orientation_y**2)
        orientations = [0,0]
        orientations[0] = np.array(orientation_x.tolist() + [final_orientation[0] for i in range(N_points - len(orientation_x))])
        orientations[1] = np.array(orientation_y.tolist() + [final_orientation[1] for i in range(N_points - len(orientation_y))])
        orientations=np.array(orientations)

        final_feet_positions =[]
        for leg in self.Legs:
            final_feet_positions += [leg.get_leg_absolute_position(final_position, final_orientation)]


        for cycle in range(1, N_points):
        # We start at cycle 1 since 0 is the initial position.
            print ""
            print "Statuses at start of cycle {0} : {1}".format(cycle, [leg.status for leg in self.Legs])

            # We set the different history variables for this cycle
            self.angles_history += [[]]
            self.feet_positions_history += [[]]
            # Update position for this cycle and save it
            self.position = np.array(positions[:,cycle].tolist()+[0.])
            self.orientation = orientations[:,cycle]
            self.history += [self.position]
            self.landmark_history += [self.current_landmark]

            # The demands define the need for a leg to takeoff, and its value traduces the intensity of the need.
            # 0 means no need, 1 a slight need, 2 a real need.
            demands = []

            mean_alpha_move = 0 # Gives an estimate of the speed of the robot at this timestep. Allows to create previsions for a flight.
            legs_down = 0

            for leg in self.Legs:
                if leg.status == 'down': # If the leg is currently on the ground
                    legs_down += 1
                    leg.relative_feet_position = leg.get_leg_relative_position(self.position, self.orientation) # We first update the relative position of the grounded legs
                    leg.update_angles_from_position() # We update the new angles for this relative position
                    self.angles_history[-1] += [leg.angles] 
                    mean_alpha_move += abs(self.angles_history[-2][-1][0]-self.angles_history[-1][-1][0])
                    self.feet_positions_history[-1] += [leg.absolute_feet_position]

                    leg_zones = leg.zone_presence() # Now we check the presence of the leg in the different zones
                    if leg_zones['envy'] == True:
                        demands += [0]
                    elif leg_zones['envy'] == False and leg_zones['need'] == True:
                        print "Relative distance from envy to need for leg {1} : {0}".format(leg.get_ratio_distance_from_zone_to_next('envy', 'need'), leg.leg_id)
                        demands += [1]
                    elif leg_zones['need'] == False and leg_zones['critical'] == True:
                        print "Relative distance from need to critical for leg {1} : {0}".format(leg.get_ratio_distance_from_zone_to_next('need', 'critical'), leg.leg_id)
                        demands += [1]
                    elif leg_zones['critical'] == False:
                        print "Leg {0} passed critical boundary !".format(leg.leg_id)
                        sys.exit('Error 2893254')
                    else:
                        print "Unknown leg_zones status. Probable definition issue"
                        sys.exit('Error 5249632')
                
                elif leg.status == 'up': # If the leg is currently moving in the air, towards a designed position.
                    if (cycle - leg.cycle_takeoff) <= len(leg.flight)-1:
                        leg.follow_flight(cycle)
                    else:
                        leg.extend_flight()
                    
                    leg.absolute_position = leg.get_leg_absolute_position(self.position, self.orientation)
                    
                    self.feet_positions_history[-1] += [leg.absolute_feet_position]
                    self.angles_history[-1] += [leg.angles] 
                    leg.check_landing(cycle)
                    demands += [0]

                elif leg.status == 'end': # If the leg reached its final position. Basically here we only do data saving
                    legs_down += 1
                    leg.relative_feet_position = leg.get_leg_relative_position(self.position, self.orientation)
                    leg.update_angles_from_position()
                    self.angles_history[-1] += [leg.angles] 
                    self.feet_positions_history[-1] += [leg.absolute_feet_position]

                    demands += [0]
                else:
                    print "Unknown status for leg {0} : {1}".format(leg.leg_id, leg.status)

            # Now, all legs positions have been updated. We now check the demands of each leg, and update the statuses
            mean_alpha_move /= legs_down
            print "{0} legs are on the ground after update of cycle {1}".format(legs_down, cycle)

            if demands.count(0) == self.N_legs:
                print "Demands before alphas condition check are {0}".format(demands)
                # Here, no demand was made, thus all legs are inside the 'envy' zone
                if legs_down > self.minimum_legs_down and (np.array(R.angles_history[-1])[:,0] < 0).sum() > self.max_sameside_leg_number: 
                    # and they are too many legs on the same negative side and too many are on the floor
                    distances = []
                    for leg in self.Legs:
                        if leg.angles[0] < 0 and leg.status == 'down':
                            distances += [leg.get_ratio_distance_from_zone_to_next('center', 'envy')]
                        else:
                            distances += [0]
                    print "Too many legs found backwards. Maximum distance found for leg {0} at value {1}".format(distances.index(max(distances)), max(distances))
                    demands[distances.index(max(distances))] = 1

                elif  (np.array(R.angles_history[0])[:,0] > 0).sum() > self.max_sameside_leg_number:
                    # and they are too many legs on the same positive side, meaning we moved forward too many of them !
                    print "Algorithmic error, too many legs moved forward, can't keep up with the rules !"
                    sys.exit("Error 5874216")
            print "Finals demands for cycle {1} are {0}".format(demands, cycle)
                        
            if sum(demands)>0:
                # If at least one leg asked for takeoff
                if demands.count(max(demands)) == 1:
                    # If the higher demand was asked once, then it is the only priority and it is selected
                    leg_to_raise = demands.index(max(demands))
                else:
                    # Else it means we have a conflict we must solve
                    N_zones=[]
                    for leg in self.Legs:
                        if demands[leg.leg_id] == max(demands):
                            N_zones += [leg.zone_presence().values().count(True)]
                        else:
                            N_zones += [10]
                    if N_zones.count(min(N_zones)) == 1:
                        # Here we look for the minimum of zone presence. A leg only inside critical boundaries will have N_zones=1. If this minimum is reached only once, then we select it
                        leg_to_raise = N_zones.index(min(N_zones))
                    else:
                        # If once again, we have a conflict, we must go one step further and check the maximum ratio distance
                        ratios = []
                        for leg in self.Legs:
                            if N_zones[leg.leg_id] == max(N_zones):
                                # If this leg was asking to go up
                                if not leg.zone_presence()['need']:
                                    # If it is between need and critical (thus it is not inside the "need" boundaries anymore)
                                    ratios += [leg.get_ratio_distance_from_zone_to_next('need', 'critical')]
                                else:
                                    if not leg.zone_presence()['envy']:
                                        # If it is between envy and need (thus it is not inside the "need" boundaries anymore)
                                        ratios += [leg.get_ratio_distance_from_zone_to_next('envy', 'need')]
                                    else:
                                        ratios += [leg.get_ratio_distance_from_zone_to_next('center', 'envy')]
                            else:
                                ratios += [0]
                        leg_to_raise = ratios.index(max(ratios))
                    #self.schedule_flight(demands.index(max(demands)), points_center, orientation, mean_alpha_move, cycle)
            else:
                leg_to_raise = None

            if leg_to_raise == None:
                print "No leg to be raised at the end of cycle {0}".format(cycle)
            else:
                print "Leg {0} is going to be raised at the end of cycle {1}".format(leg_to_raise, cycle)
                N_points = self.get_flight_length(leg_to_raise, mean_alpha_move)
                self.Legs[leg_to_raise].initiate_flight(cycle, self.get_relative_point(final_feet_positions[leg_to_raise], positions[:,cycle+N_points], orientations[:,cycle+N_points], leg_to_raise), self.get_relative_orientation(orientations[:,cycle+N_points], leg_to_raise), N_points)

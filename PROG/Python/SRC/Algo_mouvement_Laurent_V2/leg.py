import numpy as np
import sys
import math
import tools
import matplotlib.pylab as pyl

# The leg class is defined as something static. We don't consider any movement of the robot in here, thus all input and output will be in its self referential :
# The landmark is located of the fix_position of the leg. Thus, if the feet if on the ground, the z position is most likely negative.
#
#           [0., 0.]    
#               o       
#               |     Z O----> 
#		|       |    y 
#               |       |
#               |       |
#		|	v x
#          _____|____
#         /     |R   \
#        /      |     \        
#       /       o      \        
#      /                \        
#     /                  \        
#    /____________________\

# Zones are defined this way
#          ___________________________________________________
#          |        _________________________________        |
#          |        |        _______________        |        |
#          |        |        |             |        |        |    
# CRITICAL |  NEED  |  ENVY  | TRANQUILITY |  ENVY  |  NEED  | CRITICAL
#          |        |        |_____________|        |        |    
#          |        |_______________________________|        |
#          |_________________________________________________|


class Leg:
    def __init__(self, leg_id, neighbours, fix_position, fix_angle, side, alpha_data, beta_data, gamma_data, x_repos, y_repos, h_repos, l1, l2, l3, landing, envy, need, speed_ratio, up_down_ratio, flight_angle, frm, color):
        self.alphamin = alpha_data[0]
        self.alphamax = alpha_data[1]

        self.betamin = beta_data[0]
        self.betamax = beta_data[1]

        self.gammamin = gamma_data[0]
        self.gammamax = gamma_data[1]

        self.side = side
        self.fix_position = fix_position
	self.fix_angle = fix_angle
        self.refchanging_leg_to_rob_matrix = np.array([[np.cos(fix_angle), -np.sin(fix_angle), 0], [np.sin(fix_angle), np.cos(fix_angle), 0], [0, 0, 1]])
        self.refchanging_rob_to_leg_matrix = np.linalg.inv(self.refchanging_leg_to_rob_matrix)
        self.leg_id = leg_id

        self.neighbours = neighbours

        self.l1 = l1
        self.l2 = l2
	self.l3 = l3

        self.color = color
        
        self.zone_colors = {"need": 'r',
                            "envy": 'b',
                            "landing" : 'g',
                            "center" : 'y',
                            "critical" : 'k'}
        self.ratios = {"need": need,
                       "envy": envy,
                       "landing":landing,
                       "center":0.5,
                       "critical": 0}

        self.speed_ratio_up_down = speed_ratio
        
        self.x_repos = x_repos
        self.y_repos = y_repos

        self.set_height(h_repos)
        self.update_angles_from_position()
        self.absolute_feet_position = None
        self.h_up = self.h*up_down_ratio
        self.flight_angle = flight_angle # Currently unused
        self.frm = frm

        self.R_repos = np.sqrt(x_repos**2+y_repos**2)
        sBetaRmin = (self.h - np.sin(self.gammamin)*self.l3)/self.l2
        self.Rmin = self.l1 + self.l2*np.sqrt(1-sBetaRmin**2) + self.l3*np.cos(self.gammamax)
        self.Rmax = self.l1 + np.sqrt((self.l2+self.l3)**2-self.h**2)

        self.flight = None
        self.cycle_takeoff = None
        self.status = 'down'
        self.demand = None
        self.contact = True

    def set_height(self, height):
        '''ONLY USE WHEN ALL LETS ARE BEING RESET AND DOWN '''
        self.h = height
        self.relative_feet_position = np.array([self.x_repos, self.y_repos, -self.h])

    def get_position_from_angles(self, angles):
        alpha = angles[0]
        beta  = angles[1]
        gamma = angles[2]

        x = self.l1*np.cos(alpha) + self.l2*np.cos(alpha)*np.cos(beta) + self.l3*np.cos(alpha)*np.cos(gamma)
        y = self.l1*np.sin(alpha) + self.l2*np.sin(alpha)*np.cos(beta) + self.l3*np.sin(alpha)*np.cos(gamma)
        z = 0                     - self.l2*np.sin(beta)               - self.l3*np.sin(gamma)

        return np.array([x,y,z])

    def get_extension(self, angles):
        position = self.get_position_from_angles(angles)
        return np.sqrt(position[0]**2 + position[1]**2)

    def get_angles_from(self, position):
        # Computes the 3 angles alpha, beta, gamma from a certain position of the leg in its own referential
        x = position[0]
        y = position[1]
        z = position[2]

        R = np.sqrt(x**2+y**2)
        alpha = np.arctan(y/x)
                
        theta = np.arccos(((R-self.l1)**2 + z**2 + self.l2**2 - self.l3**2)
                /(2 * self.l2 * np.sqrt((R-self.l1)**2 + z**2)))
        theta2 = -np.arctan((R-self.l1)/z) #possible "-"z missing
        
        beta = np.pi/2-(theta+theta2)
        if R > self.l1 + np.cos(beta)*self.l2:
            gamma = np.arcsin((np.abs(z)-self.l2*np.sin(np.pi/2-(theta+theta2)))/self.l3)
        else:
            gamma = np.pi - np.arcsin((np.abs(z)-self.l2*np.sin(np.pi/2-(theta+theta2)))/self.l3)
            
        return [alpha, beta, gamma]

    def update_angles_from_position(self):
        # Computes the 3 angles alpha, beta, gamma from the position of the leg in its own referential
        angles = self.get_angles_from(self.relative_feet_position)
        if angles != None:
            print "Changing angles for leg {4} : {0},{1} and {2} from position {3}".format(angles[0], angles[1], angles[2], self.relative_feet_position, self.leg_id)
            self.angles = angles
        else:
            print "Unable to find angles for leg {0} at position {1}".format(self.leg_id, self.relative_feet_position)

    def get_leg_absolute_position(self, robot):
        return robot.position + robot.rotate_vector_to_absolute_from_robot(self.fix_position + self.rotate_vector_to_robot_from_leg(self.relative_feet_position))

    def get_leg_relative_position(self, robot):
        return self.rotate_vector_to_leg_from_robot(robot.rotate_vector_to_robot_from_absolute(self.absolute_feet_position - robot.position) - self.fix_position)

    def rotate_vector_to_leg_from_robot(self, V):
        return np.dot(self.refchanging_rob_to_leg_matrix,V)

    def rotate_vector_to_robot_from_leg(self, V):
        return np.dot(self.refchanging_leg_to_rob_matrix,V)

    def zone_presence(self):
# Function to check in which zone the leg is or is not. 
# It returns a dictionnary with a boolean for each zone, being True if the leg is within its boundaries. 

        R = self.get_extension(self.angles)
        alpha = self.angles[0]

        zone_presence = {zone: True for zone in self.ratios.keys()}
        
        for zone in zone_presence.keys():
            limits = self.get_min_max_values(zone)
            if R < limits[0] or R > limits[1] or alpha < limits[2] or alpha > limits[3]:
                zone_presence[zone] = False
        zone_presence.pop('center')
        return zone_presence

    def get_min_max_values(self, zone_name):
        Rmin = self.Rmin + (self.Rmax - self.Rmin)*self.ratios[zone_name]
        Rmax = self.Rmax - (self.Rmax - self.Rmin)*self.ratios[zone_name]
        alphamin = self.alphamin + (self.alphamax - self.alphamin)*self.ratios[zone_name]
        alphamax = self.alphamax - (self.alphamax - self.alphamin)*self.ratios[zone_name]

        return Rmin, Rmax, alphamin, alphamax

    def get_corners(self, zone_name):
        Rmin, Rmax, alphamin, alphamax = self.get_min_max_values(zone_name)

        return [np.array([Rmin*np.cos(alphamin), -Rmin*np.sin(alphamin)]),
                np.array([Rmax*np.cos(alphamin), -Rmax*np.sin(alphamin)]),
                np.array([Rmax*np.cos(alphamax), -Rmax*np.sin(alphamax)]),
                np.array([Rmin*np.cos(alphamax), -Rmin*np.sin(alphamax)])]

    def plot_line(self, point, vector, length=10):
        '''Plots a line of total length 'length' centered in point, with a direction 'vector'
        
        Input :
            - point : 2D np.array
            - vector : 2D np.array
            - length : scalar
        '''
        x = [point[0] - length/2*vector[0], point[0] + length/2*vector[0]]
        y = [point[1] - length/2*vector[1], point[1] + length/2*vector[1]]
        pyl.plot(x, y, '-')

    def plot_zone(self, zone_name):
        N_points_arcs = 10
        Rmin, Rmax, alphamin, alphamax = self.get_min_max_values(zone_name)

        corners = self.get_corners(zone_name)
        pyl.plot([corners[0][0], corners[1][0]], [corners[0][1], corners[1][1]], self.zone_colors[zone_name]+'-')
        pyl.plot([corners[2][0], corners[3][0]], [corners[2][1], corners[3][1]], self.zone_colors[zone_name]+'-')
        
        arcs_points = [np.array([Rmin*np.cos(alpha), -Rmin*np.sin(alpha)]) for alpha in np.linspace(alphamin, alphamax, N_points_arcs)]
        for n_point in range(len(arcs_points)-1):
            pyl.plot([arcs_points[n_point][0], arcs_points[n_point+1][0]], [arcs_points[n_point][1], arcs_points[n_point+1][1]], self.zone_colors[zone_name]+'-')
        arcs_points = [np.array([Rmax*np.cos(alpha), -Rmax*np.sin(alpha)]) for alpha in np.linspace(alphamin, alphamax, N_points_arcs)]
        for n_point in range(len(arcs_points)-1):
            pyl.plot([arcs_points[n_point][0], arcs_points[n_point+1][0]], [arcs_points[n_point][1], arcs_points[n_point+1][1]], self.zone_colors[zone_name]+'-')

    def plot_feet(self, ref):
        if ref=='relative':
            pyl.plot(self.relative_feet_position[0], self.relative_feet_position[1], 'x'+self.color)
        elif ref == 'absolute':
            pyl.plot(self.absolute_feet_position[0], self.absolute_feet_position[1], 'x'+self.color)

    def has_neighbour_up(self, robot):
        '''Function to study which legs are up, and thus which legs are allowed to be raise:
        Input :
            - robot : Class robot describing the current robot master
        '''
        for neighbour_id in self.neighbours:
            if robot.Legs[neighbour_id].status == 'up':
                return True
        return False

    def get_ratio_distance_from_zone_to_next(self, zone_in, zone_out):
        '''Computes the ratio of distances between the two - inner and outer - zones the leg is around. 
        Needs the names of these two zones'''

        # Function FAILS maybe. Apparently not but weird
        R = self.get_extension(self.angles)
        r1, r2, a1, a2 = self.get_min_max_values(zone_out)
        dmin1 = min (np.abs(R-r1), np.abs(r2-R), np.abs(R*(self.angles[0]-a1)), np.abs(R*(self.angles[0]-a2)))
        r1, r2, a1, a2 = self.get_min_max_values(zone_in)
        micro_zone_value = 0
        if not r1<R<r2:
            micro_zone_value += 2
        if not a1<self.angles[0]<a2:
            micro_zone_value += 1
        if micro_zone_value == 3:
            corners = self.get_corners(zone_in)
            dmin2 = min([np.linalg.norm(self.relative_feet_position[:2]-corner) for corner in corners])
        elif micro_zone_value == 2:
            dmin2 = min(np.linalg.norm(r1-R), np.linalg.norm(r2-R))
        elif micro_zone_value == 1:
            dmin2 = R*min(np.linalg.norm(self.angles[0]-a1), np.linalg.norm(self.angles[0]-a2))
        else:
            print "Zone_in {0} is actually outer for leg {1} and relative_feet_position {2} ! Weird".format(zone_in, self.leg_id, self.relative_feet_position)
        return dmin2/(dmin1 + dmin2)

    def extend_flight(self):
        '''
        In case of an abnormaly long flight (i.e a hole where the leg should land), extends the flight by simply repeating the last vertical variation
        '''
        self.relative_feet_position += np.array([0., 0., (self.flight[-1][2]-self.flight[-2][2])])
        self.update_angles_from_position()
    
    def follow_flight(self, cycle):
        '''
        Function to follow the schedule flight as long as it is within its length
        Input :
            - cycle : int, cycle number by comparison to the initial cycle_takeoff
        '''
        self.relative_feet_position = self.flight[cycle - self.cycle_takeoff] # We update the new position from the predifined flight
        self.update_angles_from_position() # Update the leg angles from this position

    def check_landing(self, cycle=None):
        '''
        Function to check if the leg has landed or not, and updates the different statuses
        Input :
            cycle : int, cycle number. Allows to cheat for tests and developpement stage
        '''
        if cycle != None:
            if cycle-self.cycle_takeoff == len(self.flight):
                self.status='down'
                print "Leg {0} landed".format(self.leg_id)
                
        else:
            if self.contact == True:
                self.status='down'
                print "Leg {0} landed".format(self.leg_id)

    def initiate_flight(self, cycle, final_feet_point, final_orientation, N_points, N_points_aimed):
        '''Function to initiate a flight of the considered leg.
        Should contain all the variables changes, the call for this function should be enough for the leg to move on later cycles.
        Input :
            cycle : cycle number at the start of the flight
            final_feet_point : 3-dimensional np.array vector containing the relative position of the feet when the move is over AT THE END OF THE FLIGHT.
            final_orientation : 3-D vector containing the final (relative) orientation of the robot AT THE END OF THE FLIGHT.
            N_points : number of points this flight should contain
            N_points_aimed : theoretical number of points the flight should have, if for example the flight was programmed at the very end of the move'''

        print "Initiating flight with parameters {0}, {1}, {2}, {3}".format(cycle, final_feet_point, final_orientation, N_points_aimed)
        self.create_flight(final_feet_point[:2], final_orientation[:2], N_points_aimed)
        self.status = 'up'
        self.cycle_takeoff = cycle

    def create_flight(self, final_feet_point, final_orientation, N_points):
        '''Creates the array of (relative) positions the leg should be at while in the air. 
        
        Needs the estimated relative arrival point and the number of points for the flight duration.
        Also, we assume that the leg will land in z = 0. Possible necessary modifications about plannification
        Input :
            final_feet_point : 3-dimensional np.array vector containing the relative position of the feet when the move is over AT THE END OF THE FLIGHT. 
            final_orientation : 2-D vector containing the final (relative) orientation of the robot AT THE END OF THE FLIGHT.
            N_points : number of points this flight should contain'''
        
        # First we look for the point to be aimed.
        arrival = self.get_arrival_point(final_feet_point, final_orientation)
        start = self.relative_feet_position

        self.flight = tools.flight(start, arrival, self.h_up, N_points)
        print "Final flight : {0} points for a distance of {1}, from {2} to {3}".format(len(self.flight), np.linalg.norm(arrival-start), start, arrival)

    def is_point_in_zone(self, point, zone_aimed):
        '''Checks if a point is inside the zone_aimed.

        Needs the position of the point
        Input :
            point : 3-dimensional np.array vector containing the (relative) position of this point. The third value is irrelevant.
            zone_aimed : name of the contour the function is checking'''

        point_2D = np.array(point[0:2])
        R = np.linalg.norm(point_2D)
        zone_limits = self.get_min_max_values(zone_aimed)
        if zone_limits[0] <= R <= zone_limits[1] and zone_limits[2] <= np.arctan(point_2D[1]/-point_2D[0]) <= zone_limits[3]:
            return True
        else:
            return False

    def get_arrival_point(self, final_feet_point, final_orientation, zone_aimed = 'envy'):
        '''Computes the landing point of a leg depending of the final point this leg should be on at the very end.
        Needs the final position of this leg and the final orientation of the robot
        Input :
            final_feet_point : 3-dimensional np.array vector containing the relative position of the feet when the move is over AT THE END OF THE FLIGHT. Most likely the third value is 0 as we can assume the ground is flat
            final_orientation : 2-D vector containing the final (relative) orientation of the robot AT THE END OF THE FLIGHT.
            zone_aimed : name of the contour the feet is aiming. Landing on 'critical' is *VERY* dangerous, while landing on 'envy' might be very power consuming'''

        # TODO : issue of definitions here : We must take into account the movement of the robot during the flight. While we are moving towards the final line, and while it doesn't come in the zone during it, no pb.
        # But once it gets in during the flight, we can't only use the relative coordinates. Thus we have to pass into arguments the last coorinates and orientation of the robot at the end of the N_points of "create_flight".

        final_feet_point = final_feet_point[0:2]
        zone_corners = self.get_corners(zone_aimed)
        zone_limits = self.get_min_max_values(zone_aimed)
        final_line = ['L', final_feet_point, final_orientation] #TODO : final_orientation must be relative ! Create new function to turn it, depending on the leg.side, Robot.orientation and final_orientation

        # entities = [['C', [0., 0.], self.get_min_max_values(zone_aimed)[0]], OLD, segment in interior makes zone convex
        entities = [['S', zone_corners[0], zone_corners[3]],
                    ['C', [0., 0.], self.get_min_max_values(zone_aimed)[1]],
                    ['S', zone_corners[0], zone_corners[1]],
                    ['S', zone_corners[2], zone_corners[3]]]

        # First we try to intersect the final line with the possible landing zone
        intersections = []
        for entity in entities:
            print "Intersecting {0} and {1} for leg {2}".format(final_line, entity, self.leg_id)
            intersections_tmp = [[tools.intersect(final_line, entity), entity[0]]]
            for intersection_tmp in intersections_tmp:
                if len(intersection_tmp[0])==0:
                    print "No intersection here"
                else:
                    print "Tmp intersection found : {0}".format(intersection_tmp)
                    for intersection in intersection_tmp[0]:
                        print "Considering {0} with constrains {1}".format(intersection, zone_limits)
                        if intersection_tmp[1] == 'S' or (zone_limits[2] <= np.arctan(intersection[1]/-intersection[0]) <= zone_limits[3] and intersection[0] > 0):
                            print "Accepted"
                            intersections += [intersection]
                        else:
                            print "Rejected"

        if intersections != []:
            # If they were intersections, it is possible that the final point is inside this zone.
            print "Final line found intersecting"
            if self.is_point_in_zone(final_feet_point, zone_aimed):
                # If this point iss indeed within the boundaries, we do aim for it
                # TODO : In this case, we should reduce the number of points necessary !
                print "Found final point in zone. Aiming {0}".format(final_feet_point)
                return np.array(final_feet_point.tolist() + [-self.h])
            else:
                # If it is not, we aim for the closest point to this final point that intersect zone_aimed
                print "Final point not in zone. Aiming closest to {0} between intersections_found : {1}".format(final_feet_point, intersections)
                norms=[np.linalg.norm(intersection - final_feet_point) for intersection in intersections]
                print "Computed norms : {0}".format(norms)
                elected_point = np.array(intersections[norms.index(min(norms))].tolist() + [-self.h])
                print "Elected point {0} for landing".format(elected_point)
                return elected_point
        else:
            print "No final line intersection found"
            intersections = []
            # If they were no intersection, we use the frm variable to compute the movement.
            I_point = tools.intersect(final_line, ['L', np.array([0., 0.]), np.array([final_orientation[1], -final_orientation[0]])])[0]
            aimed_point = (1-self.frm) * I_point + self.frm * final_feet_point
            for entity in entities:
                print "Intersecting {0} and {1} for leg {2}".format(final_line, entity, self.leg_id)
                intersections_tmp = [[tools.intersect(['S', self.relative_feet_position[0:2], aimed_point], entity), entity[0]]]
                for intersection_tmp in intersections_tmp:
                    if len(intersection_tmp[0])==0:
                        print "No intersection here"
                    else:
                        print "Tmp intersection found : {0}".format(intersection_tmp)
                        for intersection in intersection_tmp[0]:
                            if intersection_tmp[1] == 'S' or (zone_limits[2] <= np.arctan(intersection[0]/-intersection[1]) <= zone_limits[3] and intersection[1] < 0):
                                intersections += [intersection]
            if len(intersections) == 0:
                print "Failed to find an intersection point with frm value"
                return None
            elif len(intersections) > 1:
                print "Found more than one intersection with frm value, abnormal : {0}".format(intersections)
                return None
            else:
                print "Intends to land in {0}".format(intersections[0])
                return intersections[0]

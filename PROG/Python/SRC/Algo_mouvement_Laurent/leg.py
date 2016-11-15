import numpy as np
import math
import tools
import matplotlib.pylab as pyl

# The leg class is defined as something static. We don't consider any movement of the robot in here, thus all input and output will be in its self referential :
#
#           [0., 0.]    y
#               o       ^
#                       |
#                       |
#                       ----> x
#
#          __________
#         /          \
#        /            \        
#       /              \        
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
    def __init__(self, leg_id, fix_position, side, alpha_data, beta_data, gamma_data, l1, l2, landing, envy, need, speed_ratio, up_down_ratio, color):
        self.alphamin = alpha_data[0]
        self.alphamax = alpha_data[1]
        self.alpharepos = alpha_data[2]

        self.betamin = beta_data[0]
        self.betamax = beta_data[1]
        self.betarepos = beta_data[2]

        self.gammamin = gamma_data[0]
        self.gammamax = gamma_data[1]
        self.gammarepos = gamma_data[2]

        self.angles = [self.alpharepos, self.betarepos, self.gammarepos]

        self.side = side
        self.fix_position = fix_position
        self.leg_id = leg_id

        self.l1 = l1
        self.l2 = l2

        self.color = color
        
        self.zone_colors = {"need": 'r',
                            "envy": 'b',
                            "landing" : 'g',
                            "critical" : 'k'}
        self.ratios = {"need": need,
                       "envy": envy,
                       "landing":landing,
                       "critical": 0}

        self.speed_ratio_up_down = speed_ratio
        
        self.h = 0
        self.relative_feet_position = self.get_position_from_angles(self.angles, init=True)
        self.absolute_feet_position = []
        self.h_up = self.h*up_down_ratio
        
        self.R_repos = self.get_extension(self.angles)
        self.Rmin = np.sqrt(self.l1**2 - (self.h - self.l2*np.cos(self.gammamin))**2) + self.l2*np.sin(self.gammamin)
        self.Rmax = np.sqrt((self.l1+self.l2)**2-self.h**2)

        self.t_origin = None
        self.flight = None
        self.status = 'down'
        self.demand = None

    def get_position_from_angles(self, angles, init=False):
        R = self.l1*np.cos(angles[1]) + self.l2*np.sin(angles[2])
        h = self.l1*np.sin(angles[1]) + self.l2*np.cos(angles[2])
        x = R*np.sin(angles[0])
        y = R*np.cos(angles[0])

        if init:
            self.h = h

        return np.array([x, -y, self.h-h])

    def get_extension(self, angles):
        return self.l1*np.cos(angles[1]) + self.l2*np.sin(angles[2])

    def update_angles_from_position(self):
# Computes the 3 angles alpha, beta, gamma from the position of the leg in its own referential
        R = np.linalg.norm(self.relative_feet_position[:2])
        z = self.h - self.relative_feet_position[2]
        try:
            alpha = math.asin(self.relative_feet_position[0]/R)
            theta = math.acos((R**2 + z**2 + self.l1**2 - self.l2**2)
                             /(2 * self.l1 * np.sqrt(R**2 + z**2)))
            theta2 = math.atan(R/z)
            beta = -np.pi/2 + (theta + theta2)
            phi = math.acos((R**2 + z**2 + self.l2**2 - self.l1**2)
                           /(2 * self.l2 * np.sqrt(R**2 + z**2)))
            phi2 = math.atan(z/R)
            gamma = np.pi/2 - (phi + phi2)
            print "Saving angles : {0},{1} et {2} from position {3}".format(alpha, beta, gamma, self.relative_feet_position)
            self.angles = [alpha, beta, gamma]
            return None
        except:
            print "Unable to find angles for leg {0} at position {1}".format(self.leg_id, self.relative_feet_position)
            return None
    
    def get_leg_absolute_position(self, position, orientation):
# Function that computes the leg absolute position from the current position of the robot and the relative position of the leg in its own landmark
# Input:
#   Robot_object : Structure robot, that gives opsition and orientation in absolute landmark
        if self.side == 'right':
            return np.array(position)+tools.rotate(self.fix_position + self.relative_feet_position[0:2], orientation)
        else:
            return np.array(position)+tools.rotate(self.fix_position - self.relative_feet_position[0:2], orientation)

    def get_leg_relative_position(self, position, orientation):
# Opposite function to get_leg_absolute_position. Gives relative position of the leg considering its absolute position and the absolute position of the robot
# Input:
#   Robot_object : Structure robot, that gives opsition and orientation in absolute landmark
        if self.side == 'right':
            return np.array((tools.rotate(self.absolute_feet_position - np.array(position), [orientation[0], -orientation[1]]) - self.fix_position).tolist() + [0.])
        else:
            return -np.array((tools.rotate(self.absolute_feet_position - np.array(position), [orientation[0], -orientation[1]]) - self.fix_position).tolist() + [0.])

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
        return zone_presence

    def get_min_max_values(self, zone_name):
        Rmin = self.Rmin + (self.Rmax - self.Rmin)*self.ratios[zone_name]
        Rmax = self.Rmax - (self.Rmax - self.Rmin)*self.ratios[zone_name]
        alphamin = self.alphamin + (self.alphamax - self.alphamin)*self.ratios[zone_name]
        alphamax = self.alphamax - (self.alphamax - self.alphamin)*self.ratios[zone_name]

        return Rmin, Rmax, alphamin, alphamax

    def get_corners(self, zone_name):
        Rmin, Rmax, alphamin, alphamax = self.get_min_max_values(zone_name)

        return [np.array([Rmin*np.sin(alphamin), -Rmin*np.cos(alphamin)]),
                np.array([Rmax*np.sin(alphamin), -Rmax*np.cos(alphamin)]),
                np.array([Rmax*np.sin(alphamax), -Rmax*np.cos(alphamax)]),
                np.array([Rmin*np.sin(alphamax), -Rmin*np.cos(alphamax)])]

    def plot_zone(self, zone_name):
        N_points_arcs = 10
        Rmin, Rmax, alphamin, alphamax = self.get_min_max_values(zone_name)

        corners = self.get_corners(zone_name)
        pyl.plot([corners[0][0], corners[1][0]], [corners[0][1], corners[1][1]], self.zone_colors[zone_name]+'-')
        pyl.plot([corners[2][0], corners[3][0]], [corners[2][1], corners[3][1]], self.zone_colors[zone_name]+'-')
        
        arcs_points = [np.array([Rmin*np.sin(alpha), -Rmin*np.cos(alpha)]) for alpha in np.linspace(alphamin, alphamax, N_points_arcs)]
        for n_point in range(len(arcs_points)-1):
            pyl.plot([arcs_points[n_point][0], arcs_points[n_point+1][0]], [arcs_points[n_point][1], arcs_points[n_point+1][1]], self.zone_colors[zone_name]+'-')
        arcs_points = [np.array([Rmax*np.sin(alpha), -Rmax*np.cos(alpha)]) for alpha in np.linspace(alphamin, alphamax, N_points_arcs)]
        for n_point in range(len(arcs_points)-1):
            pyl.plot([arcs_points[n_point][0], arcs_points[n_point+1][0]], [arcs_points[n_point][1], arcs_points[n_point+1][1]], self.zone_colors[zone_name]+'-')

    def get_ratio_distance_from_zone_to_next(self, zone_in, zone_out):
        R = self.get_extension(self.angles)
        r1, r2, a1, a2 = self.get_min_max_values(zone_out)
        dmin1 = min (abs(R-r1), abs(r2-R), abs(R*(self.angles[0]-a1)), abs(R*(self.angles[0]-a2)))
        print "Distance to {0} : {1}".format(zone_out, dmin1)
        r1, r2, a1, a2 = self.get_min_max_values(zone_in)
        dmin2 = min (abs(r1-R), abs(R-r2), abs(R*(self.angles[0]-a1)), abs(R*(self.angles[0]-a2)))
        print "Distance to {0} : {1}".format(zone_in, dmin2)
        return dmin2/(dmin1 + dmin2)

    def create_flight(self, start, arrival, mean_speed):
        D = np.linalg.norm(np.array(start)-np.array(arrival))
        O = [(start[0]+arrival[0])/2, (start[1]+arrival[1])/2, ((D**2)/4-self.h_up**2)/(2*self.h_up)]
        delta = 2*math.atan(D/(2*O[2]))
        L = delta*(self.h_up + O[2])
        N = max(3, int(L/(self.speed_ratio_up_down*mean_speed*self.R_repos)))
        deltas = np.linspace(-delta/2, delta/2, N)
        self.flight = [start]
        for delta in deltas:
            self.flight += [[O[0]+np.sin(delta)*(O[2]+h_up)*(arrival[0]-start[0])/D, O[1]+np.sin(delta)*(O[2]+h_up)*(arrival[1]-start[1])/D, -O[2]+np.cos(delta)*(O[2]+self.h_up)]]
        self.flight += [arrival]
        print "Final flight : {0} points for a distance of {1}, from {2} to {3}".format(len(self.flight), D, start, arrival)

    def get_arrival_point(self, start, robot_current, robot_final, final_orientation):
        zone_corners_landing = self.get_corners("landing")
        leg_rest_vector = np.array([0., -1])
        F = - self.fix_position + robot_final - robot_current + tools.rotate((self.fix_position + leg_rest_vector*R_repos),
                                                                             final_orientation)
        if np.dot(leg_rest_vector, (F/np.linalg.norm(F))) >= np.dot(leg_rest_vector, (zone_corners_landing[3]/np.linalg.norm(zone_corners[3]))):
            print "Found direct flight for final position"
            return np.array(F)
        side = None
        for corner in zone_corners:
            if np.dot(corner-np.array(F), rotate(u, [0, 1])) > 0:
                if side == None:
                    side = 1
                elif side == -1:
                    side = 0
                else:
                    if side == None:
                        side = -1
                    elif side == 1:
                        side = 0
        entities = [['C', [0., 0.], Rmax-Rdiff*self.ratios["landing"]],
                    ['C', [0., 0.], Rmin+Rdiff*self.ratios["landing"]],
                    ['S', zone_corners[0], zone_corners[1]],
                    ['S', zone_corners[2], zone_corners[3]]]
        if side == 0:
            print "Found final line in zone"

            final_line = ['L', F, u]
            inter_kept = []
            for entity in entities:
                inters = tools.intersect(final_line, entity)
                print "{0} intersection(s) between {1} and {2}".format(len(inters), final_line, entity)
                for tmp in inters:
                    print "Possible landing point from {1} : {0}".format(tmp, start)
                    if entity[0] == 'C':
                        print "Cosinus is {2} compared to {3}".format(np.dot(leg_rest_vector, (tmp/np.linalg.norm(tmp))), np.dot(leg_rest_vector, (zone_corners[3]/np.linalg.norm(zone_corners[3]))))
                    if entity[0] == 'S' or np.dot(leg_rest_vector, (tmp-1*np.array(zone_center))/np.linalg.norm(np.array(zone_center)-tmp)) >= np.dot(leg_rest_vector, (zone_corners[3]-np.array(zone_center))/np.linalg.norm(np.array(zone_center)-zone_corners[3])):
                        inter_kept += [tmp]
                        print "Kept :"
                        print zone_center, tmp, -1*np.array(zone_center)+tmp
            dists = []
            for inter in inter_kept:
                dists += [np.linalg.norm(inter-np.array(F))]
            final_point = inter_kept[dists.index(min(dists))]
            print "Found direct fight for final parallel line"
            print final_point
            return final_point
        else:
            print "Computing tmp position before final line"
            print "Computing pure rotating movement"
            C_rot = ['C', np.array(O), np.linalg.norm(np.array(O)-np.array(I))]
            intersections = []
            inter_kept = []
            for entity in entities:
                inters = intersect(C_rot, entity)
                print "{0} intersection(s) between {1} and {2}".format(len(inters), C_rot, entity)
                for tmp in inters:
                    print "Possible landing point : {0} from {1}, cosinus is {2} compared to {3}".format(tmp, I, np.dot(leg_rest_vector, (tmp-np.array(zone_center))/np.linalg.norm(np.array(zone_center)-tmp)), np.dot(leg_rest_vector, (zone_corners[3]-np.array(zone_center))/np.linalg.norm(np.array(zone_center)-zone_corners[3])))
                    if entity[0] == 'S' or np.dot(leg_rest_vector, tmp/np.linalg.norm(tmp)) >= np.dot(leg_rest_vector, zone_corners[3]/np.linalg.norm(zone_corners[3])):
                        inter_kept += [tmp]
                        print "Kept :"
            dists = []
            for inter in inter_kept:
                dists += [np.linalg.norm(inter-np.array(I))]
            v1 = inter_kept[dists.index(max(dists))] - np.array(I)
            print "v1 : {0}".format(v1)
            
            print "Computing permanent rotating movement"
            S_direct = ['S', np.array(I), np.array(F)]
            intersections = []
            inter_kept=[]
            for entity in entities:
                inters = intersect(S_direct, entity)
                print "{0} intersection(s) between {1} and {2}".format(len(inters), S_direct, entity)
                for tmp in inters:
                    print "Possible landing point : {0} from {1}, cosinus is {2} compared to {3}".format(tmp, I, np.dot(leg_rest_vector, (tmp-np.array(zone_center))/np.lingalg.norm(np.array(zone_center)-tmp)), np.dot(leg_rest_vector, (zone_corners[3]-np.array(zone_center))/np.linalg.norm(np.array(zone_center)-zone_corners[3])))
                    if entity[0] == 'S' or np.dot(leg_rest_vector, tmp-1/np.linalg.norm(tmp)) >= np.dot(leg_rest_vector, zone_corners[3]/np.linalg.norm(zone_corners[3])):
                        inter_kept += [tmp]
                        print "Kept :"
            dists = []
            for inter in inter_kept:
                dists += [np.linalg.norm(inter-np.array(F))]
            v2 = inter_kept[dists.index(min(dists))] - np.array(I)
            print "v2 : {0}".format(v2)
            
            print "Computing final point considering v_mean"
            v_mean = v2
            #v_mean = facteur_rotation_mouvement * v2 + (1 - facteur_rotation_mouvement)*v1
            tmp_line = ['L', I, v_mean]
            inter_kept = []
            for entity in entities:
                inters = intersect(tmp_line, entity)
                print "{0} intersection(s) between {1} and {2}".format(len(inters), tmp_line, entity)
                for tmp in inters:
                    print "Possible landing point : {0} from {1}, cosinus is {2} compared to {3}".format(tmp, I, np.dot(leg_rest_vector, (tmp-np.array(zone_center))/np.linalg.norm(np.array(zone_center)-tmp)), np.dot(leg_rest_vector, (zone_corners[3]-np.array(zone_center))/np.linalg.norm(np.array(zone_center)-zone_corners[3])))
                    if entity[0] == 'S' or np.dot(leg_rest_vector, (tmp-1*np.array(zone_center))/np.linalg.norm(np.array(zone_center)-tmp)) >= np.dot(leg_rest_vector, (zone_corners[3]-np.array(zone_center))/np.linalg.norm(np.array(zone_center)-zone_corners[3])):
                        inter_kept += [tmp]
                        print "Kept :"
                        print zone_center, tmp, -1*np.array(zone_center)+tmp
            dists = []
            for inter in inter_kept:
                dists += [np.linalg.norm(inter-np.array(F))]
            print "Arrival point from {0} : {1}".format(I, inter_kept[dists.index(min(dists))])
            ideal_point = inter_kept[dists.index(min(dists))]
                                                            
            return final_point
                                                        

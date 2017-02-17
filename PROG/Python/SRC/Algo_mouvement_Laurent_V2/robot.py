from leg import *
from rospy_tutorials.msg import Floats
import numpy as np
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Float64
from std_msgs.msg import Float32
from std_msgs.msg import String
import rospy
import tools
import time

class Robot:
    def __init__(self, debug = False, minimum_legs_down = 4, taxiway_delay_cycles = 8, max_sameside_leg_number = 4, alpha_margin_sameside = np.pi/40, frm = 0.1, srud =30., up_initial_value = 2., udhr = 0.1, leg_contact_wait_cycles = 1, CAPDelay = 0.002, artefact=False, artefact_position = None, artefact_orientation = None):
        if not artefact:
            geometry_data = tools.file_loader('geometry.txt')

            # All length units here are in arbitrary units. 
            self.N_legs = int(geometry_data['N_LEGS']) # Number of leg in this model
            self.artefact = False
            self.debug = debug

#Initialization of ROSPY
            self.command = 'STOP' # Possibilities : 'STOP', 'ERROR', 'MOVE', 'SETH', 'RESET'
            self.status = 'STOPPED' # Possibilities : 'STOPPED', 'ERROR', 'MOVING', 'RESETING', 'SETHING', 'CLOSING'
            self.NextAction = None # Possibilities : 'MOVE', 'SETH' , 'RESET'
            self.speedRatio = 0.
            self.h = float(geometry_data['H'])
            self.NewHeight = self.h

            self.motor_publishers = [rospy.Publisher("angles_raw_leg_{0}".format(i), numpy_msg(Floats),queue_size=1) for i in range(self.N_legs)]
            self.delta_publisher = rospy.Publisher("delta", Float32 ,queue_size=1)
            self.position_publisher = rospy.Publisher("position", numpy_msg(Floats),queue_size=1)
            self.orientation_publisher = rospy.Publisher("orientation", numpy_msg(Floats),queue_size=1)
            self.command_publisher = rospy.Publisher("command", String, queue_size=1)
            self.status_publisher = rospy.Publisher("status", String, queue_size=1)
            rospy.Subscriber("command", String, self.UpdateCommand)
            rospy.Subscriber("speed", Float32, self.UpdateSpeed)
            rospy.Subscriber("height", Float32, self.UpdateHeight)
            rospy.Subscriber("direction", numpy_msg(Floats), self.UpdateDirection)
            rospy.Subscriber("legs_contacts", String, self.UpdateContact)
            rospy.init_node('moving_algo', anonymous=True)
            self.r = rospy.Rate(1)

            self.speed = float(geometry_data['SPEED']) # In cm/s
            self.N_points_by_cm = 20
            self.ResetTimeOneLeg = 1. #In seconds
            self.NPointsResetFlight = 30
            self.SetHeightSpeed = 1 # In cm/s
            self.NPointsSetHeight = 2
            #self.MeanAlphaMove = 0.015 # Gives an estimate of the speed of the robot at this timestep. Allows to create previsions for a flight.
            self.MeanAlphaMove = 0.007 # Gives an estimate of the speed of the robot at this timestep. Allows to create previsions for a flight.
            self.computeDeltasT()
        
            # RULES DEFINITIONS
            self.minimum_legs_down = minimum_legs_down # Minimum number of leg on the ground at all cycles to ensure bearing
            self.max_sameside_leg_number = max_sameside_leg_number # Max number of leg that can be on the same side
            self.alpha_margin_sameside =  alpha_margin_sameside # Margin allowed to avoid any issue around all alphas = 0
            self.taxiway_delay_cycles = taxiway_delay_cycles
            self.last_takeoff = -taxiway_delay_cycles
            self.CAPDelay = CAPDelay
            
            # Variables defining the history of the robot.
            self.history = []
            self.angles_history = []
            self.absolute_feet_positions_history = []
            self.relative_feet_positions_history = []
            self.landmark_history = [] # List of landmark changes
            
            # Values to define the leg demands zones (see leg.py). Keep the order need < envy (< landing, to be removed).
            # The larger these values, the more safe you are about geetting close to mechanical stops, but the more power consuming this model gets. 
            need = float(geometry_data['NEED'])
            envy = float(geometry_data['ENVY'])
            landing = float(geometry_data['LANDING']) #Useless for now
            
            # Definition of allowed and default leg angles. Alpha is the horizontal rotation angle, Beta is the first vertical rotation  angle and gamma is the second one.
            # Xrepos is the default value chosen.
            alphamin = float(geometry_data['ALPHAMIN'])
            alphamax = float(geometry_data['ALPHAMAX'])
            alphadiff = alphamax - alphamin
            
            betamin = float(geometry_data['BETAMIN'])
            betamax = float(geometry_data['BETAMAX'])
            
            gammamin = float(geometry_data['GAMMAMIN'])
            gammamax = float(geometry_data['GAMMAMAX'])

            x_repos = float(geometry_data['X_REPOS'])
            y_repos = float(geometry_data['Y_REPOS'])
            
            self.frm = frm # Rotation/Movement factor. At 0, the  robot rotates on himself then goes forward. At 1, the robot permenently rotates as it translates to the final point
            self.srud = srud # Speed ratio up/down (SRUD) defines how fast a leg travels forward when lifted compared to grounded legs. For this model to work, we need rud > N_legs/(N_legs - minimum_legs_down). False. TODO
            # It will be mostly constrained by physical capacities of actuators
            self.udhr = udhr # Up/Down Height Ratio. Height to which each leg has to be lifted when not grounded compared to the height of the robot. 
            # Should be interfaced with trajectories generation.
            self.leg_contact_wait_cycles = leg_contact_wait_cycles
            self.up_initial_value = up_initial_value # Initial value to raise the leg up (cm)
            
            colors = ['r', 'b', 'y', 'm', 'g', 'c'] # Set of color for display
            
            #legs_fix = [[L/2, -l/2, 0], [0, -l/2, 0], [-L/2, -l/2, 0], [L/2, l/2, 0], [0, l/2, 0], [-L/2, l/2, 0]] # Position of each leg on the body or the robot. Also used to define the 'side' of the leg
            legs_fix = [np.array([float(subvalue) for subvalue in value.split('=')]) for value in geometry_data['LEGS_FIX_POSITIONS']]
            #legs_fix_angles = [-np.pi/4, -np.pi/2, -3*np.pi/4, np.pi/4, np.pi/2, 3*np.pi/4]
            legs_fix_angles = [float(value) for value in geometry_data['LEGS_FIX_ANGLES']]
            #legs_neighbours = [[1,3], [0,2,4], [1,5], [0,4], [1,3,5],[2,4]]
            legs_neighbours = [[int(subvalue) for subvalue in value.split('=')] for value in geometry_data['LEGS_NEIGHBOURS']]
                
            self.Legs= []
            # Now we define all the legs with the set of parameters defined here
            for n_leg in range(self.N_legs):
                if legs_fix[n_leg][1] < 0:
                    side = 'right'
                else:
                    side= 'left'
                self.Legs += [Leg(n_leg, 
                    legs_neighbours[n_leg],
                    legs_fix[n_leg], 
                    legs_fix_angles[n_leg],
                    side, 
                    [alphamin, alphamax], 
                    [betamin, betamax],
                    [gammamin, gammamax],
                    x_repos,
                    y_repos,
                    self.h,
                    float(geometry_data['L1']),
                    float(geometry_data['L2']),
                    float(geometry_data['L3']),
                    landing,
                    envy,
                    need,
                    srud,
                    udhr,
                    up_initial_value, 
                    frm,
                    colors[n_leg])]
                
            self.position = np.array([0., 0., self.h]) # Initial position. At the initialisation, the robot is at the vertical of its landmark
            self.orientation = np.array([1., 0., 0.]) # Initial orientation. At the initialisation, the robot is along the x axis
            self.Direction = np.array([0., 0., 1.])
            self.PublishRobotData()

            self.compute_refchanging_rob_to_abs_matrix()
            
            self.history += [self.position]
            self.current_landmark = [self.position, self.orientation]
            self.landmark_history += [self.current_landmark]
            
            self.angles_history += [[]]
            for leg in self.Legs:
                self.angles_history[-1] += [leg.angles]
                
            self.absolute_feet_positions_history += [[]]
            self.relative_feet_positions_history += [[]]
            for leg in self.Legs:
                leg.absolute_feet_position = leg.get_leg_absolute_position(self)
                self.absolute_feet_positions_history[-1] += [leg.absolute_feet_position]
                self.relative_feet_positions_history[-1] += [leg.relative_feet_position]

            #self.SaveAndPublishCommand('RESET')
            #self.SaveAndPublishStatus('SETHING')
            self.Initialize()

            self.SaveAndPublishStatus('STOPPED')

        else:
            # The artefact option allows to create a fake robot to use in the different routines, for example in leg.get_leg_absolute_position
            self.artefact = True
            self.position = artefact_position
            self.orientation = artefact_orientation
            self.compute_refchanging_rob_to_abs_matrix()

    def Initialize(self):
            for n_leg in range(self.N_legs):
                self.motor_publishers[n_leg].publish(np.array(self.Legs[n_leg].angles, dtype = np.float32))
                time.sleep(0.07)

    def ConcatenateAnglesAndPublish(self):
        for leg in self.Legs:
            AnglesMessage = np.array(leg.angles, dtype = np.float32)
            time.sleep(self.CAPDelay)
            self.motor_publishers[leg.leg_id].publish(AnglesMessage)

    def PublishRobotData(self):
        self.position_publisher.publish(np.array(self.position, dtype= np.float32))
        self.orientation_publisher.publish(np.array(self.orientation, dtype= np.float32))

    def computeDeltasT(self):
        if self.speedRatio != 0:
            self.DeltaTGoto = 1./(self.N_points_by_cm * self.speedRatio * self.speed)
            self.DeltaTReset = self.ResetTimeOneLeg/(self.speedRatio*self.NPointsResetFlight)
            self.DeltaTSetHeight = 1./(self.speedRatio * self.SetHeightSpeed * self.NPointsSetHeight)
        else:
            print "Unable to compute DeltaTs, since self.speedRatio is 0. Waiting for speed ratio update."

    def UpdateContact(self, contactsMessage):
        for n_leg in range(self.N_legs):
            self.Legs[n_leg].contact = bool(int(contactsMessage.data.split('&')[n_leg]))

    def UpdateCommand(self, commandMessage):
        self.command = commandMessage.data
        print "Set command to {0}".format(self.command)

        current_status = self.status
        if self.command == 'ERROR':
            print "'ERROR' command received, changed status"
            self.SaveAndPublishStatus('ERROR')

        elif self.command == 'STOP':
            if current_status == 'MOVING':
                self.SaveAndPublishStatus('CLOSING')
                self.NextAction = None

            elif current_status == 'SETHING':
                print "Unable to stop SETH move for now. TODO"
                self.NextAction = None

            elif current_status == 'RESET':
                print "Unable to stop RESET move for now. TODO"
                self.NextAction = None

            elif current_status == 'ERROR':
                print "Current status is 'ERROR'. Try reset first. Ignoring order"
                self.NextAction = None
                #self.SaveAndPublishCommand('ERROR')

            elif current_status == 'CLOSING':
                print "Can't stop a CLOSING status midterm. Ignoring order"
                self.NextAction = None

            elif current_status == 'STOPPED':
                print "Already stopped. Ignoring order"

            else:
                print "Unknown combination of command/status : {0}/{1}".format(self.command, current_status)

        elif self.command == 'RESET':
            if current_status == 'MOVING':
                print "Closing current action and resetting"
                self.SaveAndPublishStatus('CLOSING')
                self.NextAction = 'RESET'

            elif current_status == 'ERROR':
                print "Current status is 'ERROR'. Trying the reset"
                self.SaveAndPublishStatus('RESETING')
                self.NextAction = None
                self.ResetLegsPositions()

            elif current_status == 'CLOSING':
                print "Added Reset at the end of current 'CLOSING' action"
                self.NextAction = 'RESET'

            elif current_status == 'STOPPED':
                print "Starting Reset"
                self.SaveAndPublishStatus('RESETING')
                self.NextAction = None
                self.ResetLegsPositions()

            elif current_status == 'RESETING':
                print "Currently reseting while sent reset order. Ignoring order."
                self.NextAction = None

            elif current_status == 'SETHING':
                print "Currently setting height, should already have reset legs. Ignoring order."
                self.NextAction = None
                #self.SaveAndPublishCommand('SETH')

            else:
                print "Unknown combination of command/status : {0}/{1}".format(self.command, current_status)

        elif self.command == 'SETH':
            if current_status == 'MOVING':
                print "Closing current action and setting height"
                self.SaveAndPublishStatus('CLOSING')
                self.NextAction = 'SETH'

            elif current_status == 'ERROR':
                print "Current status is 'ERROR'. Try reset first. Ignoring order"
                self.NextAction = None
                #self.SaveAndPublishCommand('ERROR')

            elif current_status == 'CLOSING':
                print "Added Set Height at the end of current 'CLOSING' action"
                self.NextAction = 'SETH'

            elif current_status == 'STOPPED':
                print "Starting Height Set global execution"
                self.NextAction = None
                self.SaveAndPublishStatus('SETHING')
                self.ResetLegsPositions(forSetHeight = True)

            elif current_status == 'RESETING':
                print "Added SetHeight at the end of current reset"
                self.NextAction = 'SETH'

            elif current_status == 'SETHING':
                self.NextAction = None
                print "Currently setting height. Ignoring order."

            else:
                print "Unknown combination of command/status : {0}/{1}".format(self.command, current_status)

        elif self.command == 'MOVE':
            if current_status == 'MOVING':
                print "Error, sent moving while already moving. Should not happend, ignoring order"

            elif current_status == 'ERROR':
                print "Dangerous to restart while currently in ERROR. Try reset first. Ignoring order."
                self.NextAction = None
                #self.SaveAndPublishCommand('ERROR')
        
            elif current_status == 'CLOSING' or current_status == 'RESETING' or current_status == 'SETHING':
                print "Added Move at  the end of current action"
                self.NextAction = 'MOVE'

            elif current_status == 'STOPPED':
                print "Starting MOVING"
                self.NextAction = None
                self.SaveAndPublishStatus('MOVING')
                self.Move()

            else:
                print "Unknown combination of command/status : {0}/{1}".format(self.command, self.status)

        else:
            print "Unknown command. VERY weird (from Robot class). For safety, passing in 'ERROR' status"
            self.SaveAndPublishStatus('ERROR')


    def UpdateHeight(self, heightMessage):
        self.NewHeight = heightMessage.data
        print "Set New height to {0}. Waiting for activation command.".format(self.NewHeight)
        #self.SaveAndPublishCommand('SETH')

    def UpdateSpeed(self, speedRatioMessage):
        self.speedRatio = speedRatioMessage.data
        self.computeDeltasT()
        print "Set speed ratio to {0}".format(self.speedRatio)

    def UpdateDirection(self, directionMessage):
        self.Direction = directionMessage.data

    def SaveAndPublishCommand(self, command):
        self.command = command
        print "self publishing command {0}".format(command)
        self.command_publisher.publish(command)

    def SaveAndPublishStatus(self, status):
        self.status = status
        print "Updating status to {0}".format(status)
        self.status_publisher.publish(status)

    def get_relative_point(self, final_absolute_feet_position, flightend_absolute_robot_position, flightend_absolute_robot_orientation, leg_to_raise):
        '''Get final_feet_position in the leg referential when the leg is supposed to land.
        Input :
            - final_absolute_feet_position : 3D np.array giving the absolute position of the desired point, computed at the start of the move
            - flightend_absolute_robot_position : 3D np.array giving the absolute robot position when the leg is supposed to land
            - flightend_absolute_robot_orientation : 3D np.array giving the absolute orientation vector of the robot when the leg is supposed to land
            - leg_to_raise : id of the leg to be raised
        '''
        # We create an artefact robot to get the rotation matrices
        robot_at_landing = Robot(artefact = True, artefact_position = flightend_absolute_robot_position, artefact_orientation = flightend_absolute_robot_orientation)
        final_relative_feet_position = self.Legs[leg_to_raise].rotate_vector_to_leg_from_robot(- self.Legs[leg_to_raise].fix_position
                                                                                               - self.rotate_vector_to_robot_from_absolute(robot_at_landing.position
                                                                                                                                         - final_absolute_feet_position))
        
        return final_relative_feet_position

    def get_relative_orientation(self, flightend_absolute_robot_orientation, leg_to_raise):
        '''Gets the orientation of the landing line in the relative referential of the leg to be raised
        Input :
            - flightend_absolute_orientation : 3D np.array of the orientation of the robot at the end of the flight
            - leg_to_raise : id of the leg to be raised
        '''
        robot_at_landing = Robot(artefact = True, artefact_orientation = flightend_absolute_robot_orientation)
        flightend_landingline_vector = self.Legs[leg_to_raise].rotate_vector_to_leg_from_robot(self.rotate_vector_to_robot_from_absolute(robot_at_landing.orientation))
        return flightend_landingline_vector

    def compute_refchanging_rob_to_abs_matrix(self):
        A = np.sqrt(self.orientation[0]**2+self.orientation[1]**2)
        Ox = self.orientation[0]
        Oy = self.orientation[1]
        Oz = self.orientation[2]
        self.refchanging_rob_to_abs_matrix = np.array([[Ox, -Oy/A, -Oz*Ox/A], [Oy, Ox/A, Oz*Oy/A], [Oz, 0, A]])

    def rotate_vector_to_absolute_from_robot(self, V):
        return np.dot(self.refchanging_rob_to_abs_matrix,V)

    def rotate_vector_to_robot_from_absolute(self, V):
        return np.dot(np.linalg.inv(self.refchanging_rob_to_abs_matrix),V)

    def get_flight_length(self, leg_to_raise):
        return int((self.Legs[leg_to_raise].alphamax-self.Legs[leg_to_raise].alphamin)/(self.MeanAlphaMove*self.srud))

    def close_flights(self, cycle_end):
        '''Routine to end all flights and land the legs still in the air'''
        n_legs_up = 0
        for leg in self.Legs:
            if leg.status == 'up':
                n_legs_up += 1
        print "Closing flight. Still {0} to land".format(n_legs_up)
        while n_legs_up > 0 and self.status == 'CLOSING':
            cycle_end += 1
            # We set the different history variables for this cycle
            self.angles_history += [[]]
            self.absolute_feet_positions_history += [[]]
            self.relative_feet_positions_history += [[]]
            # Update position for this cycle and save it
            self.history += [self.position]
            self.landmark_history += [self.current_landmark]

            for leg in self.Legs:
                if leg.status == 'down': # If the leg is currently on the ground
                    self.angles_history[-1] += [leg.angles] 
                    self.absolute_feet_positions_history[-1] += [leg.absolute_feet_position]
                    self.relative_feet_positions_history[-1] += [leg.relative_feet_position]
                else:
                    if (cycle_end - leg.cycle_takeoff) <= len(leg.flight)-1:
                        leg.follow_flight(cycle_end)
                        if leg.angles == None:
                            print "Leg {0} angles are None. (2514453)".format(leg.leg_id)
                            self.SaveAndPublishStatus('ERROR')
                    else:
                        leg.extend_flight()
                        if leg.angles == None:
                            print "Leg {0} angles are None. (8754215)".format(leg.leg_id)
                            self.SaveAndPublishStatus('ERROR')
                    
                    leg.absolute_feet_position = leg.get_leg_absolute_position(self)
                    print "New absolute position of leg {0} : {1}".format(leg.leg_id,  leg.absolute_feet_position)
                    
                    self.absolute_feet_positions_history[-1] += [leg.absolute_feet_position]
                    self.relative_feet_positions_history[-1] += [leg.relative_feet_position]
                    self.angles_history[-1] += [leg.angles] 
                    leg.check_landing(cycle_end)
                    if leg.status == 'down':
                        n_legs_up -= 1
            if self.status == 'CLOSING':
                self.ConcatenateAnglesAndPublish()

        print "All legs landed at final cycle {0}.".format(cycle_end)
        self.SaveAndPublishStatus('STOPPED')
        self.ExecuteNextAction()

    def ExecuteNextAction(self):
        if self.status != 'STOPPED':
            print "Starting next action while not stopped. WEIRD !"
        if self.NextAction == None:
            print "No next action defined"
        elif self.NextAction == 'RESET':
            self.NextAction = None
            self.ResetLegsPositions(forSetHeight = False)
        elif self.NextAction == 'SETH':
            self.NextAction = None
            self.ResetLegsPositions(forSetHeight = True)
        elif self.NextAction == 'MOVE':
            self.NextAction = None
            self.Move()
        else:
            print "Unknown NextAction"

    def CheckHeight(self, n_legs_down, current_mean_height):
        correction = self.h - current_mean_height
        print "Height correction : {0}".format(correction)
        for leg in self.Legs:
            if leg.status == 'down':
                leg.relative_feet_position += np.array([0., 0., correction])
                leg.update_angles_from_position()
                if leg.angles == None:
                    print "Leg {0} angles are None. (8495845)".format(leg.leg_id)
                    self.SaveAndPublishStatus('ERROR')
                if np.isnan(leg.angles).any():
                    print "Leg {0} angles are None. (4145254)".format(leg.leg_id)
                    self.SaveAndPublishStatus('ERROR')

    def compute_move_data(self, final_position, final_orientation, N_points):
        if final_orientation == None:
            final_orientation = final_position-self.position[:2]
            final_orientation /= norm(final_orientation)

            points_center = [0,0,0]
            points_center[0] = np.linspace(self.position[0], final_position[0], N_points)
            points_center[1] = np.linspace(self.position[1], final_position[1], N_points)
            points_center[2] = np.linspace(self.h, self.h, N_points)
            positions = np.array(points_center)
            orientation_x = np.linspace(self.orientation[0], final_orientation[0], int(self.frm*N_points))
            orientation_y = np.linspace(self.orientation[1], final_orientation[1], int(self.frm*N_points))
            orientation_x /= np.sqrt(orientation_x**2+orientation_y**2)
            orientation_y /= np.sqrt(orientation_x**2+orientation_y**2)
            orientations = [0,0,0]
            orientations[0] = np.array(orientation_x.tolist() + [final_orientation[0] for i in range(N_points - len(orientation_x))])
            orientations[1] = np.array(orientation_y.tolist() + [final_orientation[1] for i in range(N_points - len(orientation_y))])
            orientations[2] = np.linspace(0, 0, N_points)
            orientations=np.array(orientations)

            return positions, orientations
        else:
            final_position = np.copy(self.position)
            orientation_x = np.linspace(self.orientation[0], final_orientation[0], N_points)
            orientation_y = np.linspace(self.orientation[1], final_orientation[1], N_points)
            orientation_x /= np.sqrt(orientation_x**2+orientation_y**2)
            orientation_y /= np.sqrt(orientation_x**2+orientation_y**2)
            orientations = [0,0,0]
            orientations[0] = np.array(orientation_x.tolist())
            orientations[1] = np.array(orientation_y.tolist())
            orientations[2] = np.linspace(0, 0, N_points)
            orientations=np.array(orientations)

            return positions, orientations

    def ResetLegsPositions(self, forSetHeight = False):
        '''Function to reset the legs to their original state'''
        self.delta_publisher.publish(self.DeltaTReset)
        order_th = [0,4,2,3,1,5]
        print "Starting Reset"

        order = []
        for leg in self.Legs:
            if leg.status== 'up':
                order += [leg.leg_id]
        for leg_id in order_th:
            if leg_id not in order:
                order += [leg_id]

        tolerance = 1 #Error tolerance in cm to avoid moving a leg uselessly
        time_one_leg = 1.

        for leg_id in order:
            if self.status != 'RESETING' and self.status != 'SETHING':
                break
            print "Reseting leg {0}".format(leg_id)
            leg = self.Legs[leg_id]
            cycle = 0
            if leg.contact == False or np.linalg.norm(leg.relative_feet_position - np.array([leg.x_repos, leg.y_repos, -leg.h])) > tolerance:
                
                last_publish = 0
                leg.flight = tools.flight(leg.relative_feet_position, np.array([leg.x_repos, leg.y_repos, -leg.h]), leg.h_up*1.05, self.NPointsResetFlight)
                leg.status = 'up'
                leg.cycle_takeoff = 0
                while leg.status != 'down':
                    if self.status != 'RESETING' and self.status != 'SETHING':
                        break
                    cycle += 1
                    if (cycle - leg.cycle_takeoff) <= len(leg.flight)-1:
                        leg.follow_flight(cycle)
                        if leg.angles == None:
                            print "Leg {0} angles are None. (4754126)".format(leg.leg_id)
                            self.SaveAndPublishStatus('ERROR')
                        AnglesMessage = np.array(leg.angles, dtype = np.float32)
                    else:
                        leg.extend_flight()
                        if leg.angles == None:
                            print "Leg {0} angles are None. (7548754)".format(leg.leg_id)
                            self.SaveAndPublishStatus('ERROR')
                    leg.absolute_feet_position = leg.get_leg_absolute_position(self)
                    while time.time()-last_publish < self.DeltaTReset:
                        time.sleep(self.DeltaTReset/10)
                    self.motor_publishers[leg.leg_id].publish(AnglesMessage)
                    last_publish = time.time()
                    leg.check_landing(cycle)
            print "Reset for leg {0} ended at cycle {1}".format(leg_id, cycle)
        self.PublishRobotData()
        if forSetHeight:
            self.SetRobotHeight()
        
        else:
            self.SaveAndPublishStatus('STOPPED')
            self.ExecuteNextAction()

    def SetRobotHeight(self):
        initial_time = time.time()
        self.delta_publisher.publish(self.DeltaTSetHeight)
        print "Setting height from {0} to {1}".format(self.h, self.NewHeight)
        if self.NewHeight != self.h:
            heights = np.linspace(self.h, self.NewHeight, self.NPointsSetHeight)
            last_publish = time.time()

            for current_height in heights[1:]:
                self.position[2] = current_height
                if self.status != 'SETHING':
                    break
                for leg in self.Legs:
                    leg.set_height(current_height)
                    leg.absolute_feet_position = leg.get_leg_absolute_position(self)
                    leg.update_angles_from_position()
                    if leg.angles == None:
                        print "Leg {0} angles are None. (5465546)".format(leg.leg_id)
                        self.SaveAndPublishStatus('ERROR')

                while time.time()-last_publish < abs(self.NewHeight - self.h)*self.DeltaTSetHeight:
                    time.sleep(self.DeltaTSetHeight/10)
                self.ConcatenateAnglesAndPublish()
                self.PublishRobotData()
                last_publish = time.time()
            print "Done after {0} cycles, in {1:.2} seconds.".format(len(heights), time.time() - initial_time)
        else:
            print "Nothing to do here !"
        self.h = self.NewHeight

        self.SaveAndPublishStatus('STOPPED')
        self.ExecuteNextAction()

    def Move(self):
        '''Function that handles the whole movement of the robot.'''
        if self.Direction.tolist() == [0., 0., 1.]:
            print "Relative direction command is NULL. Not moving"
            #self.SaveAndPublishStatus('STOPPED')
        elif self.Direction[2] == 1. and self.Direction[0] == 40.:
            self.GoTo(self.position[:2] + self.rotate_vector_to_absolute_from_robot(self.Direction)[:2])

    def GoTo(self, final_position, rotation_factor = 0.1):
        '''
        Function to move the robot.
        final_position : Desired position as vector [x,y] in the temporary landmark of the robot
        N_points : number of points for this move. Probably useless in the end, necessary for the current algorythm
        Returns an history of the center positions, feet positions, and angles of each leg.
        '''
        CycleOffset = 0
        Cycle = 0
        self.last_takeoff = -self.taxiway_delay_cycles
        N_points = int(np.linalg.norm(np.array(final_position)-np.array(self.position)[:-1])*self.N_points_by_cm)
        self.delta_publisher.publish(self.DeltaTGoto)
        print "Starting move with final_position {0}".format(final_position)

        positions, orientations = self.compute_move_data(final_position, None, N_points)
        print positions

        final_feet_positions =[]
        final_robot = Robot(artefact=True, artefact_position = positions[:,-1], artefact_orientation = orientations[:,-1])
        for leg in self.Legs:
            final_feet_positions += [leg.get_leg_absolute_position(final_robot)]

        last_cycle_time = time.time()
        while self.status == 'MOVING':
        # We start at cycle 1 since 0 is the initial position.
            print ""
            print "Statuses at start of cycle {0} : {1}".format(Cycle, [leg.status for leg in self.Legs])

            Cycle += 1

            if Cycle-CycleOffset > 10:
                if self.status != 'ERROR':
                    if self.Direction[2] == 1 and self.Direction[0] == 40.:
                        print "Updating direction informations"
                        final_position = self.position[:2] + self.rotate_vector_to_absolute_from_robot(self.Direction)[:2]
                        N_points = int(np.linalg.norm(np.array(final_position)-np.array(self.position)[:-1])*self.N_points_by_cm)
                        
                        positions, orientations = self.compute_move_data(final_position, None, N_points)
                        
                        final_feet_positions =[]
                        final_robot = Robot(artefact=True, artefact_position = positions[:,-1], artefact_orientation = orientations[:,-1])
                        for leg in self.Legs:
                            final_feet_positions += [leg.get_leg_absolute_position(final_robot)]
                        CycleOffset = Cycle
                    elif self.Direction[2] == 1 and self.Direction[0] == 0.:
                        print "Relative direction set to null. Starting closing procedure"
                        #print "Robot should move while direction is 0., 0., 1..  Weird. Going into error mode (should do something better)"
                        self.SaveAndPublishStatus('CLOSING')

            # We set the different history variables for this cycle
            self.angles_history += [[]]
            self.absolute_feet_positions_history += [[]]
            self.relative_feet_positions_history += [[]]
            # Update position for this cycle and save it
            self.position = positions[:,Cycle-CycleOffset]
            self.orientation = orientations[:,Cycle-CycleOffset]
            self.history += [self.position]
            self.landmark_history += [self.current_landmark]

            # The demands define the need for a leg to takeoff, and its value traduces the intensity of the need.
            # 0 means no need, 1 a slight need, 2 a real need.
            demands = []

            legs_down = 0
            current_mean_height = 0

            # We start by setting each leg to its new relative/absolute position

            for leg in self.Legs:
                if leg.status == 'down': # If the leg is currently on the ground
                    legs_down += 1
                    leg.relative_feet_position = leg.get_leg_relative_position(self) # We first update the relative position of the grounded legs
                    current_mean_height += leg.relative_feet_position[2]
                    
                elif leg.status == 'up': # If the leg is currently moving in the air, towards a designed position.
                    if (Cycle - leg.cycle_takeoff) <= len(leg.flight)-1:
                        leg.follow_flight(Cycle)
                        if leg.angles == None:
                            print "Leg {0} angles are None. (7539129)".format(leg.leg_id)
                            self.SaveAndPublishStatus('ERROR')
                        leg.absolute_feet_position = leg.get_leg_absolute_position(self)
                        print "New absolute position of leg {0} : {1}".format(leg.leg_id,  leg.absolute_feet_position)
                    elif len(leg.flight)-1 <(Cycle - leg.cycle_takeoff) <=len(leg.flight) -1 + self.leg_contact_wait_cycles:
                        print "Waiting for contact confirmation for leg {0}".format(leg.leg_id)

                    else:
                        leg.extend_flight()
                        if leg.angles == None:
                            print "Leg {0} angles are None. (0215423)".format(leg.leg_id)
                            self.SaveAndPublishStatus('ERROR')
                        leg.absolute_feet_position = leg.get_leg_absolute_position(self)
                        print "New absolute position of leg {0} : {1}".format(leg.leg_id,  leg.absolute_feet_position)
                    
                    if (Cycle - leg.cycle_takeoff) > len(leg.flight)/2:
                        if self.debug:
                            leg.check_landing(Cycle)
                        else:
                            leg.check_landing()
                    
            # Now we start all the corrections for height and orientation of the robot for the legs down, and update the new angles for these relative positions
            current_mean_height *= -1/legs_down
            self.CheckHeight(legs_down, current_mean_height)

            # Once all legs are were they are supposed to be, we check all the envy/needs and save the different data in the history variables

            for leg in self.Legs:
                if leg.status == 'down': # If the leg is currently on the ground
                    self.angles_history[-1] += [leg.angles] 
                    self.absolute_feet_positions_history[-1] += [leg.absolute_feet_position]
                    self.relative_feet_positions_history[-1] += [leg.relative_feet_position]

                    leg_zones = leg.zone_presence() # Now we check the presence of the leg in the different zones
                    if leg_zones['envy'] == True:
                        demands += [0]
                    elif leg_zones['envy'] == False and leg_zones['need'] == True:
                        print "Relative distance from envy to need for leg {1} : {0}".format(leg.get_ratio_distance_from_zone_to_next('envy', 'need'), leg.leg_id)
                        demands += [1]
                    elif leg_zones['need'] == False and leg_zones['critical'] == True:
                        print "Relative distance from need to critical for leg {1} : {0}".format(leg.get_ratio_distance_from_zone_to_next('need', 'critical'), leg.leg_id)
                        demands += [2]
                    elif leg_zones['critical'] == False:
                        print "Leg {0} passed critical boundary !".format(leg.leg_id)
                        self.SaveAndPublishStatus('ERROR')
                    else:
                        print "Unknown leg_zones status. Probable definition issue"
                        self.SaveAndPublishStatus('ERROR')
                
                elif leg.status == 'up': # If the leg is currently moving in the air, towards a designed position.
                    self.absolute_feet_positions_history[-1] += [leg.absolute_feet_position]
                    self.relative_feet_positions_history[-1] += [leg.relative_feet_position]
                    self.angles_history[-1] += [leg.angles] 

                    demands += [0]

                else:
                    print "Unknown status for leg {0} : {1}".format(leg.leg_id, leg.status)
                    self.SaveAndPublishStatus('ERROR')

            # Now, all legs positions have been updated. We now check the demands of each leg, and update the statuses
            print "{0} legs are on the ground after update of cycle {1}".format(legs_down, Cycle)

            if demands.count(0) == self.N_legs:
                print "Demands before alphas condition check are {0}".format(demands)
                # Here, no demand was made, thus all legs are inside the 'envy' zone
                if legs_down > self.minimum_legs_down: 
                    N_legs_sameside = 0
                    for leg in self.Legs:
                        if (leg.side == 'right' and leg.angles[0] < self.alpha_margin_sameside) or (leg.side == 'left' and leg.angles[0] > self.alpha_margin_sameside):
                            N_legs_sameside += 1
                    if N_legs_sameside > self.max_sameside_leg_number:
                        # and they are too many legs on the same negative side and too many are on the floor
                        distances = []
                        for leg in self.Legs:
                            if (leg.status == 'down' 
                                and not leg.has_neighbour_up(self) 
                                and ((leg.side == 'right' and leg.angles[0] < self.alpha_margin_sameside) 
                                        or (leg.side == 'left' and leg.angles[0] > self.alpha_margin_sameside))): # The potential candidates are the ones down, with no neighbour in the air and breaking the alpha condition.
                                distances += [leg.get_ratio_distance_from_zone_to_next('center', 'envy')]
                            else:
                                distances += [0]
                        print "Too many legs found backwards. Maximum distance found for leg {0} at value {1}".format(distances.index(np.max(distances)), np.max(distances))
                        demands[distances.index(np.max(distances))] = 1

                elif  (np.array(R.angles_history[0])[:,0] > 0).sum() > self.max_sameside_leg_number:
                    # and they are too many legs on the same positive side, meaning we moved forward too many of them !
                    print "Algorithmic error, too many legs moved forward, can't keep up with the rules !"
                    self.SaveAndPublishStatus('ERROR')
            print "Finals demands for cycle {1} are {0}".format(demands, Cycle)
                        
            if sum(demands)>0:
                # If at least one leg asked for takeoff
                if demands.count(np.max(demands)) == 1:
                    # If the higher demand was asked once, then it is the only priority and it is selected
                    leg_to_raise = demands.index(np.max(demands))
                else:
                    # Else it means we have a conflict we must solve
                    N_zones=[]
                    for leg in self.Legs:
                        if demands[leg.leg_id] == np.max(demands):
                            N_zones += [leg.zone_presence().values().count(True)]
                        else:
                            N_zones += [10]
                    print N_zones
                    if N_zones.count(min(N_zones)) == 1:
                        # Here we look for the minimum of zone presence. A leg only inside critical boundaries will have N_zones=1. If this minimum is reached only once, then we select it
                        leg_to_raise = N_zones.index(min(N_zones))
                    else:
                        # If once again, we have a conflict, we must go one step further and check the maximum ratio distance
                        ratios = []
                        for leg in self.Legs:
                            if leg.status == 'down' and N_zones[leg.leg_id] == min(N_zones):
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
                        leg_to_raise = ratios.index(np.max(ratios))
            else:
                leg_to_raise = None

            if leg_to_raise == None:
                print "No leg to be raised at the end of cycle {0}".format(Cycle)
            else:
                if not self.Legs[leg_to_raise].has_neighbour_up(self) and legs_down > self.minimum_legs_down and (2 in demands or Cycle-self.last_takeoff >= self.taxiway_delay_cycles):
                    print "Leg {0} is going to be raised at the end of cycle {1}".format(leg_to_raise, Cycle)
                    N_points_flight = self.get_flight_length(leg_to_raise)
                    if (Cycle-CycleOffset)+N_points_flight > N_points-1:
                        N_points_aimed = N_points_flight 
                        N_points_flight = (N_points-1)-(Cycle-CycleOffset)
                    else:
                        N_points_aimed = N_points_flight
                    self.Legs[leg_to_raise].initiate_flight(Cycle, self.get_relative_point(final_feet_positions[leg_to_raise], positions[:,(Cycle-CycleOffset)+N_points_flight], orientations[:,(Cycle-CycleOffset)+N_points_flight], leg_to_raise), self.get_relative_orientation(orientations[:,(Cycle-CycleOffset)+N_points_flight], leg_to_raise), N_points_flight, N_points_aimed)
                    self.last_takeoff = Cycle
                else:
                    print "Leg {0} should takeoff but current conditions forbid it. Reason :".format(leg_to_raise)
                    if self.Legs[leg_to_raise].has_neighbour_up(self):
                        print "Neighbour currently in the air"
                    if legs_down <= self.minimum_legs_down:
                        print "Too many legs in the air"
                    if Cycle-self.last_takeoff < self.taxiway_delay_cycles:
                        print "Must wait {0} cycles before possible takeoff".format(-(Cycle-self.last_takeoff) + self.taxiway_delay_cycles)

            while time.time() - last_cycle_time < self.DeltaTGoto:
                time.sleep(self.DeltaTGoto/10)
            if self.status == 'MOVING':
                self.ConcatenateAnglesAndPublish()
                self.PublishRobotData()
                last_publish = time.time()
        print "End of main GoTo loop at cycle {0} with exiting status {1}. Starting the close_flights procedure".format(Cycle, self.status)
        if self.status != 'ERROR':
            self.close_flights(Cycle)
        if self.status != 'ERROR':
            self.SaveAndPublishStatus('STOPPED')
            self.ExecuteNextAction()

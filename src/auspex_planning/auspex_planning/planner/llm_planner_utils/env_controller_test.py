#!/usr/bin/env python3
import os
import math
import csv
from itertools import product
import rclpy
from rclpy.node import Node
from auspex_msgs.action import ExecuteAction
from auspex_msgs.srv import AddActionString
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
import torch.distributed as dist
from geographic_msgs.msg import GeoPoint
import airsim
from enum import Enum


class DroneStateInfo(Enum):
    LANDED = 1
    AIRBORNE = 2

TAKE_OFF_CONST = "Take off"
LAND_CONST = "Land"

class DetectedObject():
    def __init__(self):
        self._detection_time = 0
        self._detection_location = ""
        self._detection_specifier = ""
        self._veloctiy = [0,0,0]

class DroneState():

    def __init__(self, airborne_state=DroneStateInfo.LANDED, currentWaypoint="home"):
        self._lat = 0.0
        self._lon = 0.0
        self._alt = 0.0
        self._airborne_state = airborne_state
        self._detected_object = "?" # object -> detection count | object -> detection count | ...
        self._camera = "RGB_CAMERA" # "NO_CAMERA" | "RGB_CAMERA" | "THERMAL_CAMERA"
        self._payload = "FIRST_AID_KIT" # ....

        self._detection_state = ""
        self._currentWaypoint = currentWaypoint
        self._searchedWaypoints = []
        self._battery_state = 1.0



class EnvController(Node):
    '''
    Init function to initialize basic values 
    '''
    sim_target = "Backpack2_2"
    camera_name = 'front_30'
    image_type = airsim.ImageType.Scene

    def __init__(self):
        super().__init__('auspex_env_controller')


        self.n_detection=0.0
        self.n_totalinference=0.0
        self.probability=0.0
        self.n_notdetections=0.0
        self.n_detection_this_flight = 0
        
        self._drone_home = self.getAirSimPose(0,0,0,0,0,0,0)
        self._openareas_altitude = 13.0
        self._woods_altitude = 30.0
        self._urbanareas_altitude = 20.0

        self._winter_openareas_altitude = 18.0
        self._winter_woods_altitude = 35.0
        self._winter_urbanareas_altitude = 25.0

        self._summer_openareas_positions = self.getGeoPoints(48.00983636505821, 11.898978749762726, self._openareas_altitude, 48.00896269725013, 11.899292535510558, self._openareas_altitude)
        self._summer_woods_positions = self.getGeoPoints(48.00751462582585, 11.897287892897287, self._woods_altitude, 48.00736976377351, 11.895291810659725, self._woods_altitude)
        self._summer_urbanareas_positions = self.getGeoPoints(48.010661331969686, 11.900876338503116, self._urbanareas_altitude, 48.01146103509754, 11.90169958145985, self._urbanareas_altitude)

        self._winter_openareas_positions = self.getGeoPoints(48.013727370252056, 11.89856340004075, self._winter_openareas_altitude, 48.013677226878336, 11.899785056062669, self._openareas_altitude)
        self._winter_woods_positions = self.getGeoPoints(48.0140015055061, 11.8986907494182574, self._winter_woods_altitude,48.01399989858494, 11.900108681692874, self._woods_altitude)
        self._winter_urbanareas_positions = self.getGeoPoints(48.01353206670589, 11.898902242819197, self._winter_urbanareas_altitude, 48.01423823526816, 11.898813560025548, self._urbanareas_altitude)


        self._summer_position = []
        self._summer_position.append(self._summer_openareas_positions)
        self._summer_position.append(self._summer_woods_positions)
        self._summer_position.append(self._summer_urbanareas_positions)

        self._winter_position = []
        self._winter_position.append(self._winter_openareas_positions)
        self._winter_position.append(self._winter_woods_positions)
        self._winter_position.append(self._winter_urbanareas_positions)

        self._positions = []
        self._positions.append(self._summer_position)
        self._positions.append(self._winter_position)
       
        self._droneState = DroneState()

        self._sim_connected = True
        self._current_action = ""
        self._is_executing = False

        spawn_openareas_summer = self.getAirSimPose(x=-56.39999771118164,y=12.5,z=2.700000047683716,ox=-0.41070184111595154,oy=0.6125814318656921,oz=-0.5609269738197327,ow=0.37607038021087646)
        spawn_woods_summer = self.getAirSimPose(x=-275.6000061035156,y=-206.59999084472656,z=-6.399999618530273,ox=-0.41070184111595154,oy=0.6125814318656921,oz=-0.5609269738197327,ow=0.37607038021087646)
        spawn_urbanareas_summer = self.getAirSimPose(x=145.09999084472656,y=173.8000030517578,z=3.5,ox=-0.41070184111595154,oy=0.6125814318656921,oz=-0.5609269738197327,ow=0.37607038021087646)
        
        spawn_openareas_winter = self.getAirSimPose(x=428.3999938964844,y=23.69999885559082,z=12.799999237060547,ox=-0.493563175201416,oy=-0.4873982071876526,oz=0.5061211585998535,ow=0.5125229358673096)
        spawn_woods_winter = self.getAirSimPose(x=461.0,y=25.100000381469727,z=12.799999237060547,ox=-0.493563175201416,oy=-0.4873982071876526,oz=0.5061211585998535,ow=0.5125229358673096)
        spawn_urbanareas_winter = self.getAirSimPose(x=461.5,y=-9.09999942779541,z=12.799999237060547,ox=-0.493563175201416,oy=-0.4873982071876526,oz=0.5061211585998535,ow=0.5125229358673096)
        
        self._spawn_out_of_bounds = self.getAirSimPose(x=-29.099998474121094,y=5.900000095367432,z=-100,ox=-0.41070184111595154,oy=0.6125814318656921,oz=-0.5609269738197327,ow=0.37607038021087646)
        
                                #"MAN":"SkeletalMeshActor_UAID_60452E0D3E95391002_1587704425"
        self.object_map = {"PERSON_DEFAULT":"SkeletalMeshActor_UAID_60452E0D3E95741702_1466732200",
                           "PERSON_WHITE":"SkeletalMeshActor_UAID_107C6175D67D801702_1729292313",
                           "PERSON_BLACK":"SkeletalMeshActor_UAID_107C6175D67D821702_2045225664",
                           "PERSON_YELLOW":"SkeletalMeshActor_UAID_107C6175D67D831702_1240334841",
                           "PERSON_BLUE":"SkeletalMeshActor_UAID_107C6175D67D831702_1319628842",
                           "PERSON_GREEN":"SkeletalMeshActor_UAID_107C6175D67D831702_1366316843",
                           "PERSON_ORANGE":"SkeletalMeshActor_UAID_107C6175D67D831702_1456467844",
                           "PERSON_PURPLE":"SkeletalMeshActor_UAID_107C6175D67D831702_1530599845",
                           "PERSON_RED":"SkeletalMeshActor_UAID_107C6175D67D811702_1099589487"}

        self.person_color_list = ["PERSON_BLUE", "PERSON_RED", "PERSON_YELLOW","PERSON_ORANGE", "PERSON_GREEN", "PERSON_PURPLE", "PERSON_WHITE", "PERSON_BLACK"]
        self.child_color_list = ["PERSON_BLUE", "PERSON_RED", "PERSON_YELLOW","PERSON_ORANGE", "PERSON_GREEN", "PERSON_PURPLE", "PERSON_WHITE", "PERSON_BLACK"]
        self.object_list = []
        self.object_list.append(self.person_color_list)
        self.object_list.append(self.child_color_list)
        
        self.object_map
        
        self._summer_object_spawn = []
        self._summer_object_spawn.append(spawn_openareas_summer)
        self._summer_object_spawn.append(spawn_woods_summer)
        self._summer_object_spawn.append(spawn_urbanareas_summer)

        self._winter_object_spawn = []
        self._winter_object_spawn.append(spawn_openareas_winter)
        self._winter_object_spawn.append(spawn_woods_winter)
        self._winter_object_spawn.append(spawn_urbanareas_winter)

        self._object_spawn = []
        self._object_spawn.append(self._summer_object_spawn)
        self._object_spawn.append(self._winter_object_spawn)


        '''
        ROS2 Client for sending action
        '''
        if self._sim_connected:
            """
            Initialize clients, enable API control and arm the drone
            """
            self.image_shape=(640,480)
            ip_addr = os.environ["WSL_HOST_IP"]

            self._env_client = airsim.MultirotorClient(ip_addr)
            self._env_client.confirmConnection()     
            
            #self.drone_state = self._env_client.getMultirotorState("Drone1")

            self._action_client = ActionClient(self, ExecuteAction, '/vhcl0/execute_action_action_server')

            while not self._action_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')

        #print(self.getListSceneObjects())
        # for object in self.getListSceneObjects():
        #     if "SkeletalMeshActor_UAID_" in object:
        #         print(object)
        print(self.getObjectPose(self.object_map['PERSON_DEFAULT'])[1])
    
        self.reset_objects()


    def getGeoPoints(self, lat1, lon1, alt1, lat2, lon2, alt2):
        first_wp = GeoPoint()
        first_wp.latitude = lat1
        first_wp.longitude = lon1
        first_wp.altitude = alt1

        second_wp = GeoPoint()
        second_wp.latitude = lat2
        second_wp.longitude = lon2
        second_wp.longitude = lon2
        second_wp.altitude = alt2
        
        positions = []
        positions.append(first_wp)
        positions.append(second_wp)
        return positions

    def getDroneState(self):
        return self._env_client.getMultirotorState("Drone1")

    def getObjectPose(self, identifier):
        pose =  self._env_client.simGetObjectPose(identifier)
        pose_string = f"(x={pose.position.x_val},y={pose.position.y_val},z={pose.position.z_val},ox={pose.orientation.x_val},oy={pose.orientation.y_val},oz={pose.orientation.z_val},ow={pose.orientation.w_val})"
        return [pose, pose_string]

    def getListSceneObjects(self):
        return self._env_client.simListSceneObjects()

    def setObjectScale(self, identifier="", scale=1.0):
        self._env_client.simSetObjectScale(identifier, airsim.Vector3r(scale,scale,scale))

    def setObjectPose(self, object_name, pose): 
        object_name_UE = self.object_map[object_name]
        self._env_client.simSetObjectPose(object_name_UE, pose)

    def getHomeGPSCoordinates(self):
        coord = self._env_client.getHomeGeoPoint(vehicle_name="Drone1")
        return coord #altitude latitude ,longitude 

    def getGPSCoordinates(self):
        coord = self._env_client.getGpsData(vehicle_name="Drone1")
        return coord #altitude latitude ,longitude 

    def getDistanceToTarget(self, target=""):
        distance = 0.0
        target_pos = self._env_client.simGetObjectPose(self.object_map[target])[0]
        drone_pos = self._env_client.simGetVehiclePose("Drone1")
        distance = self.computeDistance3D(drone_pos, target_pos)
        return distance

    def computeDistance3D(self, pos1, pos2):
        x_p2 = (pos2.position.x_val - pos1.position.x_val)**2
        y_p2 = (pos2.position.y_val - pos1.position.y_val)**2
        z_p2 = (pos2.position.z_val - pos1.position.z_val)**2
        return (math.sqrt(x_p2 + y_p2 + z_p2))

    def getAirSimPose(self, x=0.0,y=0.0,z=0.0, ox=0.0,oy=0.0,oz=0.0,ow=0.0):
        pose = airsim.Pose()
        pose.position.x_val = x
        pose.position.y_val = y
        pose.position.z_val = z
        pose.orientation.x_val = ox
        pose.orientation.y_val = oy
        pose.orientation.z_val = oz
        pose.orientation.w_val = ow
        return pose

    '''
    Sends an action to the auspex_main to be executed
    '''
    def send_action(self, actions):
        self._is_executing = True

        if self._sim_connected:
            req = ExecuteAction.Goal()
            req.actions = actions
            self.future = self._action_client.send_goal_async(req,feedback_callback=self.action_feedback_callback)
            self.future.add_done_callback(self.goal_response_callback)
            if len(actions) >1:
                self._current_action = actions[1]
            else:
                self._current_action = actions[0]

    def goal_response_callback(self, future):
        self._action_goal_handle = future.result()
        if not self._action_goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self._get_result_future = self._action_goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        if future.result().result.success == True:
            print(future.result().result.result)
            if TAKE_OFF_CONST in self._current_action:
                return_airborne_state = DroneStateInfo.AIRBORNE
            elif LAND_CONST in self._current_action:
                return_airborne_state = DroneStateInfo.LANDED
        self._is_executing = False


    def action_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        detected_objects = feedback.detection_result
        
        self._droneState._lat = feedback.gps_position.latitude
        self._droneState._lon = feedback.gps_position.longitude
        self._droneState._alt = feedback.gps_position.altitude

        if detected_objects == "POSITION_ONLY_FEEDBACK":
            #callback also triggered from the position feedback 
            return 

        self.n_totalinference=self.n_totalinference+1
        if "person" in detected_objects.lower():
            self.n_detection=self.n_detection+1
            self.n_detection_this_flight = self.n_detection_this_flight + 1

        

    def cancel_goal(self):
        if self._sim_connected:
            cancel_future = self._action_goal_handle.cancel_goal_async()

    def cancel_callback(self, future):
        print("Successfully canceled previous actions...")


    #################################################################################################################################
    #  ['take_off', height_in_metres] # takeoff to height in metres above ground 
    # ['land'] # Land at current position
    # ['fly_home_and_land'] # flies to home position, lands and terminates the program
    # ['ascend', 'amsl|agl|rel', height_in_metres] # ascend either above mean sea level (amsl), above ground level (agl), relative to current height (rel)
    # ['descend', 'amsl|agl|rel', height_in_metres] # descend either above mean sea level (amsl), above ground level (agl), relative to current height (rel)
    # ['hover', duration] # hover for duration in milliseconds (10000ms -> 10 seconds)
    # ['fly_2D', lat, lon] # flies in current height to next lat lon coordinates

    def step(self, action):
        self.send_action(action)

    def reset_objects(self):
       for objects in self.object_list:
            for color in objects:
                self.setObjectPose(color, self._spawn_out_of_bounds)


    #['Season', 'Location', 'Object', 'Color', 'N_Detections', 'N_TotalInference', 'Probability', 'N_NotDetections']
    def fill_and_write_csv(self, season, location, object, color, n_detection, n_totalinference, probability, n_boolean_detections):
        data = {'Season': season,
                'Location': location,
                'Object': object,
                'Color': color,
                'N_Detections': n_detection,
                'N_TotalInference':n_totalinference,
                'Probability':probability,
                'N_Detections_T_F':n_boolean_detections
                }       

        write_to_csv(data, file_name="output/results.csv")


    def rollout(self, mte=None):

        action_execution_list = []
        action_execution_list.append("['take_off', 10.0]")
        action_execution_list.append(f"['fly_3D_step', {self._summer_urbanareas_positions[1].latitude}, {self._summer_urbanareas_positions[1].longitude}, {self._summer_urbanareas_positions[1].altitude}]")
        action_execution_list.append([f"['fly_3D_step', 48.013476339177885, 11.899189742014773,50]"])
        


        count = 0

        steps = -2
        max_steps = 20

        current_season = 1 # 0 == summer ||  1 == winter
        current_location = 2 # 0 == openareas || 1 == woods || 2 == urbanareas
        current_object = 0
        current_color = 3

        next_position = None
        next_object_pose = self._object_spawn[current_season][current_location]
        self.reset_objects()
        self.setObjectPose(self.object_list[current_object][current_color], next_object_pose)

        if current_object == 1:
            self.setObjectScale(self.object_map[self.object_list[current_object][current_color]], 0.5)
        else:
            self.setObjectScale(self.object_map[self.object_list[current_object][current_color]], 1.0)

        self.step(["['take_off', 10.0]"])
        n_detection_per_flight = 0

        while True:
            #print(f"Step: {steps}")
            try:
                mte.spin_once(timeout_sec=0.1)
                if self._is_executing:
                    continue

                if steps == -2:
                    if current_season == 1:
                        self.step([f"['fly_3D_step', 48.013476339177885, 11.899189742014773, 40.0]"])
                    steps = -1
                elif steps == -1:
                    next_position = self._positions[current_season][current_location][count]
                    next_action = [f"['fly_3D_step', {next_position.latitude}, {next_position.longitude}, {next_position.altitude}]"]
                    print(f"Sending initial action: {next_action}")
                    self.step(next_action)
                    steps = 0
                    continue

                elif steps < max_steps:
                    if self.n_detection_this_flight > 0:
                        n_detection_per_flight += 1
                        self.n_detection_this_flight = 0

                    next_position = self._positions[current_season][current_location][count]

                    next_actions = ["['startDetection']"]
                    next_actions.append(f"['fly_3D_step', {next_position.latitude}, {next_position.longitude}, {next_position.altitude}]")
                    next_actions.append("['stopDetection']")

        
                    print(f"Sending next actions: {next_actions}")
                    self.step(next_actions)
                    steps += 1
                    count += 1
                    count = count%2
                else:
                    '''
                    Store data
                    '''
                    n_frames = 5.0
                    if current_season == 0 and current_location == 2:
                        n_frames = 10.0


                    self.fill_and_write_csv(season=current_season, 
                                            location=current_location, 
                                            object=current_object, 
                                            color=current_color, 
                                            n_detection=self.n_detection, 
                                            n_totalinference=self.n_totalinference, 
                                            probability=n_detection_per_flight/20.0, 
                                            n_boolean_detections=n_detection_per_flight)
                    
                    self.n_detection = 0
                    self.n_totalinference = 0.0
                    self.n_notdetections = 0.0
                    self.probability = 0.0
                    n_detection_per_flight = 0

                    count = 0
                    steps = -1

                    current_color += 1
                    if current_color >= len(self.object_list[current_object]):
                        current_color = 0
                        current_object += 1
                        if current_object >= len(self.object_list):
                            current_object = 0
                            current_location += 1
                            if current_location >= len(self._positions[current_season]):
                                current_location = 0
                                current_season += 1
                                if current_season >= len(self._positions):
                                    print("Finished training!")
                                    break 

                    next_object_pose = self._object_spawn[current_season][current_location]
                    self.reset_objects()
                    self.setObjectPose(self.object_list[current_object][current_color], next_object_pose)
                    if current_object == 1:
                        self.setObjectScale(self.object_map[self.object_list[current_object][current_color]], 0.5)


                # self.setObjectScale(self.object_map["MAN_DEFAULT"], 100.0)
                

            except KeyboardInterrupt:
                break
        
        self.step(["['fly_home_and_land']"])
            



def main(args=None):
    print("Starting auspex Env Controller")
    rclpy.init(args=args)

    mte = MultiThreadedExecutor()
    env_controller = EnvController()
    mte.add_node(env_controller)
    
    env_controller.rollout(mte)

    mte.shutdown()
    rclpy.shutdown()



def write_to_csv(data, file_name='results.csv'):
    """
    Writes data to a CSV file. If the file does not exist, it creates it and adds a header.
    :param data: A dictionary containing the data to write. Keys are column names.
    :param file_name: The name of the CSV file.
    """
    # Define the CSV header as a list of column names.

    header = ['Season', 'Location', 'Object', 'Color', 'N_Detections', 'N_TotalInference', 'Probability', 'N_Detections_T_F']

    # Check if the file exists
    file_exists = os.path.isfile(file_name)

    # Open the file in append mode ('a') so we can write without overwriting existing data.

    with open(file_name, 'a', newline='') as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=header)
        # If the file does not exist, write the header first.
        if not file_exists:
            writer.writeheader()

        # Write the data.
        writer.writerow(data)


if __name__=='__main__':
    main()






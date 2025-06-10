

class EnvDiscription:

    env_description ='''In the following the operation area is divided into multiple areas named by their category. The areas are listed with a respective centre coordinate and the respective lower and upper bounds of their bounding box:\n'''

    instruction_prompt = """\n**Instruction**:
                You are controlling a quadrocopter. The quadracopter has an rgb camera together with an object detection algorithm installed. The quadracopter has the following possible actions: {action_params_description} \n These are a set of actions. Some of them are useful to control a drone.\n
                This means, you are given a goal and you will then reason an adequat action for the quadrocopter to execute.
                Include the Take off action once to start the drone.
                **Every area can only be searched once.**
                **DO NEVER INCLUDE NOT GOAL-RELEVANT ACTIONS. If a action is unhelpful to reach a goal just ignore it.**
                **IMPORTANT: Avoid choosing actions which are not suitable for this scenario.**
                **Choose only actions which help the progress of the goal**
                Always evaluate if the selected action is applicable in a quadcopter scenario.
                After either reaching the goal, or the drone has visited all related areas use the "Return to Home and Land" action to fly to the home position, land and temrinate the program.
                Please select the appropriate actions to achieve the goal, ensuring that the drone lands once the goal has been reached and no further actions are necessary.

                **Previous actions of the drone were:**
                {actions}
                **Goal:**
                {goal_prompt}
                The {search_object} is likely to be in the {search_area} area.
                """
    look_up_action_list_without_params = ["Return to Home and Land.", "Take off", "Search an area", "Hover", "Confirm an object", "Ascend", "Descend", "Perform mapping", "Take one single image", "Continue with action"]

    action_string_with_description = '''
    - Return to Home and Land. | Description: Quadcopter flies to the home position and lands.
    - Take off to height_in_metres metres altitude. | Description: Quadcopter takes off and ascends to given altitude above ground.
    - Search the area around operation_area for a object description. | Description: Quadcopter searches the given area for an object, while the object detection is activated.
    - Hover for duration milliseconds at the current position. | Description: Quadcopter hovers at the current position for a given time.
    - Confirm a detected Object. | Description: If an object is detected, the Quadcopter circles it and tries to confirm the detected object.
    - Ascend to height_in_metres metres altitude. | Description: Quadcopter ascends to a given altitude.
    - Descend to height_in_metres metres altitude. | Description: Quadcopter descends to a given altitude.
    - Map the area around operation_area. | Description: Quadcopter maps a given area.
    - Take one single image. | Description: Quadcopter takes one single image.
    - Continue with action. | Description: Resumes the current action, which was paused.
    '''

    person_coordinates = [48.00900663404765, 11.90073503048528]

    heights = [20,30,50]
    durations = [10000]
    objects = ['person', 'backpack']#, 'cat','car']
    wps = ['openareas0', 'openareas1', 'openareas2', 'woods0', 'woods1','woods2', 'woods3', 'woods4', 'woods5', 'urbanareas0', 'urbanareas1']#'openareas3', 'openareas4', 'openareas5'

    seasons = ['summer', 'winter']
    detection_color = [
            'red',
            'green',
            'blue',
            'yellow',
            'orange',
            'purple',
            'black',
            'white',
            'gray',
            'NOT_SPECIFIED'
        ]

    goals = [
            'SEARCH',
            'FOLLOW',
            'FIND',
            'INSPECT',
            'SURVEY',
            'MONITOR',
            'NOT_SPECIFIED'
        ]

    #durations = [5000,10000]
    #heights = [20,30,50,10,60,70,80]
    #objects = ['person', 'child', 'cat','car', 'tiger', 'waffle', 'tank','truck', 'bottle', 'drone', 'monitor', 'pc', 'pencil', 'board', 'cat1', 'cat2', 'cat4', 'child', 'woman', 'bjoern', 'cobra', 'megatron', 'prime', 'sheet', 'sheep', 'motor', 'engine', 'rpi', 'kai', 'slack', 'rucksack', 'jacket', 'stock', 'firstaid', 'remote', 'controller', 'xbox', 'ps5', 'ps4', 'ps3', 'ps2', 'ps1', 'yo', 'mouse', 'keyboard', 'water', 'juice']
    #wps = ['woods2', "woods3",  "urbanareas0"]
    #wps_actions = ['openareas0', 'openareas1', 'openareas2', 'woods0', 'woods1','woods2', 'woods3', 'woods4', 'woods10', 'urbanareas14', 'urbanareas11','openareas3', 'openareas4', 'openareas5', 'woods5', 'woods6','woods7', 'woods8', 'woods49', 'woods9', 'urbanareas014', 'urbanareas15', 'woods15', 'woods16', 'woods17', 'woods18', 'woods19', 'woods20', 'woods21', 'woods22', 'woods23', 'woods24', 'woods25', 'woods26', 'woods27', 'woods28', 'woods29', 'woods30', 'woods31', 'woods32', 'woods33', 'woods34', 'woods35', 'woods36', 'woods37', 'woods38', 'woods39', 'woods40', 'woods41', 'woods42', 'woods43', 'woods44', 'woods45','woods46','woods47','woods48','woods49','woods50','woods51','woods52','woods53','woods54','woods55','woods56','woods57','woods58','woods59','woods60','woods61','woods62','woods63','woods64','woods65','woods66','woods67','woods68','woods69','woods70']#'openareas3', 'openareas4', 'openareas5'

    def __init__(self) -> None:
        pass

    def stripActionList(self, action_list):
        #["Return to Home and Land.", "Take off", "Search an area", "Hover", "Confirm an object", "Ascend", "Descend", "Perform mapping", "Take one single Image"]
        return_array = []
        for action in action_list:
            if "Take off" in action:
                return_array.append(self.look_up_action_list_without_params[1])
            elif "Land" in action:
                return_array.append(self.look_up_action_list_without_params[0])
            elif "Continue" in action:
                return_array.append(self.look_up_action_list_without_params[9])
            elif "Search" in action:
                return_array.append(self.look_up_action_list_without_params[2])
            elif "Hover" in action:
                return_array.append(self.look_up_action_list_without_params[3])
            elif "Confirm" in action:
                return_array.append(self.look_up_action_list_without_params[4])
            elif "Ascend" in action:
                return_array.append(self.look_up_action_list_without_params[5])
            elif "Descend" in action:
                return_array.append(self.look_up_action_list_without_params[6])
            elif "Map" in action:
                return_array.append(self.look_up_action_list_without_params[7])
            elif "single image" in action:
                return_array.append(self.look_up_action_list_without_params[8])
        return return_array


    def getWaypointFromString(self, action):
        for wp in self.wps:
            if wp in action:
                return wp
        return None

    def getObjectFromString(self, action):
        for object in self.objects:
            if object in action:
                return object
        return None

    def getDurationFromString(self, action):
        for duration in self.durations:
            if str(duration) in action:
                return duration
        return None

    def getHeightFromString(self, action):
        for height in self.heights:
            if str(height) in action:
                return height
        return None

    def getUniqueLocations(self):
        unique_locations = set()  # Use a set to store unique values
        for item in self.wps:
            # Remove the digits from the end of the string
            location = ''.join(filter(str.isalpha, item))
            unique_locations.add(location)
        return list(unique_locations)

    def getGeneralLocation(self, waypoint):
        if "woods" in waypoint:
            return "woods"
        if "openareas" in waypoint:
            return "openareas"
        if "urbanareas" in waypoint:
            return "urbanareas"
        return None

    def NL2action_msg(self, action, num_actions):

        if "Land" in action:
            return ['return_home_and_land']
        elif "Search" in action:
            wp = self.getWaypointFromString(action)
            loc = self.getGeneralLocation(wp)
            height = 10.0
            if loc == "woods":
                height = 20.0
            elif loc == "openareas":
                height = 15.0
            elif loc == "urbanareas":
                height = 10.0

            s = ['searchArea', 'ahp', height, self.getObjectFromString(action), wp]
            return s
        elif "Hover" in action:
            return ['hover', self.getDurationFromString(action)]
        elif "Confirm" in action:
            #return f"['circle_poi', 'agl', 30, {self._droneState._detected_objects[-1]._detection_location.latitude}, {self._droneState._detected_objects[-1]._detection_location.longitude}]"
            return ['circle_poi', 'agl', 30, 0, 0]
        elif "Ascend" in action:
            return ['ascend', 'rel', self.getHeightFromString(action)]
        elif "Descend" in action:
            return ['descend', 'rel', self.getHeightFromString(action)]
        elif "Map" in action:
            return ['scanArea', 'ahp', 40, self.getWaypointFromString(action)]
        elif "Take one single" in action:
            return ['takeImage']
        elif "Take off" in action:
            return ['take_off', self.getHeightFromString(action)]
        return action
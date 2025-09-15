from ament_index_python.packages import get_package_share_directory
from collections import defaultdict
import numpy as np
from .utils import *
import csv

def read_csv_to_nested_dict(csv_file):
    # Initialize the nested dictionary
    results = defaultdict(lambda: defaultdict(lambda: defaultdict(lambda: defaultdict(dict))))
    # Open and read the CSV file
    try:
        with open(csv_file, mode='r') as file:
            reader = csv.DictReader(file)
            # Step 2: Populate the nested dictionary
            for row in reader:
                season = int(row['Season'])
                location = int(row['Location'])
                object_ = int(row['Object'])
                color = int(row['Color'])
                # Store the number of detections and probability
                results[season][location][object_][color] = {
                    'N_Detections': float(row['N_Detections']),
                    'Probability': float(row['Probability']),
                    'N_Detections_T_F': float(row['N_Detections_T_F']),
                }
    except Exception as e:
        return None
    return results


class CanModel():
    def __new__(cls):
        package_share_directory = get_package_share_directory('auspex_planning')
        workspace_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(package_share_directory))))
        search_weights_path = os.path.join(workspace_dir, "src", 'auspex_planning', 'auspex_planning', "planner", "llm_planner_utils", 'search_weights.csv')
        learned_params = read_csv_to_nested_dict(search_weights_path)
        if learned_params is None:
            return None

        instance =  super().__new__(cls)
        instance._learned_params = learned_params

        return instance

    def __init__(self):
        self._object_list = self._learned_params[0][0].keys()
        self._color_list = self._learned_params[0][0][0].keys()

    '''
    Computes the affordance
    '''
    def compute_affordance_score(self, droneState, detected_objects,  actions, planner):
        aff_scores = np.ones(len(actions))
        version = "learned"
        p_detection = 0.5
        p_conditions = 1/97.0 #Number of permutations
        platform_status = droneState.get('platform_status', 'UNKNOWN')

        try:
            for index, action in enumerate(actions):
                '''
                Learned Probs in unreal engine
                '''

                _openareas_altitude = 10.0
                _woods_altitude = 30.0
                _urbanareas_altitude = 25.0

                _winter_openareas_altitude = 10.0 #20.0
                _winter_woods_altitude = 30.0 #37.0
                _winter_urbanareas_altitude = 25.0 # 20.0

                #P(detected_person | season, location, rgb_camera, color, size)
                aff_scores[index] = 0.0

                '''
                Get an affordance for each action in the list
                '''
                if "search" in action.lower():
                    if platform_status == "AIRBORNE":
                        '''
                        get affordance from learned value
                        '''
                        season_index = planner.available_seasons.index("summer")
                        location = planner.environment_description.getWaypointFromString(action)
                        object = planner.environment_description.getObjectFromString(action)
                        color = planner.target_color

                        if "operation_area" in action.lower():
                            location_list = self._learned_params[season_index].keys()
                            object_list = self._learned_params[season_index][0].keys()
                            color_list = self._learned_params[season_index][0][0].keys()

                            if color != "NOT_SPECIFIED":#if color added as paramas remove this here
                                color_list = [planner.available_detection_colors.index(color)]

                            mean_probabilities = []
                            for location in location_list:
                                for object in object_list:
                                    for color in color_list:
                                        detections = self._learned_params[season_index][location][object][color]['N_Detections_T_F']
                                        mean_probabilities.append(detections)
                            if mean_probabilities:
                                mean_probability = sum(mean_probabilities) / (len(mean_probabilities)*20.0)
                            else:
                                mean_probability = 0
                            aff_scores[index] = mean_probability


                        elif location != None and object != None:
                            location_index = planner.environment_description.getUniqueLocations().index(planner.environment_description.getGeneralLocation(location))
                            object_index = planner.available_detections.index(object)

                            if planner.target_area == "NOT_SPECIFIED":
                                if color != "NOT_SPECIFIED":
                                    color_index = planner.available_detection_colors.index(color)
                                    if color == planner.target_color and object == planner.target_object:
                                        aff_scores[index] = self._learned_params[season_index][location_index][object_index][color_index]['Probability']
                                    else:
                                        aff_scores[index] = 0.0
                                else:
                                    if planner.target_area in location and object == planner.target_object:
                                        prob = 0.0
                                        for index_c, color in enumerate(planner.available_detection_colors):
                                            prob += self._learned_params[season_index][location_index][object_index][index_c]['N_Detections_T_F']
                                        aff_scores[index] = prob / (len(planner.available_detection_colors)*20.0)
                                    else:
                                        aff_scores[index] = 0.0
                            else:
                                if color != "NOT_SPECIFIED":
                                    color_index = planner.available_detection_colors.index(color)
                                    if color == planner.target_color and planner.target_area in location and object == planner.target_object:
                                        aff_scores[index] = self._learned_params[season_index][location_index][object_index][color_index]['Probability']
                                    else:
                                        aff_scores[index] = 0.0
                                else:
                                    if planner.target_area in location and object == planner.target_object:
                                        prob = 0.0
                                        for index_c, color in enumerate(planner.available_detection_colors):
                                            prob += self._learned_params[season_index][location_index][object_index][index_c]['N_Detections_T_F']
                                        aff_scores[index] = prob / (len(planner.available_detection_colors)*20.0)
                                    else:
                                        aff_scores[index] = 0.0
                        '''
                        Bayes Formula P(A|B) = (P(B|A) * P(A)) / P(B)
                        B = (season, location, object, color)
                        P(B) = P(Season) * P(Location) * P(Object) * P(Color)
                            = 0.5 * 0.3333333333333 * 0.5 * 0.1666666
                        '''
                        aff_scores[index] =  aff_scores[index] #(aff_scores[index] * p_detection) / p_conditions
                    else:
                        aff_scores[index] = 0.0
                elif "continue" in action.lower():
                    if platform_status == "AIRBORNE":
                        if len(detected_objects) > 0:
                            aff_scores[index] = 0.0
                            for object in detected_objects:
                                if detected_objects.confidence > 0.9:
                                    aff_scores[index] = 1.0
                                    break
                        else:
                            aff_scores[index] = 1.0
                    else:
                        aff_scores[index] = 0.0
                elif "map" in action.lower():
                    if platform_status == "AIRBORNE":
                        if "map" in planner.goal.lower():
                            #no other goal implemented for affordance
                            aff_scores[index] = 1.0
                    else:
                        aff_scores[index] = 0.0
                elif "take off" in action.lower():
                    if platform_status == "AIRBORNE":
                        aff_scores[index] = 0.0
                    else:
                        aff_scores[index] = 1.0
                elif "land" in action.lower():
                    if platform_status == "AIRBORNE":
                        aff_scores[index] = 1.0 #maybe depends on battery state or else
                    else:
                        aff_scores[index] = 0.0
                elif "hover" in action.lower():
                    if platform_status == "AIRBORNE":
                        aff_scores[index] = 0.0
                    else:
                        aff_scores[index] = 0.0
                elif "confirm" in action.lower():
                    if platform_status == "AIRBORNE":
                        if len(detected_objects) > 0:
                            aff_scores[index] = 0.0
                            for object in detected_objects:
                                if detected_objects.confidence > 0.9:
                                    aff_scores[index] = 1.0
                                    break
                        else:
                            aff_scores[index] = 0.0
                    else:
                        aff_scores[index] = 0.0
                elif "ascend" in action.lower() or "descend" in action.lower():
                    if platform_status == "AIRBORNE":
                        pass
                    else:
                        aff_scores[index] = 0.0
                elif "single image" in action.lower():
                    if platform_status == "AIRBORNE":
                        aff_scores[index] = 0.5
                    else:
                        aff_scores[index] = 0.0
        except Exception as e:
            print(e)
        return aff_scores

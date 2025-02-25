import numpy as np
import math
from ament_index_python.packages import get_package_share_directory
from .aems_utils.AlphaVectorPolicy import *
from .aems_utils.aems_solver import * 
from .aems_utils.fib_solver import *
from .aems_utils.aems_pomdp import * 
from .aems_utils.fa_solver import *
from .aems_utils.graph import * 
from .aems_utils.utils import *
from scipy.stats import rv_discrete
from collections import defaultdict
import csv
import sys
import copy
import os

sys.setrecursionlimit(1500)

class PayModel():
    def __init__(self, planner, operation_areas):
        self._operation_areas = operation_areas    
        
        package_share_directory = get_package_share_directory('auspex_planning')
        workspace_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(package_share_directory))))
        search_weights_path = os.path.join(workspace_dir, "src", 'auspex_planning', 'auspex_planning', "planner", "llm_planner_utils", 'search_weights.csv')
        self._learned_params = self.read_csv_to_nested_dict(search_weights_path)
        self._planner = planner

    def setup_pomdp(self, planner, states, actions):
        """
        Setting up the pomdp
        """
        actions.append("Continue with Skill")
        self._pomdp = AEMS_POMDP(max_iterations=2, root_manager="clear", states=states, actions=actions, obs_param=self._learned_params, planner=planner)
        self._pomdp.lower_bound = setup_fib(self._pomdp, self.alphas_fib)   #solve_fib(self._pomdp)
        self._pomdp.upper_bound = setup_fa(self._pomdp, self.alphas_fa)      # solve_fa(self._pomdp)
        self._solver = AEMS_Solver(self._pomdp, actions)

        self._bn_root = None
        n = len(self._pomdp.states)
        #probs = [0.0] * len(n)
        self._b_init = rv_discrete(values=(list(range(n)), [1/n for _ in range(n)]))
        print("Set up POMDP!")

    def compute_pay_score(self, droneState, actions):
        '''
        One step look ahead with depth max_iterations
        '''
        print("Computing pay scores...")
        pay_scores = np.ones(len(actions))
        location_of_drone = droneState._currentWaypoint
        location_of_searched_object = "woods"

        action, self._bn_root = self._solver.action(self._b_init, self._bn_root, actions)

        for ai, a_ in enumerate(actions):
            if action == a_:
                pay_scores[ai] = 0.51
            else:
                pay_scores[ai] = 0.49/len(pay_scores)

        #pay_scores = pay_scores / np.sum(pay_scores)
        return pay_scores

    def update_pomdp(self, action, observation):
        if self._planner.mode != "SayCan":
            self._bn_root.b = self._pomdp.update_function(self._bn_root.b, action, observation)

    def gps_distance(self, lat1, lon1, lat2, lon2):
        # Convert latitude and longitude from degrees to radians
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

        # Haversine formula
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        r = 6371  # Radius of Earth in kilometers
        return r * c

    def read_csv_to_nested_dict(self, csv_file):
        # Initialize the nested dictionary
        results = defaultdict(lambda: defaultdict(lambda: defaultdict(lambda: defaultdict(dict))))
        
        # Open and read the CSV file
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
        
        return results

    alphas_fa = [[-149.74231124,-149.74231124,-149.74231124,-149.74231124,-149.74231124,
                    -149.74231124,-149.74231124,-149.74231124,-149.74231124,-149.74231124,
                    -149.74231124],
                    [-149.74231124,-149.74231124,-149.74231124,-149.74231124,-149.74231124,
                    -149.74231124,-149.74231124,-149.74231124,-149.74231124,-149.74231124,
                    -149.74231124],
                    [-100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,       ],
                    [-149.74231124,-149.74231124,-149.74231124,-149.74231124,-149.74231124,
                    -149.74231124,-149.74231124,-149.74231124,-149.74231124,-149.74231124,
                    -149.74231124],
                    [-149.74231124,-149.74231124,-149.74231124,-149.74231124,-149.74231124,
                    -149.74231124,-149.74231124,-149.74231124,-149.74231124,-149.74231124,
                    -149.74231124],
                    [-149.74231124,-149.74231124,-149.74231124,-149.74231124,-149.74231124,
                    -149.74231124,-149.74231124,-149.74231124,-149.74231124,-149.74231124,
                    -149.74231124],
                    [-149.74231124,-149.74231124,-149.74231124,-149.74231124,-149.74231124,
                    -149.74231124,-149.74231124,-149.74231124,-149.74231124,-149.74231124,
                    -149.74231124],
                    [-149.74231124,-149.74231124,-149.74231124,-149.74231124,-149.74231124,
                    -149.74231124,-149.74231124,-149.74231124,-149.74231124,-149.74231124,
                    -149.74231124],
                    [-149.74231124,-149.74231124,-149.74231124,-149.74231124,-149.74231124,
                    -149.74231124,-149.74231124,-149.74231124,-149.74231124,-149.74231124,
                    -149.74231124],
                    [-100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,       ],
                    [-100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,       ],
                    [-100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,       ],
                    [-100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,       ],
                    [-149.74231124,-149.74231124,-149.74231124,-149.74231124,-149.74231124,
                    -149.74231124,-149.74231124,-149.74231124,-149.74231124,-149.74231124,
                    -149.74231124],
                    [-100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,       ],
                    [-100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,       ],
                    [-100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,       ],
                    [-100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,       ],
                    [-149.74231124,-149.74231124,-149.74231124,-149.74231124,-149.74231124,
                    -149.74231124,-149.74231124,-149.74231124,-149.74231124,-149.74231124,
                    -149.74231124],
                    [-100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,       ],
                    [-100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,       ],
                    [-100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,       ],
                    [-100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,       ],
                    [-149.74231124,-149.74231124,-149.74231124,-149.74231124,-149.74231124,
                    -149.74231124,-149.74231124,-149.74231124,-149.74231124,-149.74231124,
                    -149.74231124],
                    [-100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,       ],
                    [-100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,       ],
                    [-100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,       ],
                    [-100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,       ],
                    [-149.74231124,-149.74231124,-149.74231124,-149.74231124,-149.74231124,
                    -149.74231124,-149.74231124,-149.74231124,-149.74231124,-149.74231124,
                    -149.74231124],
                    [-100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,       ],
                    [-100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,       ],
                    [-100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,       ],
                    [-100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,       ],
                    [-149.74231124,-149.74231124,-149.74231124,-149.74231124,-149.74231124,
                    -149.74231124,-149.74231124,-149.74231124,-149.74231124,-149.74231124,
                    -149.74231124],
                    [-100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,       ],
                    [-100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,       ],
                    [-100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,       ],
                    [-100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,       ],
                    [-149.74231124,-149.74231124,-149.74231124,-149.74231124,-149.74231124,
                    -149.74231124,-149.74231124,-149.74231124,-149.74231124,-149.74231124,
                    -149.74231124],
                    [-100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,       ],
                    [-100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,       ],
                    [-100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,       ],
                    [-100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,       ],
                    [-149.74231124,-149.74231124,-149.74231124,-149.74231124,-149.74231124,
                    -149.74231124,-149.74231124,-149.74231124,-149.74231124,-149.74231124,
                    -149.74231124],
                    [-100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,       ],
                    [-100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,       ],
                    [-100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,       ],
                    [-100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,       ],
                    [-149.74231124,-149.74231124,-149.74231124,-149.74231124,-149.74231124,
                    -149.74231124,-149.74231124,-149.74231124,-149.74231124,-149.74231124,
                    -149.74231124],
                    [-100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,       ],
                    [-100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,       ],
                    [-100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,       ],
                    [-100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,       ],
                    [-149.74231124,-149.74231124,-149.74231124,-149.74231124,-149.74231124,
                    -149.74231124,-149.74231124,-149.74231124,-149.74231124,-149.74231124,
                    -149.74231124],
                    [-100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,       ],
                    [-100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,       ],
                    [-100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,       ],
                    [-100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,       ],
                    [-149.74231124,-149.74231124,-149.74231124,-149.74231124,-149.74231124,
                    -149.74231124,-149.74231124,-149.74231124,-149.74231124,-149.74231124,
                    -149.74231124],
                    [-100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,       ],
                    [-100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,       ],
                    [-100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,        -100.,        -100.,        -100.,        -100.,
                    -100.,       ],
                    [-149.74231124,-149.74231124,-149.74231124,-149.74231124,-149.74231124,
                    -149.74231124,-149.74231124,-149.74231124,-149.74231124,-149.74231124,
                    -149.74231124],
                    [-149.74231124,-149.74231124,-149.74231124,-149.74231124,-149.74231124,
                    -149.74231124,-149.74231124,-149.74231124,-149.74231124,-149.74231124,
                    -149.74231124],
                    [-149.74231124,-149.74231124,-149.74231124,-149.74231124,-149.74231124,
                    -149.74231124,-149.74231124,-149.74231124,-149.74231124,-149.74231124,
                    -149.74231124],
                    [-149.74231124,-149.74231124,-149.74231124,-149.74231124,-149.74231124,
                    -149.74231124,-149.74231124,-149.74231124,-149.74231124,-149.74231124,
                    -149.74231124],
                    [-149.74231124,-149.74231124,-149.74231124,-149.74231124,-149.74231124,
                    -149.74231124,-149.74231124,-149.74231124,-149.74231124,-149.74231124,
                    -149.74231124],
                    [-149.74231124,-149.74231124,-149.74231124,-149.74231124,-149.74231124,
                    -149.74231124,-149.74231124,-149.74231124,-149.74231124,-149.74231124,
                    -149.74231124]]

    alphas_fib = [[-15.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,-15.],
                    [-15.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,-15.],
                    [-10.,-10.,-10.,-10.,-10.,-10.,-10.,-10.,-10.,-10.,-10.],
                    [-15.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,-15.],
                    [-15.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,-15.],
                    [-15.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,-15.],
                    [-15.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,-15.],
                    [-15.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,-15.],
                    [-15.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,-15.],
                    [-10.,-10.,-10.,-10.,-10.,-10.,-10.,-10.,-10.,-10.,-10.],
                    [-10.,-10.,-10.,-10.,-10.,-10.,-10.,-10.,-10.,-10.,-10.],
                    [-10.,-10.,-10.,-10.,-10.,-10.,-10.,-10.,-10.,-10.,-10.],
                    [  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.],
                    [-15.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,-15.],
                    [  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.],
                    [  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.],
                    [  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.],
                    [  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.],
                    [-15.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,-15.],
                    [  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.],
                    [  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.],
                    [  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.],
                    [  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.],
                    [-15.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,-15.],
                    [  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.],
                    [  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.],
                    [  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.],
                    [  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.],
                    [-15.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,-15.],
                    [  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.],
                    [  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.],
                    [  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.],
                    [  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.],
                    [-15.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,-15.],
                    [  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.],
                    [  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.],
                    [  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.],
                    [  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.],
                    [-15.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,-15.],
                    [  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.],
                    [  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.],
                    [  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.],
                    [  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.],
                    [-15.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,-15.],
                    [  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.],
                    [  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.],
                    [  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.],
                    [  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.],
                    [-15.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,-15.],
                    [  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.],
                    [  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.],
                    [  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.],
                    [  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.],
                    [-15.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,-15.],
                    [  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.],
                    [  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.],
                    [  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.],
                    [-10.,-10.,-10.,-10.,-10.,-10.,-10.,-10.,-10.,-10.,-10.],
                    [-15.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,-15.],
                    [-10.,-10.,-10.,-10.,-10.,-10.,-10.,-10.,-10.,-10.,-10.],
                    [-10.,-10.,-10.,-10.,-10.,-10.,-10.,-10.,-10.,-10.,-10.],
                    [-10.,-10.,-10.,-10.,-10.,-10.,-10.,-10.,-10.,-10.,-10.],
                    [-15.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,-15.],
                    [-15.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,-15.],
                    [-15.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,-15.],
                    [-15.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,-15.],
                    [-15.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,-15.],
                    [-15.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,-15.]]

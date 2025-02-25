import numpy as np
from .graph import * 
from .aems_solver import * 
from scipy.stats import rv_discrete
from .fib_solver import *
from .AlphaVectorPolicy import *
from .fa_solver import *
from .utils import *
import itertools
import time 


class AEMS_POMDP():
    '''
    This class defines the POMPDP
    '''

    def __init__(self, max_iterations=100, max_time=20000000, root_manager="clear", states=[], actions=[], obs_param=None, planner=None):
        self._planner = planner


        self.states = states#[list(p) for p in itertools.product(states, states)] #product from all wps 
        self.actions = actions
        self.observations = ["object detected", "object not detected"]

        self.discount_factor = 0.9
        if obs_param != None:
            self._learned_obs_params = obs_param

        self._object_locaiton = "openareas1"
        self._object_list = self._learned_obs_params[0][0].keys()
        self._color_list = self._learned_obs_params[0][0][0].keys()

        self.r_state= -10
        self.r_action= -5

        
        self.G = Graph(self.discount_factor)
        self.max_iterations = max_iterations
        self.max_time = max_time
        self.max_depth = 2 #only for own value iteration needed

        self.root_manager = root_manager
        self.action_selector = "U" # "L for lower bounds somewhere"

        self.lower_bound = None
        self.upper_bound = None
        

    def transition_function(self, s_1, a, s_2):
        cats = list(range(len(self.states)))
        p_over_states = len(self.states) * [0.0]
        p_prob = 0.0

        wp = self._planner.environment_description.getWaypointFromString(a)
        height = self._planner.environment_description.getHeightFromString(a)

        if wp == None and height == None:# and not("Land" in action):
            p_over_states[s_1] = 1.0
            return rv_discrete(values=(cats, p_over_states))

        if "Search" in a:
            new_state_index = self.states.index(wp)
            p_over_states[new_state_index] = 1.0
            return rv_discrete(values=(cats, p_over_states))
        else:
            p_over_states[s_1] = 1.0
            return rv_discrete(values=(cats, p_over_states))


        if "Take off" in a:
            p_over_states[s_1] = 1.0
            return rv_discrete(values=(cats, p_over_states)) 
        elif "Continue" in a:
            p_over_states[s_1] = 1.0
            return rv_discrete(values=(cats, p_over_states)) 
        elif "Confirm" in a:
            p_over_states[s_1] = 1.0
            return rv_discrete(values=(cats, p_over_states))
        elif "Ascend" in a:
            p_over_states[s_1] = 1.0
            return rv_discrete(values=(cats, p_over_states)) 
        elif "Descend" in a:
            p_over_states[s_1] = 1.0
            return rv_discrete(values=(cats, p_over_states))
        else:
            p_over_states[s_1] = 1.0
            return rv_discrete(values=(cats, p_over_states))

            if "Land" in a:
                p_prob = 0.0 
            elif "Hover" in a:
                p_prob = 0.0 
            elif "Map" in a:#has wp but does not detect
                p_prob = 0.0 
            elif "single image" in a:
                p_prob = 0.0 

        if p_prob == 0.0:
            p_over_states[s_1] = 1.0
            return rv_discrete(values=(cats, p_over_states))

        return rv_discrete(values=(cats, p_over_states))


    def observation_function(self, o, a, s2):
        cats = ["object detected", "object not detected"]
        cats = [0, 1]

        location = self._planner.available_locations.index(self._planner.environment_description.getGeneralLocation(self.states[s2]))

        if "Take off" in a:
            p_object_detected_when_in_state = 0.01
        elif "Land" in a:
            p_object_detected_when_in_state = 0.01
        elif "Continue" in a:
            p_object_detected_when_in_state = 0.01
        elif "Search" in a:
            mean_detections = []     
            for object in self._object_list:
                for color in self._color_list:
                    detections = self._learned_obs_params[0][location][object][color]['N_Detections_T_F']
                    mean_detections.append(detections)
            if mean_detections:
                mean_detections = sum(mean_detections) / (len(mean_detections)*20.0) 
            else:
                mean_detections = 0

            p_object_detected_when_in_state = mean_detections
        elif "Hover" in a:
            p_object_detected_when_in_state = 0.01
        elif "Confirm" in a:
            mean_detections = []               
            for object in self._object_list:
                for color in self._color_list:
                    detections = self._learned_obs_params[0][location][object][color]['N_Detections_T_F']
                    mean_detections.append(detections)
            if mean_detections:
                mean_detections = sum(mean_detections) / (len(mean_detections)*20.0)
            else:
                mean_detections = 0
            p_object_detected_when_in_state = min((mean_detections * 1.02), 0.98)
        elif "Ascend" in a:
            p_object_detected_when_in_state = 0.01
        elif "Descend" in a:
            p_object_detected_when_in_state = 0.01
        elif "Map" in a:
            p_object_detected_when_in_state = 0.01
        elif "single image" in a:
            p_object_detected_when_in_state = 0.01
        return rv_discrete(values=(cats, [p_object_detected_when_in_state, 1-p_object_detected_when_in_state]))



    def reward_function(self, s, a):
        state_strings = self.states[s]
        state_reward = self.r_state
        if "openareas1" in state_strings[1]:
            state_reward = 0.0
            
        action_reward = self.r_action
        if "Search" in a:
            action_reward = 0.0

        return state_reward + action_reward
    
    '''
    ---END---
    '''
    def update_function(self, b, a, oi):
        """
        Update the discrete belief based on action `a` and observation `o`.

        Parameters:
        - b: DiscreteBelief object, the current belief state
        - a: action
        - o: observation

        Returns:
        - Updated DiscreteBelief object
        """
        state_space = self.states
        bp = np.zeros(len(state_space))
        for si, s in enumerate(state_space):
            b_sampled = pmf(b, si)
            #print(f"Sampled prob {b_sampled} for state {s} and action is {a} and obs = {o}")
            if b_sampled > 0.0:
                td = self.transition_function(si, a, si)
                for spi, tp in weighted_iterator(td):
                    #op = obs_weight(s, a, sp, o)  # Observation probability
                    op = pmf(self.observation_function(oi, a, spi), oi)
                    # print(f"oi = {oi} , a = {a}, spi {spi}")
                    # print(f"op = {op} , tp = {tp}, b_s {b_sampled}")
                    # print("--  ")
                    bp[spi] += op * tp * b_sampled

        bp_sum = np.sum(bp)

        if bp_sum == 0.0:
            raise ValueError(f"""Failed discrete belief update: new probabilities sum to zero""")

        # Normalize
        bp /= bp_sum
        states = list(range(len(self.states)))
        probabilities = list(bp)
        #print(f"probs: {bp}")
        updated_belief = rv_discrete(values=(states, probabilities))

        return updated_belief


    def update_root(self, a, oi):
        # Check if the root manager is set to user, otherwise raise an error
        if self.root_manager != "user":
            raise ValueError("User is trying to update root but root_manager != 'user'.")

        # Get the action and observation indices
        ai = self.actions.index(a)
        oi = oi

        # Get the original root node from the graph
        original_root = self.G.get_root()

        # Calculate the index of the action node (an)
        an_ind = original_root.children.start + ai - 1
        an = get_an(self.G, an_ind)

        # Calculate the index of the new root based on observation
        new_root_ind = an.children.start + oi - 1
        self.G.root_ind = new_root_ind

    def value_function(self, solver, b, depth=0):
            """
            The value function. Called recursively till dself.max_depth is reached
            """
            if depth == self.max_depth:
                return np.array([0.0,0.0])
            max_reward = -float('inf')
            min_reward = float('inf')
            for a in self.actions:
                sum_obs =  np.array([0.0,0.0])

                for oi, o in enumerate(self.observations):
                    #P_o_given_b_a = self.probability_function(b,a,oi)
                    P_o_given_b_a = solver.O(b, a, oi)
                    b_next = self.update_function(b, a, oi)
                    v_next =  self.value_function(solver, b_next, depth+1)
                    sum_obs += P_o_given_b_a * v_next 

                exptected_reward = self.reward_function(b, a)
                exptected_reward = [exptected_reward, exptected_reward] + self.discount_factor * sum_obs
                max_reward = max(exptected_reward[0], max_reward)
                min_reward = min(exptected_reward[1], min_reward)

            return np.array([max_reward, min_reward])



    

    # def e_circ(self, b, depth):
    #     b_value = self.value_function(b)
    #     ub = max(b_value)
    #     lb = min(b_value)

    #     error = ub - lb

    #     return error,ub, lb

    # def ExpectedError(self, b, depth):
    #     gamma = pow(self.discount_factor, depth)

    #     return gamma * self.p_reach_fringe_state(b, depth) * self.e_circ(b, depth)
# max_iterations = 30
#     iteration = 0
#     depth=1
#     max_error = -float('inf')

#     while iteration != max_iterations:
#         max_index = -1
        
#         backtrack

#         b_current = np.ones(len(states))  # array mit belief values for fringe states

#         for index, leaf_node in enumerate(b_leafs):
#             E_expected = ExpectedError(b_init, depth)
#             if E_expected > max_error:
#                 max_error = E_expected
#                 max_index = index
#         iteration += 1
#         depth+=1


    # def p_obersation(self, b, a, o) -> float: # P(o|b,a)
    #     p = 0.0
    #     for  s_2 in self.states:
    #         p_current = self.observation_function(o, a ,s_2)
    #         t = []
    #         for s in self.states: 
    #             t += self.transition_function(s, a, s_2) * self.belief_state[s]
    #     # return 0.0



    # def update_function(self, b, a, o):
    #     total_prob = 0.0  # For normalization
    #     b_2 = {}  # Updated belief will be stored here as a dictionary for clarity

    #     # Step 1: Compute the unnormalized belief update for each state s_2
    #     for spi, s_2 in enumerate(self.states):
    #         belief_sum = 0.0   
    #         # Sum over all possible previous states s_1
    #         for si, s_1 in enumerate(self.states):
    #             # Get the transition distribution T(s'|s, a) and belief b(s)
    #             transition_dist = self.transition_function(si, a, spi)  # Returns a distribution over s_2
    #             transition_prob = pmf(transition_dist,spi)  
                
    #             belief_prob = pmf(b,si)  # Belief distribution b(s_1)
    #             belief_sum += transition_prob * belief_prob
            
    #         # Multiply by observation probability O(o | s', a)
    #         obs_dis= self.observation_function(o, a, spi)
    #         obs_prob = pmf(transition_dist,spi) 
    #         b_2[s_2] = obs_prob * belief_sum  # Unnormalized belief
    #         # Accumulate total probability for normalization
    #         total_prob += b_2[s_2]
        
    #     # Step 2: Normalize the updated belief b_2
    #     if total_prob > 0:
    #         for s_2 in b_2:
    #             b_2[s_2] /= total_prob  # Normalize each state probability
        
    #     # Step 3: Convert the updated belief into a distribution
    #     states = np.arange(0,2)
    #     probabilities = list(b_2.values())
        
    #     # Creating a new probability distribution for the updated belief
    #     updated_belief = rv_discrete(values=(states, probabilities))

    #     #print(updated_belief.__dict__)
        
    #     return updated_belief

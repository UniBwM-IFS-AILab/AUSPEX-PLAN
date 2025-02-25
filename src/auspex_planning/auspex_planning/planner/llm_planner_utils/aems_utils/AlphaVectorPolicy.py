import numpy as np
from .utils import *


import numpy as np

class AlphaVectorPolicy:
    def __init__(self, pomdp, alphas, action_map):
        """
        Initialize AlphaVectorPolicy with a POMDP, alpha vectors, and an action map.

        Args:
        - pomdp: The POMDP problem instance.
        - alphas: List of alpha vectors.
        - action_map: List of actions corresponding to each alpha vector.
        """
        self.pomdp = pomdp
        self.n_states = len(pomdp.states)  # Number of states in POMDP
        self.alphas = [np.array(alpha) for alpha in alphas]  # List of alpha vectors
        self.action_map = list(action_map)  # List of corresponding actions

    @staticmethod
    def from_matrix(self,alpha_matrix, action_map):
        """
        Construct an AlphaVectorPolicy from an alpha matrix.

        Args:
        - pomdp: The POMDP problem instance.
        - alpha_matrix: |S| x (number of alpha vecs) matrix.
        - action_map: List of actions corresponding to each alpha vector.
        """
        num_actions = alpha_matrix.shape[1]
        alpha_vectors = [alpha_matrix[:, i] for i in range(num_actions)]
        return AlphaVectorPolicy(self.pomdp, alpha_vectors, action_map)

    def alpha_pairs(self):
        """
        Return an iterator of alpha vector-action pairs in the policy.
        """
        return zip(self.alphas, self.action_map)

    def alpha_vectors(self):
        """
        Return the list of alpha vectors.
        """
        return self.alphas


    def action(self, b):
        """
        Get the best action for the belief `b`.

        Args:
        - b: Belief distribution over states.

        Returns:
        - Best action corresponding to the alpha vector with the highest value for the belief `b`.
        """
        bvec = self.belief_vector(b)
        best_idx = np.argmax([np.dot(bvec, alpha) for alpha in self.alphas])
        return self.action_map[best_idx]

    def action_values(self, b):
        """
        Compute the value of each action for the belief `b`.

        Args:
        - b: Belief distribution over states.

        Returns:
        - max_values: Maximum values of each action for the belief `b`.
        """
        bvec = self.belief_vector(b)
        max_values = -np.inf * np.ones(len(self.pomdp.actions))
        for i, alpha in enumerate(self.alphas):
            temp_value = np.dot(bvec, alpha)
            action_idx = self.pomdp.actions.index(self.action_map[i])
            max_values[action_idx] = max(max_values[action_idx], temp_value)
        return max_values

    def value(self, b):
        """
        Compute the value of the belief `b` under this policy.
        
        Args:
        - b: Belief distribution over states.

        Returns:
        - max_value: Maximum value obtained by any alpha vector for belief `b`.
        """
        bvec = self.belief_vector(b)
        rtrn_value =  max(np.dot(bvec, alpha) for alpha in self.alphas)
        return rtrn_value

    def belief_vector(self, b):
        """
        Return a vector representation of the belief `b`.

        Args:
        - b: Belief distribution.

        Returns:
        - Vector-like representation of the belief `b`.
        """
        bvec = np.zeros(len(self.pomdp.states))
        for si, s in enumerate(self.pomdp.states):
            prob = pmf(b,si)
            bvec[si] = prob
        return bvec

    def push(self, alpha, a):
        """
        Add an alpha vector and corresponding action to the policy.

        Args:
        - alpha: Alpha vector to add.
        - a: Corresponding action.
        """
        self.alphas.append(np.array(alpha))
        self.action_map.append(a)




# class AlphaVectorPolicy:
#     def __init__(self, pomdp, alphas, actions):
#         self.pomdp = pomdp
#         self.alphas = alphas
#         self.actions = actions

#     def action(self, b):
#         state_list = self.pomdp.states
#         alphas = self.alphas
#         actions = self.actions

#         best_action = actions[0]
#         best_value = -float('inf')

#         for ai, a in enumerate(actions):
#             value = 0.0
#             for si, s in enumerate(state_list):
#                 value += pmf(b, si) * alphas[si, ai]

#             if value > best_value:
#                 best_value = value
#                 best_action = a

#         return best_action

#     def value(self, b):
#         state_list = self.pomdp.states
#         alphas = self.alphas
#         actions = self.actions

#         best_value = -float('inf')
#         for ai, a in enumerate(actions):
#             value = 0.0
#             for si, s in enumerate(state_list):
#                 value += pmf(b, si) * alphas[si, ai]

#             best_value = max(best_value, value)

#         return best_value

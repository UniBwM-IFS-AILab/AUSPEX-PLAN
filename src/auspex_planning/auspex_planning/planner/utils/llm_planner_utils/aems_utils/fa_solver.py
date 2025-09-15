import numpy as np
from .AlphaVectorPolicy import * 
from .utils import *

class BlindPolicy:
    def __init__(self, action, value):
        self.a = action  # Best action
        self.v = value   # Value of the policy

def create_blind_policy(pomdp):
    """
    Create a BlindPolicy for the given POMDP.

    :param pomdp: The POMDP object which contains actions, states, rewards, and discount factor.
    :return: A BlindPolicy instance.
    """
    action_list = pomdp.actions        # List of possible actions
    state_list = pomdp.states  # List of possible states

    best_action = action_list[0]
    best_worst_reward = float('-inf')

    for a in action_list:
        worst_reward = float('inf')
        for si, s in enumerate(state_list):
            r_sa = pomdp.reward_function(si, a)
            worst_reward = min(r_sa, worst_reward)

        if worst_reward > best_worst_reward:
            best_worst_reward = worst_reward
            best_action = a

    discount_factor = pomdp.discount_factor
    v = best_worst_reward / (1.0 - discount_factor)

    return BlindPolicy(best_action, v)

def action(policy, belief):
    """
    Get the action from the BlindPolicy.

    :param policy: An instance of BlindPolicy.
    :param belief: The belief state (not used in this implementation).
    :return: The action from the policy.
    """
    return policy.a

def value(policy, belief):
    """
    Get the value of the BlindPolicy.

    :param policy: An instance of BlindPolicy.
    :param belief: The belief state (not used in this implementation).
    :return: The value of the policy.
    """
    return policy.v

class FixedActionSolver:
    def __init__(self, max_iterations=100, tolerance=1e-3):
        self.max_iterations = max_iterations
        self.tolerance = tolerance

def setup_fa(pomdp, alphas):
    return AlphaVectorPolicy(pomdp, alphas, pomdp.actions)

def solve_fa(pomdp, bounds="u"):
    """
    Solve the POMDP problem using FixedActionSolver.

    :param solver: An instance of FixedActionSolver.
    :param pomdp: The POMDP object.
    :return: An AlphaVectorPolicy instance.
    """
    tolerance=1e-3
    # Convenience variables
    state_list = pomdp.states
    action_list = pomdp.actions
    ns = len(state_list)
    na = len(action_list)

    # Fill with max_worst_value
    bp = create_blind_policy(pomdp)
    max_worst_value = bp.v
    alphas = np.full((na, ns), max_worst_value)
    old_alphas = np.full((na, ns), max_worst_value)

    for _ in range(pomdp.max_iterations):
        np.copyto(old_alphas, alphas)
        residual = 0.0

        for ai, a in enumerate(action_list):
            for si, s in enumerate(state_list):
                sp_dist = pomdp.transition_function(si, a, si)
                sp_sum = 0.0
                for spi, sp in enumerate(state_list):
                    sp_sum += pmf(sp_dist, spi) * old_alphas[ai, spi]
                r = pomdp.reward_function(si, a)
                alphas[ai, si] = r + pomdp.discount_factor * sp_sum

                alpha_diff = abs(alphas[ai, si] - old_alphas[ai, si])
                #if bounds == "u":
                residual = max(alpha_diff, residual)
                # else:
                #     residual = max(alpha_diff, residual)

        if residual < tolerance:
            break
    print(alphas)
    return AlphaVectorPolicy(pomdp, alphas, action_list)
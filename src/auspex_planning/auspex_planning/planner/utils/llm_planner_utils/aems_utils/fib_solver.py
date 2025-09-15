import numpy as np
from .AlphaVectorPolicy import * 
from .utils import *
from scipy.stats import norm, rv_discrete

class FIBSolver:
    def __init__(self, max_iterations=100, tolerance=1e-3, verbose=False):
        self.max_iterations = max_iterations
        self.tolerance = tolerance
        self.verbose = verbose

def setup_fib(pomdp, alphas):
    return AlphaVectorPolicy(pomdp, alphas, pomdp.actions)

def solve_fib(pomdp):
    """
    Solve the POMDP problem using FIBSolver.

    :param solver: An instance of FIBSolver.
    :param pomdp: The POMDP object.
    :param kwargs: Additional keyword arguments (deprecated).
    :return: An AlphaVectorPolicy instance.
    """
    tolerance=1e-3

    state_list = pomdp.states
    obs_list = pomdp.observations
    action_list = pomdp.actions

    ns = len(state_list)
    na = len(action_list)

    alphas = np.zeros((na, ns))
    old_alphas = np.zeros((na, ns))

    for i in range(pomdp.max_iterations):
        # Copy old_alphas to alphas
        np.copyto(old_alphas, alphas)

        residual = 0.0

        for ai, a in enumerate(action_list):
            for si, s in enumerate(state_list):
                sp_dist = pomdp.transition_function(si, a, si)

                r = 0.0
                for spi, p_sp in weighted_iterator(sp_dist):
                    r += p_sp * pomdp.reward_function(si, a)

                # Sum_o max_a' Sum_s' O(o | s',a) T(s'|s,a) alpha_a^k(s')
                o_sum = 0.0
                for oi, o in enumerate(obs_list):
                    # Take maximum over ap
                    ap_sum = float('-inf')
                    for api, ap in enumerate(action_list):
                        # Sum_s' O(o | s',a) T(s'|s,a) alpha_a^k(s')
                        temp_ap_sum = 0.0
                        for spi, p_sp in weighted_iterator(sp_dist):
                            o_dist = pomdp.observation_function(oi, a, spi)
                            p_o = pmf(o_dist, oi)

                            temp_ap_sum += p_o * p_sp * old_alphas[api, spi]
                        ap_sum = max(temp_ap_sum, ap_sum)

                    o_sum += ap_sum

                alphas[ai, si] = r + pomdp.discount_factor * o_sum

                alpha_diff = abs(alphas[ai, si] - old_alphas[ai, si])
                residual = max(alpha_diff, residual)

        if residual < tolerance:
            break
    print(alphas)
    return AlphaVectorPolicy(pomdp, alphas, action_list)


import time 
from .graph import * 
from .fib_solver import *
from .AlphaVectorPolicy import *
from .fa_solver import *
from .utils import *


class AEMS_Solver():

    def __init__(self, pomdp, actions):
        self.current_actions = actions
        self.pomdp = pomdp


    def action(self, b, bn_root, actions):
        self.current_actions = actions
        t_start = time.time()
        best_bn = 0

        if bn_root == None:
            bn_root = self.determine_root_node(b)
        else:
            self.pomdp.G.clear_graph()
            bn_root = BeliefNode.root(bn_root.b, bn_root.L, bn_root.U)
            self.pomdp.G.add_belief_node(bn_root)


        for i in range(self.pomdp.max_iterations):
            #print(f"Iteration: {i}")
            # Determine the node to expand and its pre-expansion bounds
            best_bn = self.select_node(bn_root)

            Lold, Uold = best_bn.L, best_bn.U
            
            best_bn = self.expand(best_bn)
            best_bn = self.backtrack(best_bn, Lold, Uold)
            
            # Stop if the timeout has been reached
            if (time.time() - t_start) > self.pomdp.max_time:
                print("Out of time...")
                break
        
        # Return the best action
        best_action =  self.get_best_action(bn_root)
        return best_action, bn_root

    def get_action_index(self, action_shrinked):
        for ai, a in enumerate(self.pomdp.actions):
            if a == action_shrinked:
                return ai

        print("ERROR: Look up error. Action not in actions...")
        return -1

    # Allows user to select action node with best upper or lower bound
    def get_best_action(self, bn_root):
        best_ai = get_an(self.pomdp.G, bn_root.aind).ai  # Upper bound desired by default
        # if self.pomdp.solver.action_selector == 'L':  # If lower bound is desired
        #     best_L = float('-inf')
        #     for an_idx in bn_root.children:
        #         an = get_an(pomdp, an_idx)
        #         if an.L > best_L:
        #             best_L = an.L
        #             best_ai = an.ai
        return self.pomdp.actions[best_ai]

    # Recursively selects best fringe node for expansion
    def select_node(self, bn):
        if self.pomdp.G.is_fringe(bn):
            return bn
        
        # Select next action node
        an = self.pomdp.G.action_nodes[bn.aind]
        
        # Iterate over child belief nodes of action node, selecting best one
        best_val = float('-inf')
        best_bn = bn
        for bn_idx in an.children:
            child_bn = self.select_node(self.pomdp.G.belief_nodes[bn_idx])
            bv = child_bn.poc * (child_bn.U - child_bn.L) * child_bn.gd
            if bv >= best_val:
                best_val = bv
                best_bn = child_bn
        return best_bn

    # Expands a belief node by creating action and belief nodes from all possible actions and observations
    def expand(self, bn):
        b = bn.b

        La_max = float('-inf') 
        Ua_max = float('-inf')  # Max Ua over actions
        an_start = self.pomdp.G.na
        ai_max = an_start  # Index of maximum upper bound (for AEMS2)

        for ai, a in enumerate(self.current_actions): #self.pomdp.actions):
            an_ind = self.pomdp.G.na 
            bn_start = self.pomdp.G.nb 

            # Bounds and reward for action node
            La = Ua = r = self.R(b, a)
            #print(r)

            for oi, o in enumerate(self.pomdp.observations):
                po = self.O(b, a, oi)  # Probability of observation o
                bp = self.pomdp.update_function(b, a, oi)  # Update belief
                # Determine bounds at new belief
                if self.pomdp.lower_bound != None:
                    L = self.pomdp.lower_bound.value(bp)
                    U = self.pomdp.upper_bound.value(bp)
                else:
                    U, L = self.pomdp.value_function(self, b)
                
                La += self.pomdp.discount_factor * po * L
                Ua += self.pomdp.discount_factor * po * U

                # Create belief node and add to graph
                poc = bn.poc * bn.po
                bpn = BeliefNode.create(bp, self.pomdp.G.nb, an_ind, oi, po, poc, L, U, bn.d + 1, bn.gd * self.pomdp.discount_factor)
                self.pomdp.G.add_belief_node(bpn)

            # Update
            if Ua > Ua_max:
                Ua_max = Ua
                ai_max = an_ind  # AEMS2
            if La > La_max:
                La_max = La
            

            # Create action node and add to graph
            b_range = range(bn_start, self.pomdp.G.nb)  # Indices of children belief nodes
           
            an = ActionNode(r, bn.ind, self.get_action_index(a) , La, Ua, b_range)  #modified ai
            self.pomdp.G.add_action_node(an)


            #print(f"Ua {Ua}, Ua_max {Ua_max} , action {self.pomdp.G.action_nodes[ai_max].ai}")

        # Child nodes of bn are the action nodes we've opened
        bn.children = range(an_start, self.pomdp.G.na)
        bn.L = La_max
        bn.U = Ua_max
        bn.aind = ai_max  # AEMS2
        return bn

    # Backtracks after expanding a belief node
    def backtrack(self, bn, Lold, Uold):
        while not self.pomdp.G.is_root(bn):
            an = self.pomdp.G.parent_beliefs_node(bn)
            an.L += self.pomdp.discount_factor * bn.po * (bn.L - Lold)
            an.U += self.pomdp.discount_factor * bn.po * (bn.U - Uold)
            bn = self.pomdp.G.parent_actions_node(an)
            Lold, Uold = self.update_node(bn)
        return bn

    # Updates L and U given children, used for backtracking
    def update_node(self, bn):
        L_old, U_old = bn.L, bn.U
        ai_max = bn.children[0]
        U_max = L_max = float('-inf')
        
        for ai in bn.children:
            an = self.pomdp.G.action_nodes[ai]
            if an.U > U_max:
                U_max = an.U
                ai_max = ai  # AEMS2
            if an.L > L_max:
                L_max = an.L
        
        bn.L = L_max
        bn.U = U_max
        bn.aind = ai_max  # AEMS2
        return L_old, U_old



    def O(self, b, a, oi):
        """
        Compute the probability of observing `o` after taking action `a` from belief `b`.
        """
        sum_sp = 0.0

        for spi, sp in enumerate(self.pomdp.states):
            od = self.pomdp.observation_function(oi, a, spi)  # Get the observation distribution given action `a` and resulting state `sp`
            po = pmf(od, oi)                 # Probability of observation `o` from the observation distribution `od`
            
            if po == 0:
                continue  # Skip the rest of the loop if the observation probability is zero
            
            sum_s = 0.0

            for si, s in enumerate(self.pomdp.states):
                spd = self.pomdp.transition_function(si, a, spi)  # Get the transition distribution from state `s` given action `a`
                sum_s += pmf(spd, spi) * pmf(b, si)  # Probability of transitioning to state `sp` and being in state `s`
            
            sum_sp += sum_s * po  # Accumulate the weighted sum over possible next states `sp`
        
        return sum_sp

    def R(self, b, a):
        """
        Compute the expected reward given a belief state, an action, and a POMDP model.
        """
        total_reward = 0.0
        
        for si, state in enumerate(self.pomdp.states):
            # Compute the reward for taking `action` in `state`
            reward_for_state = self.pomdp.reward_function(si, a)
            
            # Probability of being in `state` according to the belief
            prob_of_state = pmf(b, si)
            
            # Accumulate the expected reward
            total_reward += reward_for_state * prob_of_state
        
        return total_reward


    def determine_root_node(self, b):
        """
        Determine or create the root node of the graph based on the given belief and pomdp configuration.
        
        :param pomdp: The AEMSpomdp object, which manages the graph and configuration.
        :param belief: The belief state to check or use for the root node.
        :return: The root node of the graph.
        """
        # If the root manager is set to 'clear', clear the graph
        if self.pomdp.root_manager == 'clear':
            self.pomdp.G.clear_graph()

        # If the graph has no belief nodes, create and add the root node
        if len(self.pomdp.G.belief_nodes) == 0:

            if self.pomdp.lower_bound != None:
                L = self.pomdp.lower_bound.value(b)
                U = self.pomdp.upper_bound.value(b)
            else:
                U, L = self.pomdp.value_function(self, b)

            #U, L = self.pomdp.value_function(self, b)

            bn_root = BeliefNode.root(b, L, U)
            self.pomdp.G.add_belief_node(bn_root)
            return bn_root

        # If the root manager is set to 'belief', check existing nodes
        if self.pomdp.root_manager == 'belief':
            bn_root = self.pomdp.G.get_root()

            # If the existing root node has the same belief, return it
            if bn_root.b == b:
                return bn_root

            # Search through children nodes to see if any have the same belief
            for an_idx in bn_root.children:
                an = get_an(self.pomdp.G, an_idx)
                for bn_idx in an.children:
                    bn = get_bn(self.pomdp.G, bn_idx)
                    if bn.b == b:
                        self.pomdp.G.root_ind = bn.ind
                        return self.pomdp.G.get_root()

            # If no matching belief is found, raise an error
            raise ValueError("None of the child beliefs match input")

        # Default case: return the existing root node if no specific root manager is set
        return self.pomdp.G.get_root()

    
# Define ActionNode class
class ActionNode:
    def __init__(self, r, pind, ai, L, U, children):
        self.r = r              # reward for taking this action
        self.pind = pind        # index of parent belief node in Graph.belief_nodes
        self.ai = ai            # which action this corresponds to
        self.L = L              # lower bound
        self.U = U              # upper bound
        self.children = children  # range of child belief nodes

# Define BeliefNode class
class BeliefNode:
    def __init__(self, b, ind, pind, aind, oi, po, poc, L, U, d, gd, children):
        self.b = b              # belief associated with this node
        self.ind = ind          # index of this node in Graph.belief_nodes
        self.pind = pind        # index of parent action node in Graph.action_nodes
        self.aind = aind        # index of best child action node in action_nodes
        self.oi = oi            # index of observation corresponding with this belief
        self.po = po            # probability of seeing that observation
        self.poc = poc          # prob of all observations leading here (cumulative)
        self.L = L              # lower bound
        self.U = U              # upper bound
        self.d = d              # depth
        self.gd = gd            # discount^depth, stored to improve computation
        self.children = children  # range of child nodes

    # Constructor for root node
    @classmethod
    def root(self, b, L, U):
        return BeliefNode(b, 0, 0, 0, 0, 1.0, 1.0, L, U, 0, 1.0, range(0,0))

    # Constructor for non-root belief node
    @classmethod
    def create(self, b, ind, pind, oi, po, poc, L, U, d, gd):
        return BeliefNode(b, ind, pind, 0, oi, po, poc, L, U, d, gd, range(0,0))

# Define Graph class
class Graph:
    def __init__(self, df):
        self.action_nodes = []  # list of ActionNode
        self.belief_nodes = []  # list of BeliefNode
        self.na = 0             # number of action nodes
        self.nb = 0             # number of belief nodes
        self.root_ind = 0       # index of the root node
        self.df = df            # discount factor

    # Clear graph function
    def clear_graph(self):
        self.action_nodes = []
        self.belief_nodes = []
        self.na = 0
        self.nb = 0
        self.root_ind = 0

    # Check if node is root
    def is_root(self, bn):
        return self.root_ind == bn.ind

    # Get root node
    def get_root(self):
        return self.belief_nodes[self.root_ind]  # Python is zero-indexed

    # Check if node is a fringe node (no children)
    def is_fringe(self, bn):
        return len(bn.children) == 0

    # Add action node to graph
    def add_action_node(self, an):
        self.action_nodes.append(an)
        self.na += 1

    # Add belief node to graph
    def add_belief_node(self, bn):
        self.belief_nodes.append(bn)
        self.nb += 1

    # Get parent node for a belief node (returns ActionNode)
    def parent_beliefs_node(self, bn):
        return self.action_nodes[bn.pind]  # Python is zero-indexed

    # Get parent node for an action node (returns BeliefNode)
    def parent_actions_node(self, an):
        return self.belief_nodes[an.pind]  # Python is zero-indexed

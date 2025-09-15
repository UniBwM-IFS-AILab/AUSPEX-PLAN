import time 
import numpy as np
from aems_utils.AlphaVectorPolicy import *
from aems_utils.aems_solver import * 
from aems_utils.fib_solver import *
from aems_utils.aems_pomdp import * 
from aems_utils.fa_solver import *
from aems_utils.graph import * 
from aems_utils.utils import *
from scipy.stats import rv_discrete

# import matplotlib.pyplot as plt
import sys
sys.setrecursionlimit(1500)


def main():
    pomdp = AEMS_POMDP(max_iterations=50, root_manager="clear")
    pomdp.lower_bound = solve_fib(pomdp)
    pomdp.upper_bound = solve_fa(pomdp)
    solver = AEMS_Solver(pomdp)

    bn_root = None
    n = len(pomdp.states)

    b_init = rv_discrete(values=(list(range(n)), [1/n for _ in range(n)]))
    print("Initialized...")

    for a in range(0,5):
        action, bn_root = solver.action(b_init, bn_root)
        print(f"Vals: {bn_root.b.xk} probs: {bn_root.b.pk}")
        print(f"The next action is: {action}")


        current_obs = 0.0
        bn_root.b = pomdp.update_function(bn_root.b, action, current_obs)


    
if __name__ == "__main__":
    main()



# values, probabilities = b_init.xk, b_init.pk

# # Plot
# plt.bar(values, probabilities)
# plt.xlabel('Values')
# plt.ylabel('Probabilities')
# plt.title('Custom Discrete Distribution')
# plt.show()

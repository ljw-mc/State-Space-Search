import trying
import bidirectional_search
import numpy as np


from collections import deque
import numpy as np
from search_problems import Node, GraphSearchProblem, get_random_grid_problem
from a_star_search import a_star_search

#### Must be Commented Away Later #####
from breadth_first_search import breadth_first_search
import matplotlib.pyplot as plt

def failure(failure_count):
    N = 150
    for p_occ in failure_count.keys():
        for i in range(100):
            problem = get_random_grid_problem(p_occ, N, N)

            # Solve it
            path, num_nodes_expanded, max_frontier_size = a_star_search(problem)
            failure_count[p_occ] += num_nodes_expanded
    return failure_count

if __name__ == '__main__':
    # Test your code here!
    # Create a random instance of GridSearchProblem






    p_occ = 0.05
    failure_count = dict()
    for i in range(20):
        failure_count[p_occ] = 0
        p_occ += 0.05
        
    failure_count = failure(failure_count)
    p_occ = failure_count.keys()
    failures = []
    for x in p_occ:
        failures.append((x, failure_count[x]))

    plt.title("Failures vs p_occ")
    plt.plot(failures)
    plt.show()



        # Check the result
        # correct = problem.check_solution(path)
        # print(path)
        # print("Solution is correct: {:}".format(correct))
        # Plot the result
        # problem.plot_solution(path)
                
                
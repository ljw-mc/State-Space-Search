import queue
import numpy as np
from search_problems import Node, GridSearchProblem, get_random_grid_problem

def a_star_search(problem):
    """
    Uses the A* algorithm to solve an instance of GridSearchProblem. Use the methods of GridSearchProblem along with
    structures and functions from the allowed imports (see above) to implement A*.

    :param problem: an instance of GridSearchProblem to solve
    :return: path: a list of states (ints) describing the path from problem.init_state to problem.goal_state[0]
             num_nodes_expanded: number of nodes expanded by your search
             max_frontier_size: maximum frontier size during search
    """

    num_nodes_expanded = 0
    max_frontier_size = 0

    # Check for Trivial Solution
    if (problem.init_state == problem.goal_states[0]):
        return [problem.init_state], num_nodes_expanded, max_frontier_size
    

    # Start of Actual Algorithm - Based on Uniform-Cost-Search
    init_state = problem.init_state
    goal_state = problem.goal_states[0]
    
    frontier = queue.PriorityQueue()
    path = dict() # frontier_states[state] = path_cost
    path_cost = dict()
    explored = set() # set of states

    frontier.put((problem.heuristic(init_state), init_state))
    path[init_state] = [init_state]
    path_cost[init_state] = problem.heuristic(init_state)


    while True:
        if (frontier.empty()):
            return [], num_nodes_expanded, max_frontier_size
        
        frontier_size = frontier.qsize()
        if frontier_size > max_frontier_size:
            max_frontier_size = frontier_size
        
        (priority, state) = frontier.get()

        if problem.goal_test(state):
            return path[state], num_nodes_expanded, max_frontier_size
        num_nodes_expanded += 1
        explored.add(state)

        for action in problem.get_actions(state):
            if (action[1] not in explored) and ((action[1] not in path_cost.keys()) or (path_cost[action[1]] > problem.heuristic(action[1]) + (len(path[state])))):
                path_cost[action[1]] = problem.heuristic(action[1]) + (len(path[state]))
                child_path = (path[state]).copy()
                child_path.append(action[1])
                path[action[1]] = child_path
                frontier.put((problem.heuristic(action[1]) + (len(path[state])), action[1]))


def search_phase_transition():
    """
    Simply fill in the prob. of occupancy values for the 'phase transition' and peak nodes expanded within 0.05. You do
    NOT need to submit your code that determines the values here: that should be computed on your own machine. Simply
    fill in the values!

    :return: tuple containing (transition_start_probability, transition_end_probability, peak_probability)
    """
    ####
    #   REPLACE THESE VALUES
    ####
    transition_start_probability = 0.35
    transition_end_probability = 0.4
    peak_nodes_expanded_probability = 0.4
    return transition_start_probability, transition_end_probability, peak_nodes_expanded_probability

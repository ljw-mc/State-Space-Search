from collections import deque
import numpy as np
from search_problems import Node, GraphSearchProblem


def is_empty(list):
    return len(list) == 0

class MyNode:
    def __init__(self, path, state, next_states):
        self.path = path
        self.state = state
        self.next_states = next_states
    

def breadth_first_search(problem):
    """
    Implement a simple breadth-first search algorithm that takes instances of SimpleSearchProblem (or its derived
    classes) and provides a valid and optimal path from the initial state to the goal state. Useful for testing your
    bidirectional and A* search algorithms.

    :param problem: instance of SimpleSearchProblem
    :return: path: a list of states (ints) describing the path from problem.init_state to problem.goal_state[0]
             num_nodes_expanded: number of nodes expanded by your search
             max_frontier_size: maximum frontier size during search
    """
    ####
    #   COMPLETE THIS CODE
    ####'

    max_frontier_size = 0
    num_nodes_expanded = 0


    init_state = problem.init_state
    goal_state = problem.goal_states[0]
    

    node = MyNode([init_state], init_state, problem.neighbours[init_state])

    if (node.state == goal_state):
        return node.path, num_nodes_expanded, max_frontier_size
    
    frontier = [node]
    explored = []

    while True:
        if (is_empty(frontier)):
            return [], num_nodes_expanded, max_frontier_size
        
        frontier_size = len(frontier)
        if frontier_size > max_frontier_size:
            max_frontier_size = frontier_size


        node = frontier.pop(0)
        explored.append(node.state)
        num_nodes_expanded += 1

        for child_state in node.next_states:
            cur_path = node.path.copy()
            cur_path.append(child_state)

            child = MyNode(cur_path, child_state, problem.neighbours[child_state])
            if (child.state not in explored) or (child not in frontier):
                if (child.state == goal_state):
                    return child.path, num_nodes_expanded, max_frontier_size
                frontier.append(child)

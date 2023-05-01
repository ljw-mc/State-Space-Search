



from collections import deque
import numpy as np
from search_problems import Node, GraphSearchProblem

#### Must be Commented Away Later #####
from breadth_first_search import breadth_first_search

# doesn't like the speed of my bidirectional_search -- 
# probably because I am using BFS twice, which is rather slow 
# might want me to do IDDFS and find optimal solution faster

class MyNode:
    def __init__(self, path, state, next_states):
        self.path = path
        self.state = state
        self.next_states = next_states



def is_empty(list1, list2):
    return (len(list1), len(list2)) == (0,0)


def bidirectional_search(problem):
    """
        Implement a bidirectional search algorithm that takes instances of SimpleSearchProblem (or its derived
        classes) and provides a valid and optimal path from the initial state to the goal state.

        :param problem: instance of SimpleSearchProblem
        :return: path: a list of states (ints) describing the path from problem.init_state to problem.goal_state[0]
                 num_nodes_expanded: number of nodes expanded by your search
                 max_frontier_size: maximum frontier size during search
        """
    ####
    #   COMPLETE THIS CODE
    ####
    max_frontier_size = 0
    num_nodes_expanded = 0

    init_state = problem.init_state
    goal_state = problem.goal_states[0]

    if (init_state == goal_state):
        return [init_state], num_nodes_expanded, max_frontier_size
    
    fnode = MyNode(deque([init_state]), init_state, problem.neighbours[init_state])
    bnode = MyNode(deque([goal_state]), goal_state, problem.neighbours[goal_state])

    f_frontier = deque([fnode])
    f_explored_states = set() #for explored states
    f_explored = deque([]) #for explored nodes

    b_frontier = deque([bnode])
    b_explored_states = set() #for explored states
    b_explored = deque([]) #for explored nodes

    while True:
        if is_empty(f_frontier, b_frontier):
            return [], num_nodes_expanded, max_frontier_size
        
        frontier_size = len(f_frontier) + len(b_frontier)
        if frontier_size > max_frontier_size:
            max_frontier_size = frontier_size
        
        fnode = f_frontier.popleft()
        f_explored_states.add(fnode.state)
        num_nodes_expanded += 1

        for child_state in problem.neighbours[fnode]:
            cur_path = fnode.path.copy()
            cur_path.append(child_state)

            child = MyNode(cur_path, child_state, problem.neighbours[child_state])
            if (child_state not in f_explored_states) or (child not in f_frontier):
                if (child.state in b_explored_states):
                    for bnode in b_explored:
                        if bnode.state == child.state:
                            bpath = bnode.path.copy()
                            bpath.reverse()
                            bpath.popleft()
                            return child.path + bpath, num_nodes_expanded, max_frontier_size
            
                f_frontier.append(child)
        
        bnode = b_frontier.popleft()
        b_explored_states.add(bnode.state)
        b_explored.append(bnode)
        num_nodes_expanded += 1

        for child_state in bnode.next_states:
            cur_path = bnode.path.copy()
            cur_path.append(child_state)

            child = MyNode(cur_path, child_state, problem.neighbours[child_state])
            if (child_state not in b_explored) or (child not in b_frontier):
                if (child.state in f_explored_states):
                    for fnode in f_explored:
                        if fnode.state == child.state:
                            cpath = child.path
                            cpath.reverse()
                            cpath.popleft()
                            return fnode.path + cpath, num_nodes_expanded, max_frontier_size
                b_frontier.append(child)




if __name__ == '__main__':
    # Simple example
    goal_states = [0]
    init_state = 9
    V = np.arange(0, 10)
    E = np.array([[0, 1],
                  [1, 2],
                  [2, 3],
                  [3, 4],
                  [4, 5],
                  [5, 6],
                  [6, 7],
                  [7, 8],
                  [8, 9],
                  [0, 6],
                  [1, 7],
                  [2, 5],
                  [9, 4]])
    problem = GraphSearchProblem(goal_states, init_state, V, E)
    path, num_nodes_expanded, max_frontier_size = breadth_first_search(problem)
    correct = problem.check_graph_solution(path)
    print("Solution is correct: {:}".format(correct))
    print(path)

    # Use stanford_large_network_facebook_combined.txt to make your own test instances
    E = np.loadtxt('/Users/lincolnwen/Documents/ROB311/rob311_winter_2023_project_01_handout/stanford_large_network_facebook_combined.txt', dtype=int)
    V = np.unique(E)
    goal_states = [683]
    init_state = 2
    problem = GraphSearchProblem(goal_states, init_state, V, E)
    path, num_nodes_expanded, max_frontier_size = bidirectional_search(problem)
    correct = problem.check_graph_solution(path)
    print("Solution is correct: {:}".format(correct))
    print(path)

    # Be sure to compare with breadth_first_search!
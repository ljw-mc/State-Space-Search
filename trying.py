from collections import deque
import numpy as np
from search_problems import Node, GraphSearchProblem

#### Must be Commented Away Later #####
from breadth_first_search import breadth_first_search

# doesn't like the speed of my bidirectional_search -- 
# probably because I am using BFS twice, which is rather slow 
# might want me to do IDDFS and find optimal solution faster

def bfs(problem, queue, visited, parent, dest_visited, next_len):
    node = queue.pop(0)
    children_nodes = problem.neighbours[node]

    for child in children_nodes:
        if visited[child] == False:
            queue.append(child)
            visited[child] = True
            parent[child] = node
            next_len += 1
        
        if dest_visited[child] == True:
            return queue, visited, parent, child, next_len

    return queue, visited, parent, False, next_len

def intersection(src_visited, dest_visited):
    front = src_visited.keys()
    back = dest_visited.keys()

    for key in front:
        for node in back:
            if key == node:
                return key
    
    return False

def path_finder(intersecting_node, src, dest, src_parent, dest_parent):
    path = []
    path.append(intersecting_node)

    node = intersecting_node
    while node != src:
        path.append(src_parent[node])
        node = src_parent[node]
    
    path = path[::-1]
    node = intersecting_node

    while node != dest:
        path.append(dest_parent[node])
        node = dest_parent[node]
    
    return path

    

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

    V = problem.V
    src = problem.init_state
    dest = problem.goal_states[0]
    
    if (src == dest):
        return [src], 0, 0
    
    src_visited = dict(); 
    dest_visited = dict();
    for (node) in V:
        src_visited[int(node)] = False
        dest_visited[int(node)] = False


    src_queue = [src]
    src_visited[src] = True
    src_parent = dict(); src_parent[src] = -1

    dest_queue = [dest]
    dest_visited[dest] = True
    dest_parent = dict(); dest_parent[dest] = -1

    next_len_forward = 1
    next_len_backward = 1
    tracker = set() ; path = []
    while src_queue and dest_queue:
        for i in range(next_len_forward):
            src_queue, src_visited, src_parent, intersecting_node, next_len_forward = bfs(problem, src_queue, src_visited, src_parent, dest_visited, next_len_forward)
            if intersecting_node != False:
                tracker.add(intersecting_node)


        for i in range(next_len_backward):
            dest_queue, dest_visited, dest_parent, intersecting_node, next_len_backward = bfs(problem, dest_queue, dest_visited, dest_parent, src_visited, next_len_backward)
            if intersecting_node != False and intersecting_node not in tracker:
                tracker.add(intersecting_node)
            
        if len(tracker) != 0:
            for intersecting_node in tracker:
                path.append(path_finder(intersecting_node, src, dest, src_parent, dest_parent))
            
            return path[0], 0, 0
        # intersecting_node = intersection(src_visited, dest_visited)



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

    # Use stanford_large_network_facebook_combined.txt to make your own test instances
    E = np.loadtxt('/Users/lincolnwen/Documents/ROB311/rob311_winter_2023_project_01_handout/stanford_large_network_facebook_combined.txt', dtype=int)
    V = np.unique(E)
    goal_states = [683]
    init_state = 5
    problem = GraphSearchProblem(goal_states, init_state, V, E)
    # path, num_nodes_expanded, max_frontier_size = breadth_first_search(problem)
    # correct = problem.check_graph_solution(path)
    # print("Solution is correct: {:}".format(correct))
    # print(path)
    path, num_nodes_expanded, max_frontier_size = bidirectional_search(problem)
    correct = problem.check_graph_solution(path)
    print("Solution is correct: {:}".format(correct))
    print(path)

    # Be sure to compare with breadth_first_search!
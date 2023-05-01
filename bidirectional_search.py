from collections import deque
import numpy as np
from search_problems import Node, GraphSearchProblem


def is_empty(list1):
    return len(list1) == (0)

def bfs(problem, queue, visited, parent, dest_visited, next_len):
    node = queue.popleft()
    children_nodes = problem.neighbours[node]
    next_len -= 1

    for child in children_nodes:
        if visited[child] == False:
            queue.append(child)
            visited[child] = True
            parent[child] = node
            next_len += 1
        
        if dest_visited[child] == True:
            return queue, visited, parent, child, next_len

    return queue, visited, parent, False, next_len


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
    
    src_visited = dict()
    dest_visited = dict()
    for (node) in V:
        src_visited[int(node)] = False
        dest_visited[int(node)] = False


    src_queue = deque([src])
    src_visited[src] = True
    src_parent = dict(); src_parent[src] = -1

    dest_queue = deque([dest])
    dest_visited[dest] = True
    dest_parent = dict(); dest_parent[dest] = -1

    next_len_forward = 1 # to keep track of length of queue at all times
    next_len_backward = 1 # to keep track of length of queue at all times
    tracker = list() ; paths = []; lengths = dict()

    while src_queue and dest_queue:
        if is_empty(src_queue) and is_empty(dest_queue):
            return [], 0, 0
        

        for i in range(next_len_forward):
            src_queue, src_visited, src_parent, intersecting_node, next_len_forward = bfs(problem, src_queue, src_visited, src_parent, dest_visited, next_len_forward)
            if intersecting_node != False:
                tracker.append(intersecting_node)


        for i in range(next_len_backward):
            dest_queue, dest_visited, dest_parent, intersecting_node, next_len_backward = bfs(problem, dest_queue, dest_visited, dest_parent, src_visited, next_len_backward)
            if intersecting_node != False and intersecting_node not in tracker:
                tracker.append(intersecting_node)
            
        # this is to find all the paths where an intersection was found after forward bfs and backwards bfs
        if len(tracker) != 0:
            for i in range(len(tracker)):
                paths.append(path_finder(tracker[i], src, dest, src_parent, dest_parent))
                lengths[len(paths[i])] = paths[i]
            
            min_key = min(lengths.keys())
            return lengths[min_key], 0, 0

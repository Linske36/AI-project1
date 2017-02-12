#!/usr/bin/python

import sys
import queue
import time
import resource

class Node:
    def __init__(self, state, parent, direction, cost, depth):
        self.state = state
        self.parent = parent
        self.direction = direction
        self.cost = cost
        self.depth = depth


def create_Node(state, parent, direction, cost, depth):
    return Node(state, parent, direction, cost, depth)



def create_Frontier(node_state):
    state = node_state.state
#    print("create state", state)
    top_row = [state[0], state[1], state[2]]
    mid_row = [state[3], state[4], state[5]]
    btm_row = [state[6], state[7], state[8]]

    first_colm = [state[0], state[3], state[6]]
    last_colm = [state[2], state[5], state[8]]

    pos = state.index('0')
    frontier_Nodes = []
    new_State = list(state)

    
    #Move up
    if '0' not in top_row:
        temp = state[pos - 3]
        new_State[pos - 3] = state[pos]
        new_State[pos] = temp
        new_Node = create_Node(new_State, node_state, "Up", 1, node_state.depth + 1)
        frontier_Nodes.append(new_Node)
   #     print("up", new_State)
        new_State = list(state)

    #Move down
    if '0' not in btm_row:
        temp = state[pos + 3]
        new_State[pos + 3] = state[pos]
        new_State[pos] = temp
        new_Node = create_Node(new_State, node_state, "Down", 1, node_state.depth + 1)
        frontier_Nodes.append(new_Node)
  #      print("down", new_State)
        new_State = list(state)

    #Move Left
    if '0' not in first_colm:
        temp = state[pos - 1]
        new_State[pos - 1] = state[pos]
        new_State[pos] = temp
        new_Node = create_Node(new_State, node_state, "Left", 1, node_state.depth + 1)
        frontier_Nodes.append(new_Node)
 #       print("left", new_State)
        new_State = list(state)

    #Move Right
    if '0' not in last_colm:
        temp = state[pos + 1]
        new_State[pos + 1] = state[pos]
        new_State[pos] = temp
        new_Node = create_Node(new_State, node_state, "Right", 1, node_state.depth + 1)
        frontier_Nodes.append(new_Node) 
 #       print("right", new_State)
    
    return frontier_Nodes

   



def bfs(initial_State, goal_Test):
    start = time.time()
    frontier = []
    explored = set()
    begin = 1
    nodes_expanded = 0
    max_depth = 0
    init_Node = create_Node(initial_State, None, 0, 0, 0)
    frontier.append(init_Node)
    max_fringe_size = 0

    while frontier:
 
        if len(frontier) > max_fringe_size:
            max_fringe_size = len(frontier)
 
        state = frontier.pop(0)
        test_State = list(map(int, state.state))
        explored_State = ''.join(map(str, test_State))
        explored.add(explored_State)



        if test_State == goal_Test:
            stop = time.time()
            running_time = stop - start
            fringe_size = len(frontier)
            path = []
            cost = 0
            depth = 0
            nodes_expanded = len(explored) - 1
            last_node = frontier.pop()
            
            while state and state.parent != None:
                path.insert(0, state.direction)
                cost += state.cost
                depth += 1
                state = state.parent
               
            max_depth = last_node.depth

            ram = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1000
            
            output_file(path, cost, nodes_expanded, fringe_size, max_fringe_size, depth, max_depth, running_time, ram)


            return

        neighbor = create_Frontier(state)
   	
        if not frontier and begin == 1:
            for index in range(len(neighbor)):
                frontier.append(neighbor[index])
            begin = 0
  
        else:
            test_Frontier = set()
            for index in range(len(frontier)):
                test_frontier_State = ''.join(map(str, frontier[index].state))
                test_Frontier.add(test_frontier_State)

            for index in range(len(neighbor)):
                neighbor_State = ''.join(map(str, neighbor[index].state))
                
                if neighbor_State not in test_Frontier and neighbor_State not in explored:
                    test_Frontier.add(neighbor_State)
                    frontier.append(neighbor[index])
                
      


def dfs(initial_State, goal_Test):

    start = time.time()
    frontier = []
    explored = set()
    test_Frontier = set()
    begin = 1
    nodes_expanded = 0
    max_depth = 0
    init_Node = create_Node(initial_State, None, 0, 0, 0)
    frontier.append(init_Node)
    max_fringe_size = 0

    while frontier:
 
        if len(frontier) > max_fringe_size:
            max_fringe_size = len(frontier)
        
         	
        state = frontier.pop()
        test_State = list(map(int, state.state))
        explored_State = ''.join(map(str, state.state))
        explored.add(explored_State)

        if not begin:
            test_Frontier.remove(explored_State)
 
        if max_depth < state.depth:
            max_depth = state.depth

        if test_State == goal_Test:
            stop = time.time()
            running_time = stop - start
            fringe_size = len(frontier)
            path = []
            cost = 0
            depth = 0
            nodes_expanded = len(explored) - 1
            last_node = frontier.pop()
            
            while state and state.parent != None:
                path.insert(0, state.direction)
                cost += state.cost
                depth += 1
                state = state.parent
               

            ram = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1000
            
            output_file(path, cost, nodes_expanded, fringe_size, max_fringe_size, depth, max_depth, running_time, ram)


            return

        neighbor = create_Frontier(state)
        neighbor.reverse()  
        
        if not frontier and begin == 1:
            for index in range(len(neighbor)):
                frontier_State = ''.join(map(str, neighbor[index].state))    
                frontier.append(neighbor[index])
                test_Frontier.add(frontier_State)
            begin = 0
  
        else:
            
            for index in range(len(neighbor)):
                neighbor_State = ''.join(map(str, neighbor[index].state))
                
                if neighbor_State not in test_Frontier and neighbor_State not in explored:
                    test_Frontier.add(neighbor_State)
                    frontier.append(neighbor[index])
      

# def ast(initial_State, goal_Test):





def output_file(path, cost, expanded, fringe, max_fringe, depth, max_depth, time, ram):
    output = open("output.txt", "w")

    output.write("path_to_goal: " + str(path) + '\n' + "cost_of_path: " +
	    str(cost) + '\n' + "nodes_expanded: " + str(expanded) + '\n' +
	    "fringe_size: " + str(fringe) + '\n' + "max_fringe_size: " +
	    str(max_fringe) + '\n' + "search_depth: " + str(depth) + '\n' +
	    "max_search_depth: " + str(max_depth) + '\n' + "running_time: "
	    + str(time) + '\n' + "max_ram_usage: " + str(ram) + '\n')

def main():
    initial_State = sys.argv[2]
    initial_State = initial_State.split(',')
    goal_Test = [0, 1, 2, 3, 4, 5, 6, 7, 8]
    bfs(initial_State, goal_Test)


if __name__ == "__main__":
    main()

import util
from sudoku import SudokuSearchProblem
from maps import MapSearchProblem

################ Node structure to use for the search algorithm ################
class Node:
    def __init__(self, state, action, path_cost, parent_node, depth):
        self.state = state
        self.action = action
        self.path_cost = path_cost
        self.parent_node = parent_node
        self.depth = depth

########################## DFS for Sudoku ########################
## Choose some node to expand from the frontier with Stack like implementation
def sudokuDepthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.
    Return the final values dictionary, i.e. the values dictionary which is the goal state  
    """

    def convertStateToHash(values):
        """ 
        values as a dictionary is not hashable and hence cannot be used directly in the explored/visited set.
        This function changes values dict into a unique hashable string which can be used in the explored set.
        You may or may not use this
        """
        l = list(sorted(values.items()))
        modl = [a+b for (a, b) in l]
        return ''.join(modl)

    ## YOUR CODE HERE
    root_node = Node(problem.getStartState(), [], 0, None, 0)
    frontier = util.Stack()
    frontier.push(root_node)
    explored = []

    while not(frontier.isEmpty()):
        node_to_explore = frontier.pop()

        if problem.isGoalState(node_to_explore.state):
            return node_to_explore.state
        else:
            copy_state = node_to_explore.state.copy()
            
            if convertStateToHash(copy_state) not in explored:
	            explored.append(convertStateToHash(copy_state))
	            successors_state = problem.getSuccessors(copy_state)
	            if len(successors_state) > 0:
		            for state_action_cost in successors_state:
		                if convertStateToHash(state_action_cost[0]) in explored:
		                    continue
		                else:
		                    frontier.push(Node(state_action_cost[0], state_action_cost[1], node_to_explore.path_cost + 1, node_to_explore, node_to_explore.depth + 1))

    return False
    # util.raiseNotDefined()

######################## A-Star and DFS for Map Problem ########################
## Choose some node to expand from the frontier with priority_queue like implementation

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def heuristic(state, problem):
    # It would take a while for Flat Earther's to get accustomed to this paradigm
    # but hang in there.

    """
        Takes the state and the problem as input and returns the heuristic for the state
        Returns a real number(Float)
    """
    node1 = problem.G.node[state]
    node2 = problem.G.node[problem.end_node]
    xy1 = ((node1['x'],0,0), (node1['y'],0,0))
    xy2 = ((node2['x'],0,0), (node2['y'],0,0))
    return util.points2distance(xy1, xy2)
    # util.raiseNotDefined()

def AStar_search(problem, heuristic=nullHeuristic):

    """
        Search the node that has the lowest combined cost and heuristic first.
        Return the route as a list of nodes(Int) iterated through starting from the first to the final.
    """
    root_node = Node(problem.getStartState(), [], 0, None, 0)
    frontier = util.PriorityQueue()
    frontier.push(root_node, 0 + heuristic(root_node.state, problem))
    explored = []
    nodes_list = list(problem.G.nodes)
    # heuristic_value = [heuristic(x,problem) for x in nodes_list]
    # print problem.start_node
    # print problem.end_node
    while not(frontier.isEmpty()):

    	node_to_explore = frontier.pop()
    	if problem.isGoalState(node_to_explore.state):
    		# recontruct_path
    		# print "Some thing"
    		# print node_to_explore.parent_node.state
    		route_list = [node_to_explore.state]
    		parent = node_to_explore.parent_node
    		while parent != None:
    			route_list.append(parent.state)
    			parent = parent.parent_node
    		route_list.reverse()
    		# print route_list
    		return route_list
    	else:
			copy_state = node_to_explore.state
			# successors_state = problem.getSuccessors(copy_state)
			if copy_state not in explored:
				explored.append(copy_state)
				successors_state = problem.getSuccessors(copy_state)
				if len(successors_state) > 0:
					for suc_act_cost in successors_state:
						if suc_act_cost[0] in explored:
							continue
						else:
							cost = node_to_explore.path_cost + suc_act_cost[2]
							cost_with_heuristic = cost + heuristic(suc_act_cost[0], problem)
							frontier.update(Node(suc_act_cost[0], suc_act_cost[1], cost , node_to_explore, node_to_explore.depth + 1), cost_with_heuristic)
		    # util.raiseNotDefined()
    # print len(explored)
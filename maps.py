import networkx as nx
import osmnx as ox
import util
import itertools

class MapSearchProblem:
    """
    This class outlines the structure of a search problem
    """

    def  __init__(self, G, start_node, end_node):
        self.G = G
        self.start_node = start_node
        self.end_node = end_node
        self.nodes_expanded = 0

    def getStartState(self):
        """
        Returns the start state for the search problem which will be of type Int
        """
        # print self.G.neighbors(self.start_node)
        return self.start_node
        # util.raiseNotDefined()

    def isGoalState(self, node):
        """
        node: Search state of type Int
        Returns True if node is the goal state otherwise False
        """
        # d = node - self.end_node
        # given_node = self.G.node[node]
        # end_node = self.G.node[self.end_node]
        # distance = ((given_node['x'] - end_node['x'])**2 + (given_node['y'] - end_node['y'])**2)**0.5
        # print distance
        if node == self.end_node:
            return True
        else:
            return False
        # util.raiseNotDefined()

    def getSuccessors(self, node):
        """
        node: Search state of type Int

        For a given state, this should return a list of triples, (successor(Int),
        action(Edge ID of type Int), stepCost(Float)), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        ## Maintain for bookkeeping purposes    
        self.nodes_expanded += 1 
        ## Dont overuse this function since we calculate the nodes expanded using this
        # print self.G.get_edge_data(5270986781,65327144)
        # [self.G.node[5270986781]][self.G.node[65327144]]
        successors = []
        neighbor_nodes = self.G.neighbors(node)
        for n in neighbor_nodes:
            # print node
            # print n
            edge_data = self.G.get_edge_data(node,n)
            # edge_id = edge_data[0]['osmid']
            new_successor = [n, edge_data[0]['osmid'], edge_data[0]['length']]
            successors.append(new_successor)
        # print neighbor_nodes
        ## YOUR CODE HERE
        # util.raiseNotDefined()

        return successors

if __name__ == "__main__":

    pass
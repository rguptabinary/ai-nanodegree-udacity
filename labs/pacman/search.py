# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

class Node:
    def __init__(self, state, action, cost, parent):
        self.state = state
        self.action = action
        self.cost = cost
        self.parent = parent
        

def get_actions(node):
    actions = []
    a = node
    while a.parent:
        actions.append(a.action)
        a = a.parent
    return list(reversed(actions))
    
def graph_search(problem, frontier):
    n = problem.getStartState()
    root = Node(n, None, 0, None)
    frontier.push(root)
    explored = set()
    while frontier:
        node = frontier.pop()
        
        if problem.isGoalState(node.state):
            return get_actions(node)

        for c in problem.getSuccessors(node.state):
            child = Node(c[0], c[1], c[2], node)
            if child.state not in explored and child not in frontier.list:
                frontier.push(child)
        explored.add(node.state)
    return None

def best_fit_graph_search(problem):
    n = problem.getStartState()
    root = Node(n, [], 0, 0)
    frontier = util.PriorityQueue()
    frontier.push(root, 0)
    explored = set()
    while frontier:
        node = frontier.pop()
        actions = get_actions(node)
        if problem.isGoalState(node.state):
            return actions
        
        for c in problem.getSuccessors(node.state):
            child = Node(c[0], c[1], problem.getCostOfActions(actions), node)
            if child.state not in explored:
                frontier.update(child, child.cost)
        
        explored.add(node.state)
    return None

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    frontier = util.Queue()
    return graph_search(problem, frontier)

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    frontier = util.Stack()
    return graph_search(problem, frontier)

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    return best_fit_graph_search(problem)

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    frontier = util.PriorityQueue()
    explored = set()

    node = Node(problem.getStartState(), [], 0, None)
    frontier.push(node, 0)
    while frontier:
        node = frontier.pop()
        actions = node.action
        if problem.isGoalState(node.state):
            return actions
        for c in problem.getSuccessors(node.state):
            if c[0] not in explored:
                child = Node(c[0], actions + [c[1]], c[2], node)
                frontier.update(child, child.cost + heuristic(child.state, problem))
        explored.add(node.state)
    return None

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch

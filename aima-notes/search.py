from utils import *
from symbols import *

class Problem(object):
    def __init__(self, initial_state, goal=None):
        self.initial = initial_state
        # A goal can be a single state or a list of state
        self.goal = goal

    def actions(self, state):
        raise NotImplementedError

    def result(self, state, action):
        raise NotImplementedError

    def path_cost(self, cost1, state1, action, state2):
        return cost1 + 1

    def goal_test(self, state):
        if isinstance(self.goal, list):
            return is_in(state, self.goal)
        else:
            return state is self.goal

    def value(self, state):
        raise NotImplementedError

class Node:
    def __init__(self, state, parent=None, action=None, path_cost=0):
        self.state = state
        self.parent = parent
        if parent:
            self.depth = parent.depth + 1
        else:
            self.depth = 0
        self.action = action
        self.path_cost = path_cost

    def __repr__(self):
        return "<Node {}>".format(self.state)

    def __lt__(self, node):
        return isinstance(node, Node) and self.state < node.state

    def __eq__(self, node):
        return isinstance(node, Node) and self.state == node.state

    def __hash__(self):
        return hash(self.state)

    def child_node(self, problem, action):
        next = problem.result(self.state, action)
        return Node(self.state, self, action,
                    problem.path_cost(
                        self.path_cost, self.state, action, next.state))

    def expand(self, problem):
        return [self.child_node(problem, action) 
                        for action in problem.actions(self.state)]

    def solution(self):
        # solution is a list of actions to be taken
        return [node.action for node in self.path()[1:]]

    def path(self):
        a=[]
        node=self
        while node:
            a.append(node)
            node=node.parent
        return reversed(a)

class GraphProblem(Problem):
    
    def __init__(self, initial, goal, graph):
        Problem.__init__(self, initial, goal)
        self.graph = graph
    
    def actions(self, A):
        return list(self.graph.get(A).keys())
    
    def result(self, state, action):
        return action

    def path_cost(self, c, A, action, B):
        return c + (self.graph.get(A, B) or infinity)

    def find_min_edge(self):
        m = infinity
        raise NotImplementedError

    def h(self, node):
        raise NotImplementedError


    

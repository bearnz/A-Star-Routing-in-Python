from search import *
from math import *
from copy import deepcopy
from heapq import *
import math
 
def isInt(x):
    try:
        int(x)
        return True
    except ValueError:
        return False
         
     
class RoutingGraph(Graph):
    def __init__(self, graph):
        self.graph_str = graph.split('\n')
        for line in range(len(self.graph_str)):
            self.graph_str[line] = self.graph_str[line].lstrip()
        if self.graph_str[-1] == '':
            self.graph_str = self.graph_str[:-1]
        self.n_rows = len(self.graph_str)
        self.n_cols = len(self.graph_str[0])
        self.obstacles = []
        self.Nagents = []
        self.Sagents = []
        self.goals = []
        self.Fcells = []
        self.est_costs = []
        for row in range(self.n_rows-1):
            for col in range(self.n_cols-1):
                if self.graph_str[row][col] == 'X':
                    self.obstacles.append((row, col))
                elif isInt(self.graph_str[row][col]):
                    self.Nagents.append((row, col, int(self.graph_str[row][col])))
                elif self.graph_str[row][col] == 'S':
                    self.Sagents.append((row, col, float('inf')))
                elif self.graph_str[row][col] == 'G':
                    self.goals.append((row, col))
                elif self.graph_str[row][col] == 'F':
                    self.Fcells.append((row, col))
        self._starting_nodes = deepcopy(self.Nagents)
        for agent in self.Sagents:
            self._starting_nodes.append(agent)
     
    def is_goal(self, agent):
        position = (agent[0], agent[1])
        if position in self.goals:
            return True
        else:
            return False
     
    def starting_nodes(self):
        return self._starting_nodes
     
    def outgoing_arcs(self, agent):
        directions = [('N',-1,0), ('E',0,1), ('S',1,0), ('W',0,-1)]
        arcs = []
        if agent[-1] > 0:
            for direction in directions:
                row = agent[0] + direction[1]
                col = agent[1] + direction[2]
                fuel = agent[2] - 1
                if (row, col) not in self.obstacles:
                    if row > 0 and row < (self.n_rows - 1):
                        if col > 0 and col < (self.n_cols - 1):
                            arc = Arc(agent, (row, col, fuel), direction[0], 5)
                            arcs.append(arc)
        if (agent[0], agent[1]) in self.Fcells:
            if agent[2] < 9:
                arc = Arc(agent, (agent[0], agent[1], 9), 'Fuel up', 15)
                arcs.append(arc)
        return arcs
     
    def estimated_cost_to_goal(self, agent):
        est_costs = []
        for goal in self.goals:
            y_cost = abs(goal[0] - agent[0])
            x_cost = abs(goal[1] - agent[1])
            est_costs.append(5* (y_cost + x_cost))
        return min(est_costs)
         
class AStarFrontier():
    def __init__(self, RoutingGraph):
        self.container = []
        self.graph = RoutingGraph
        self.path_number = 0
        self.visited_nodes = []
        self.cost = 0
     
    def add(self, path):
        if path[-1] not in self.visited_nodes:
            self.visited_nodes.append(path[-1])
            heuristic = self.graph.estimated_cost_to_goal(path[-1].head)
            total_cost = 0
            self.path_number += 1
            for arc in path:
                total_cost += arc.cost
            heappush(self.container, (total_cost+heuristic, self.path_number, path))
        return self.container
     
    def __iter__(self):
        return self
     
    def __next__(self):
        if len(self.container) > 0:
            least_cost = heappop(self.container)[-1]
            return least_cost
        else:
            raise StopIteration
        return
     
class print_map():
    def __init__(self, RoutingGraph, AStarFrontier, solution):
        self.visited_nodes = AStarFrontier.visited_nodes
        self.graph_str = AStarFrontier.graph.graph_str
        self.path = solution
        self.agents = AStarFrontier.graph.Sagents
        self.goals = AStarFrontier.graph.goals
        self.solution = solution
        for node in self.visited_nodes:
            if node.tail not in self.agents and node.tail is not None and (node.tail[0], node.tail[1]) not in self.goals:
                front_lines = self.graph_str[:node.tail[0]]
                line = list(self.graph_str[node.tail[0]])
                if node.tail[0] < AStarFrontier.graph.n_rows-2:
                    back_lines = self.graph_str[node.tail[0]+1:]
                else:
                    back_lines = [self.graph_str[-1]]
                new_line = ["".join(line[:node.tail[1]] + ['.'] + line[node.tail[1]+1:])]
                self.graph_str = front_lines + new_line + back_lines
        if self.solution is not None:
            for node in self.solution:
                if node.head not in self.agents and (node.head[0], node.head[1]) not in self.goals:
                    front_lines = self.graph_str[:node.head[0]]
                    line = list(self.graph_str[node.head[0]])
                    if node.head[0] < AStarFrontier.graph.n_rows-2:
                        back_lines = self.graph_str[node.head[0]+1:]
                    else:
                        back_lines = [self.graph_str[-1]]
                    new_line = ["".join(line[:node.head[1]] + ['*'] + line[node.head[1]+1:])]
                    self.graph_str = front_lines + new_line + back_lines        
        for line in self.graph_str:
            print(line)
            
        
 
def main():
    
    map_str = """\
    +-------------+
    | G         G |
    |      S      |
    | G         G |
    +-------------+
    """
    
    map_graph = RoutingGraph(map_str)
    frontier = AStarFrontier(map_graph)
    solution = next(generic_search(map_graph, frontier), None)
    print_map(map_graph, frontier, solution)


if __name__ == "__main__":
    main()
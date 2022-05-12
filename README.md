# Dijkstra's Shortest Path Algorithm
## AIM

To develop a code to find the shortest route from the source to the destination point using Dijkstra's shortest path algorithm.

## THEORY
To implement Best-First_Search ( BFS ) algorithm to find the route between an initial state to a final state. Something like google maps. We create a dictionary to act as the dataset for the search alogrithm, containing all the distances between all the nodes ( Places ).



## DESIGN STEPS

### STEP 1:
Identify a location in the google map:

### STEP 2:
Select a specific number of nodes with distance

### STEP 3:
Create a dictionary with all the node pairs (keys) and their respective distances as the values

### STEP 4:
Implement the search algorithm by passing any node and f(node) to find the Best route.

### STEP 5:
Display the route sequence.


## ROUTE MAP
#### Include your own map
#### Example map
![166117310-3cf0ff9c-1f7d-4278-a344-3e454584b5f0](https://user-images.githubusercontent.com/75236145/167675024-458dd585-a5d5-47cf-9e19-a1a186011ef9.png)


## PROGRAM
```python
Student name : SURYA R
Reg.no : 212220230052
```
```python
%matplotlib inline
import matplotlib.pyplot as plt
import random
import math
import sys
from collections import defaultdict, deque, Counter
from itertools import combinations
import heapq

class Problem(object):
    """The abstract class for a formal problem. A new domain subclasses this,
    overriding `actions` and `results`, and perhaps other methods.
    The default heuristic is 0 and the default action cost is 1 for all states.
    When yiou create an instance of a subclass, specify `initial`, and `goal` states 
    (or give an `is_goal` method) and perhaps other keyword args for the subclass."""

    def __init__(self, initial=None, goal=None, **kwds): 
        self.__dict__.update(initial=initial, goal=goal, **kwds) 
        
    def actions(self, state):        
        raise NotImplementedError
    def result(self, state, action): 
        raise NotImplementedError
    def is_goal(self, state):        
        return state == self.goal
    def action_cost(self, s, a, s1): 
        return 1
    
    def __str__(self):
        return '{0}({1}, {2})'.format(
            type(self).__name__, self.initial, self.goal)
            
    class Node:
    "A Node in a search tree."
    def __init__(self, state, parent=None, action=None, path_cost=0):
        self.__dict__.update(state=state, parent=parent, action=action, path_cost=path_cost)

    def __str__(self): 
        return '<{0}>'.format(self.state)
    def __len__(self): 
        return 0 if self.parent is None else (1 + len(self.parent))
    def __lt__(self, other): 
        return self.path_cost < other.path_cost
        
    
    failure = Node('failure', path_cost=math.inf) # Indicates an algorithm couldn't find a solution.
    cutoff  = Node('cutoff',  path_cost=math.inf) # Indicates iterative deepening search was cut off.
    
    def expand(problem, node):
    "Expand a node, generating the children nodes."
    s = node.state
    for action in problem.actions(s):
        s1 = problem.result(s, action)
        cost = node.path_cost + problem.action_cost(s, action, s1)
        yield Node(s1, node, action, cost)
        

def path_actions(node):
    "The sequence of actions to get to this node."
    if node.parent is None:
        return []  
    return path_actions(node.parent) + [node.action]


def path_states(node):
    "The sequence of states to get to this node."
    if node in (cutoff, failure, None): 
        return []
    return path_states(node.parent) + [node.state]
    
    
class PriorityQueue:
    """A queue in which the item with minimum f(item) is always popped first."""

    def __init__(self, items=(), key=lambda x: x): 
        self.key = key
        self.items = [] # a heap of (score, item) pairs
        for item in items:
            self.add(item)
         
    def add(self, item):
        """Add item to the queuez."""
        pair = (self.key(item), item)
        heapq.heappush(self.items, pair)

    def pop(self):
        """Pop and return the item with min f(item) value."""
        return heapq.heappop(self.items)[1]
    
    def top(self): return self.items[0][1]

    def __len__(self): return len(self.items)
    
    def best_first_search(problem, f):
    "Search nodes with minimum f(node) value first."
    node = Node(problem.initial)
    frontier = PriorityQueue([node], key=f)
    reached = {problem.initial: node}
    while frontier:
        node = frontier.pop()
        if problem.is_goal(node.state):
            return node
        for child in expand(problem,node):
            s=child.state
            if s not in reached or child.path_cost < reached[s].path_cost:
                reached[s]= child
                frontier.add(child)
         
    return failure

def g(n): 
    # Write your code here ; modify the below mentioned line to find the actual cost
    cost = 1
    return cost
    
class RouteProblem(Problem):
    """A problem to find a route between locations on a `Map`.
    Create a problem with RouteProblem(start, goal, map=Map(...)}).
    States are the vertexes in the Map graph; actions are destination states."""
    
    def actions(self, state): 
        """The places neighboring `state`."""
        return self.map.neighbors[state]
    
    def result(self, state, action):
        """Go to the `action` place, if the map says that is possible."""
        return action if action in self.map.neighbors[state] else state
    
    def action_cost(self, s, action, s1):
        """The distance (cost) to go from s to s1."""
        return self.map.distances[s, s1]
    
    def h(self, node):
        "Straight-line distance between state and the goal."
        locs = self.map.locations
        return straight_line_distance(locs[node.state], locs[self.goal])
    
   class Map:
    """A map of places in a 2D world: a graph with vertexes and links between them. 
    In `Map(links, locations)`, `links` can be either [(v1, v2)...] pairs, 
    or a {(v1, v2): distance...} dict. Optional `locations` can be {v1: (x, y)} 
    If `directed=False` then for every (v1, v2) link, we add a (v2, v1) link."""

    def __init__(self, links, locations=None, directed=False):
        if not hasattr(links, 'items'): # Distances are 1 by default
            links = {link: 1 for link in links}
        if not directed:
            for (v1, v2) in list(links):
                links[v2, v1] = links[v1, v2]
        self.distances = links
        self.neighbors = multimap(links)
        self.locations = locations or defaultdict(lambda: (0, 0))

        
def multimap(pairs) -> dict:
    "Given (key, val) pairs, make a dict of {key: [val,...]}."
    result = defaultdict(list)
    for key, val in pairs:
        result[key].append(val)
    return result
    
   # Create your own map and define the nodes
# Create your own map and define the nodes
saveetha_nearby_locations = Map(
   {('SaveethaHospital', 'Chembarambakkam'):  7, ('Chembarambakkam', 'PoonamalleeBridge'): 6, ('PoonamalleeBridge', 'PoonamalleeBusTerminus'): 1, ('PoonamalleeBridge', 'SaveethaDentalCollege'): 3, ('PoonamalleeBusTerminus', 'Kattupakkam'): 2,
    ('PoonmalleeBusTerminal', 'Mangadu'):4, ('SaveethaDentalCollege', 'Kattupakkam'): 2, ('SaveethaDentalCollege', 'Maduravoyal'):  5, ('Maduravoyal', 'Koyambedu'): 5, ('Koyambedu', 'Vadapalani'): 5, ('Kattupakkam', 'Porur'): 4, 
    ('Porur', 'Vadapalani'): 8, ('Vadapalani', 'Guindy'):  8, ('Porur', 'Guindy'): 10, ('Mangadu', 'Kundrathur'): 4, ('Kundrathur', 'Porur'): 9, ('Kundrathur', 'Tiruneermalai'): 7, ('Kundrathur', 'Pammal'): 6, ('Pammal', 'Porur'): 10, ('Pammal', 'Guindy'): 14, ('Pammal', 'Airport'): 6, ('Tiruneermalai', 'Balaji college'): 2, ('Balaji college', 'Chrompet'): 2, ('Guindy', 'Velachery'): 4, ('Chrompet', 'Velachery'): 12})

r0 = RouteProblem('Maduravoyal', 'Velachery', map=saveetha_nearby_locations)
r1 = RouteProblem('Chembarambakkam', 'Guindy', map=saveetha_nearby_locations)
r2 = RouteProblem('PoonamalleeBridge', 'Kundrathur', map=saveetha_nearby_locations)
r3 = RouteProblem('SaveethaHospital', 'Kattupakkam', map=saveetha_nearby_locations)
r4 = RouteProblem('Mangadu', 'Guindy', map=saveetha_nearby_locations)

goal_state_path=best_first_search(r4,g)

print("GoalStateWithPath:{0}".format(goal_state_path))
path_states(goal_state_path) 
print("Total Distance={0} Kilometers".format(goal_state_path.path_cost))
```


## OUTPUT:

![Screenshot (99)](https://user-images.githubusercontent.com/75236145/167674283-d5d98469-8feb-468a-841d-361b3dbcb9a5.png)

## RESULT:
Hence, Best-First-Search Algorithm was implemented for a route finding problem.

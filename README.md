# Safe Roads

Discussion on calculating shortest and safest routes around town.

Use a Dijkstra's algorithm to calculated route - given a combined routing graph **cost_graph**, morality value **m**, **start_point** node and **end_point** node

Define our 'Safe' values - Example set {1, 2 , 3 , 4 } - **1** being the safest to **4** being unsafe.  Avoid using '0' - (see normalisation).


Combined costing graph - combined cost with two values distance and safety. - use a class to wrap two values.


```

SAFETY_SET = {1, 2, 3, 4} # Useful for asserting that our safety values are within our defined range
# 1 is the most safe, 4 is the least safe

class CombinedCost:

  def __init__(self, distance, safety):

      assert safety in SAFETY_SET, "Bad Safety Value"

      self.distance = distance 
      self.safety = safety


# Rather than two seperate costing graphs - define our new graph to use the CombinedCost class

combined_graph = [
          None, CombinedCost(2,3),   

  ]
  
```




**Morality Value m**

Combined cost (considering distance and safety) of travelling between nodes **n1** and **n2** 

```
            # weight is a CombinedCost object
            costing =  (1-m)*normalise(weight.distance, max_distance) + m*normalise(weight.safety, max_safety)


 ```

**m** is a 'float' value between 0 and 1

0 - meaning take a selfish attitude and find the shortest (distance) route - (giving no consideration to safety).

1 - meaning a selfless attitude - consider only safest route only.

Example of other values -

m = 0.25 means we are giving 3 times more consideration to minimising travelling distance than to safety.

m = 0.75 means we are 3 times more consideration to safety than minimising distance.



**max_distance_from_start_to_end_point** is the largest total distance taken from the set of all possible routes from our starting point to end point.

**max_safety_from_start_to_end_point** is the largest total safety value taken from the set of all possible routes from our starting point to end point.

This means we will need a routine to calculate all paths from given start node to our end node.

**normalise** function

```
def normalise(value, max_value):
  return value / (max_value + 0.0000001)
```  

The small fractional value is to make sure - we don't divide by zero - but should not be needed if choose appropriare safe values and don't have 0 distances between nodes.






## Flaws with this design

Travelling shortest distance is not always being selfish! It is good for the environment.




```
import sys

from heapq import heappush, heappop

# WARNING ! THIS IS AN AMMENDED COPY OF SOURCE FROM CHAT GPT
# When I asked the question 
# 'I want to calculate the shortest path in a graph representing distances between nodes.'


# TODO We need to find all possible routes from start to endpt and find the maxium distance and safety values


SAFETY_SET = {1, 2, 3, 4} # Useful for asserting that our safety values are within our defined range
# 1 is the most safe, 4 is the least safe

class CombinedCost():

    def __init__(self, distance, safety = 0):

        assert safety in SAFETY_SET, "Bad Safety Value"
        self.distance = distance
        self.safety = safety

    def __str__(self):
        return f"distance {self.distance} safety {self.safety}"

def normalise(value, max_value):
    return value/(max_value + (0.0000001 if max_value == 0 else 0))

def dijkstra(graph, start, morality, max_safety = 1, max_distance = 1):
    # Initialize distances and predecessors
    costs = {node: float('infinity') for node in graph}
    costs[start] = 0
    predecessors = {node: None for node in graph}

    # Priority queue to keep track of nodes with the smallest tentative cost
    priority_queue = [(0, start)]

    while priority_queue:
        current_cost, current_node = heappop(priority_queue)

        # Check if the current cost is smaller than the known cost
        if current_cost > costs[current_node]:
            continue

        for neighbor, weight in graph[current_node].items():
            cost = current_cost + (1-morality)*normalise(weight.distance, max_distance) + \
                          morality*normalise(weight.safety, max_safety)

            # Check if the new distance is smaller than the known cost
            if cost < costs[neighbor]:
                costs[neighbor] = cost
                predecessors[neighbor] = current_node
                heappush(priority_queue, (cost, neighbor))

    return costs, predecessors

def shortest_path(predecessors, destination):
    path = []
    current_node = destination

    while current_node is not None:
        path.insert(0, current_node)
        current_node = predecessors[current_node]

    return path

# Example graph represented as an adjacency dictionary
graph = {
    'A': {'B': CombinedCost(1, 1), 'C': CombinedCost(4, 2)},
    'B': {'A': CombinedCost(1, 4.0), 'C': CombinedCost(2), 'D': CombinedCost(5, 1.},
    'C': {'A': CombinedCost(4, 1), 'B': CombinedCost(2, 2), 'D': CombinedCost(1, 3)},
    'D': {'B': CombinedCost(5, 4), 'C': CombinedCost(1, 2)}
}

start_node = 'A'
end_node = 'D'

distances, predecessors = dijkstra(graph, start_node, 0)
shortest_path_route = shortest_path(predecessors, end_node)

print(f"Shortest distances: {distances}")
print(f"Shortest path from {start_node} to {end_node}: {shortest_path_route}")



```



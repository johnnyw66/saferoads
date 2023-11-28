import sys

from heapq import heappush, heappop

# WARNING ! THIS IS AN AMENDED COPY OF SOURCE FROM CHAT GPT
# When I asked the question 
# 'I want to calculate the shortest path in a graph representing distances between nodes.'
# I modified the code to give an alternative combined cost calculation - giving consideration to safety and distance.


# TODO We need to find all possible routes from start to endpt and find the maxium distance and safety values
#find_max_safe_value(grap,'A','D')
#find_max_distance_value(grap,'A','D')

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
    'B': {'A': CombinedCost(1, 4), 'C': CombinedCost(2, 4), 'D': CombinedCost(5, 1)},
    'C': {'A': CombinedCost(4, 1), 'B': CombinedCost(2, 2), 'D': CombinedCost(1, 3)},
    'D': {'B': CombinedCost(5, 4), 'C': CombinedCost(1, 2)}
}

# Example code which minimises cost (giving consideration to safety and distance travelled)
# Two functions find_max_safe_value() and find_max_distance_value() have to implemented
start_node = 'A'
end_node = 'D'

# morality = 0.25 gives 3 times more weight to minimising distance
# morality = 0.75 gives 3 times more weight to safe route)
# morality = 0 gives us shortest path (in terms of distance)
# morality = 1 gives us safest path (no consideration given to minimising distance)

# max_safe_value = find_max_safe_value(graph, start_node, end_node)   # From all possible routes from 'start_node' to 'end_node' work out a denominator for normalising safe values
# max_distance_value = find_max_distance_value(graph, start_node, end_node)  # From all possible routes from 'start_node' to 'end_node' work out a denominator for normalising safe values
#
morality = 0            
max_safe_value = 1     # Fudge
max_distance_value = 1 # Fudge

distances, predecessors = dijkstra(graph, start_node, morality, max_safety=max_safe_value, max_distance=max_distance_value)
shortest_path_route = shortest_path(predecessors, end_node)

print(f"Shortest distances: {distances}")
print(f"Shortest path from {start_node} to {end_node}: {shortest_path_route}")


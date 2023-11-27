# Safe Roads

Discussion on calculating shortest and safest routes around town.

Use a Dijkstra's algorithm to calculated route - given a combined routing graph **cost_graph**, morality value **m**, **start_point** node and **end_point** node

Define our 'Safe' values - Example set {1, 2 , 3 , 4 } - **1** being the safest to **4** being unsafe.  Avoid using '0' - (see normalisation).


Combined costing graph - combined cost with two values distance and safety. - use a class to wrap two values.


```

SAFETY_SET = {1, 2, 3, 4} # Useful for asserting that our safety values are within our defined range

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
cost() = (1 - m) * normalise(combined_graph[].distance, max_distance_from_start_to_end_point)   
    + m * normalise(combined_graph[].safety, max_safety_from_start_to_end_point)
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




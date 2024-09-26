# Search Algorithms Project

## Overview

This project implements two classic pathfinding algorithms: **Uniform Cost Search (UCS)** and **A* Search Algorithm**. Both algorithms are designed to find the shortest path in a grid-based environment, where nodes represent states and edges represent possible movements between those states.

### Uniform Cost Search (UCS)

Uniform Cost Search is a variant of Dijkstra's algorithm that expands the least-cost node first. It is particularly useful in scenarios where the cost of moving between nodes varies. UCS guarantees finding the optimal solution by always expanding the node with the lowest cumulative cost. The algorithm operates as follows:

1. Initialize the starting node and add it to the search frontier.
2. Continuously expand nodes from the frontier, updating costs and adding child nodes to the frontier.
3. Stop when a goal state is reached or the search frontier is empty.

### A* Search Algorithm

A* Search is an informed search algorithm that combines the benefits of UCS and heuristic-based search. It utilizes a heuristic function to estimate the cost to reach the goal from a given node. The A* algorithm is structured as follows:

1. Similar to UCS, initialize the starting node and add it to the search frontier.
2. For each node, calculate the total estimated cost, which is the sum of the path cost from the start node and the heuristic cost to the goal.
3. Expand nodes based on the lowest total cost.
4. The search continues until a goal state is found, utilizing the heuristic to prioritize nodes that are more likely to lead to an optimal solution.

Both algorithms are implemented to traverse a grid, accounting for blocked states and ensuring that only valid movements are considered.

## Features

- Implements Uniform Cost Search (UCS) and A* Search algorithms.
- Supports grid-based environments with free and blocked states.
- Prints the shortest path from the initial state to one of the goal states.
- Provides statistics such as process time, number of states visited, and total paths created.

## Constants
The project uses the following predefined constants:
- `#define N 5`: This defines the size of the maze (5x5 in this case).
- `#define P 0.2`: This defines the probability that a node in the maze will be blocked. 

## How to Run

To compile and run the project, execute the following commands:

```bash
gcc main.c 

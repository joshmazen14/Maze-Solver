# 3D Maze Solver

## Running the program
Run `python3 maze-solver.py`

Select your input (some samples have been provided)

View results in `output.txt`

## Description:

For my first AI project, I worked on a simple non-deterministic, accessible question: given a starting location, a goal location, a series of available states and actions, how would one find the most efficient solution to get to the exit? You can travel along each x-y-z axis, or diagonally along 2 dimensions. I employed 3 different AI search techniques to do so:

## Breadth-First Search

Assuming all path costs are equal, this is the simplest way to guarantee an optimal solution. It starts at the root (initial state) and analyzes each child node, branching off until a goal state is discovered.

## Uniform-Cost Search

This slightly more interesting form of search uses an algorithm somewhat similar to Djikstra's, determining the shortest path travelled so far and expanding along that node. This algorithm assumes travelling along the axes has a path cost of 10, while travelling diagonally has a path cost of 14.

## A* Seach

A* (or A star) adds an element of "guesswork" to speed up the selection process. It estimates the distance to the goal state from the current node and expands the node with the least estimated cost plus the cost so far. This is also guaranteed to be optimal, so long as the heuristic is admissible, ie never overestimates the distance to the goal. The heuristic used here is the Euclidian distance to the goal node scaled up a factor of 10. This is guaranteed to be at most equal to the distance to the goal if we use the same path costs as uniform-cost search.

## Actions

The actions are expected to be a number from 1 - 18. They are as follows:
1. +x
2. -x
3. +y
4. -y
5. +z
6. -z
7. +x+y
8. +x-y
9. -x+y
10. -x-y
11. +x+z
12. +x-z
13. -x+z
14. -x-z
15. +y+z
16. +y-z
17. -y+z
18. -y-z

## Input file format
- Line 1: Algorithm name (BFS, UCS, or A*)
- Line 2: Maximum dimensions of grid, space delimited
  - Example: 20 20 20
- Line 3: Location of start, space delimited
- Line 4: Location of goal, space delimited
- Line 5: Number of possible states
- Remaining Lines: The possible state values along with the available actions at that given state, space delimited
  - Example: 10 10 10 1 4 6 7
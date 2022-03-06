from collections import deque
import heapq

class Node:
  __slots__ = ['state', 'path_cost', 'total_cost', 'parent_node']

  def __init__(self, coords: tuple, path_cost: int, total_cost: int, parent_node = None):
    # A state is defined as a tuple of 3 coordinates: (x, y, z)
    self.state = coords
    self.path_cost = path_cost
    self.total_cost = total_cost
    self.parent_node = None if parent_node == None else parent_node

  # Iterate through each node's parent until we arrive at the root
  # Once at the root, join each node together, separated by a new line
  def create_solution(self):
    nodes = []
    curr = self
    while curr:
      nodes.append(curr)
      curr = curr.parent_node

    output_str = str(self.total_cost) + '\n'
    output_str += str(len(nodes)) + '\n'
    output_str += '\n'.join([str(node) for node in nodes[::-1]])
    return output_str
  
  def __str__(self):
    return ' '.join(map(str, self.state)) + ' ' + str(self.path_cost)

  def __lt__(self, other):
    return self.state < other.state

class Search:
  __slots__ = ['problem_space', 'root', 'goal', 'boundaries', 'frontier', 'explored', 'action_encoding', 'lowest_state_cost']

  def __init__(self, problem_space, initial_state, goal_state, boundaries):
    self.problem_space = problem_space
    self.root = initial_state
    self.goal = goal_state
    self.boundaries = boundaries
    self.frontier = []
    self.explored = set()
    self.action_encoding = self.get_action_encoding()
    self.lowest_state_cost = {}

  def search(self):
    # By default, return an empty solution. This will differ per algorithm
    return None

  # A mapping of actions number 1 - 18 to lambda functions corresponding with
  # the result of taking that action
  def get_action_encoding(self):
    return {
      1: lambda x,y,z: (x + 1, y, z),
      2: lambda x,y,z: (x - 1, y, z),
      3: lambda x,y,z: (x, y + 1, z),
      4: lambda x,y,z: (x, y - 1, z),
      5: lambda x,y,z: (x, y, z + 1),
      6: lambda x,y,z: (x, y, z - 1),
      7: lambda x,y,z: (x + 1, y + 1, z),
      8: lambda x,y,z: (x + 1, y - 1, z),
      9: lambda x,y,z: (x - 1, y + 1, z),
      10: lambda x,y,z: (x - 1, y - 1, z),
      11: lambda x,y,z: (x + 1, y, z + 1),
      12: lambda x,y,z: (x + 1, y, z - 1),
      13: lambda x,y,z: (x - 1, y, z + 1),
      14: lambda x,y,z: (x - 1, y, z - 1),
      15: lambda x,y,z: (x, y + 1, z + 1),
      16: lambda x,y,z: (x, y + 1, z - 1),
      17: lambda x,y,z: (x, y - 1, z + 1),
      18: lambda x,y,z: (x, y - 1, z - 1)
    }

  # By default, append node to a list
  def insert(self, node):
    self.frontier.append(node)

  # Check to see if a given node is at the goal state
  def goal_test(self, node: Node):
    return node.state == self.goal

  # Checks to ensure the state is in the boundaries of the maze
  def is_valid_state(self, state):
    return state >= (0, 0, 0) and state < self.boundaries
  
  # Advances to another state and returns it as a Node
  def get_child_node(self, node: Node, action):
    (x, y, z) = node.state
    encoding_fn = self.action_encoding.get(action, lambda *args: None)
    child_state = encoding_fn(x, y, z)
    if child_state != None and child_state in self.problem_space and self.is_valid_state(child_state):
      node_cost = self.get_action_cost(action)
      return Node(child_state, node_cost, node.total_cost + node_cost, node)
    else:
      return None
  
  # To preserve optimality in BFS, each node must have uniform cost
  def get_action_cost(self, action):
    return 1

# Assumes each path cost is equal, runs BFS starting from the root node
class BreadthFirstSearch(Search):
  def __init__(self, problem_space, initial_state, goal_state, boundaries):
    super().__init__(problem_space, initial_state, goal_state, boundaries)
    self.frontier = deque([])

  def search(self):
    if not self.is_valid_state(self.root):
      return None
    initial_node = Node(self.root, 0, 0)
    # If we start out on the initial solution, no need to enter the loop
    if self.goal_test(initial_node):
      return initial_node.create_solution()
    self.insert(initial_node)
    # Continue the loop so long as there are nodes to expand
    while self.frontier:
      node: Node = self.frontier.popleft()
      self.explored.add(node.state)
      for action in self.problem_space[node.state]:
        child = self.get_child_node(node, action)

        if child != None and child.state not in self.explored:
          if self.goal_test(child):
            return child.create_solution()
          self.insert(child)
    
    return None
  
  def insert(self, node):
    self.frontier.append(node)

# Advances to the node with the next node with the least total cost,
# Stops when the least cost path is popped from the frontier
class UniformCostSearch(Search):
  def search(self):
    if not self.is_valid_state(self.root):
      return None
    initial_node = Node(self.root, 0, 0)
    # If we start out on the initial solution, no need to enter the loop
    if self.goal_test(initial_node):
      return initial_node.create_solution()
    self.insert(initial_node)

    while self.frontier:
      curr = heapq.heappop(self.frontier)
      node: Node = curr[1]
      if node.total_cost > self.lowest_state_cost.get(node.state):
        continue
      elif self.goal_test(node):
        return node.create_solution()
      self.explored.add(node.state)
      for action in self.problem_space[node.state]:
        child = self.get_child_node(node, action)
        if child != None and (child.state not in self.explored or (
          child.state in self.lowest_state_cost and
          child.total_cost < self.lowest_state_cost.get(child.state)
        )):
          self.insert(child)
    return None


  def insert(self, node: Node):
    heapq.heappush(self.frontier, (node.total_cost, node))
    self.lowest_state_cost[node.state] = node.total_cost

  # Going along an axis costs 10, going along a non-axis plane is 14
  def get_action_cost(self, action):
    if action <= 6 and action >= 1:
      return 10
    elif action >= 7 and action <= 18:
      return 14
    else:
      return 0

# Advances to the node with the next node with the least total cost,
# preferring those with the least estimated cost to the node via an admissible heuristic
# Stops when the least cost path is popped from the frontier
class AStarSearch(Search):
  def search(self):
    if not self.is_valid_state(self.root):
      return None
    initial_node = Node(self.root, 0, 0)
    # If we start out on the initial solution, no need to enter the loop
    if self.goal_test(initial_node):
      return initial_node.create_solution()
    
    initial_estimate = self.heuristic_cost(self.root)
    self.insert(initial_node, estimated_cost=initial_estimate)

    while self.frontier:
      curr = heapq.heappop(self.frontier)
      node: Node = curr[1]

      if node.total_cost + self.heuristic_cost(node.state) > self.lowest_state_cost.get(node.state):
        continue
      elif self.goal_test(node):
        return node.create_solution()
      self.explored.add(node.state)
      for action in self.problem_space[node.state]:
        child = self.get_child_node(node, action)
        estimated_cost = child.total_cost + self.heuristic_cost(child.state)
        if child != None and child.state not in self.explored or estimated_cost < self.lowest_state_cost.get(child.state):
          self.insert(child, estimated_cost=estimated_cost)
    return None

  def insert(self, node: Node, estimated_cost=0):
    heapq.heappush(self.frontier, (estimated_cost, node))
    self.lowest_state_cost[node.state] = estimated_cost

  def heuristic_cost(self, current_state):
    (x1, y1, z1) = current_state
    (x2, y2, z2) = self.goal
    # The heuristic is the spatial distance to the goal state scaled up to 10
    return 10 * ((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2) ** .5

  # Going along an axis costs 10, going along a non-axis plane is 14
  def get_action_cost(self, action):
    if action <= 6 and action >= 1:
      return 10
    elif action >= 7 and action <= 18:
      return 14
    else:
      return 0

# Maps a string parsed from an input to a tuple of integers
def to_state(state):
  return tuple(map(int, state))

# set up problem space as a dictionary for O(1) access time
def configure_problem_space(locs, size):
  problem_space = {}
  for i in range(size):
    x, y, z, *actions = [int(loc) for loc in locs[i].split(' ')]
    problem_space[(x, y, z)] = actions
  return problem_space


file_name = input('Enter a file name: ')

def determine_search(search_type, problem_space, initial_state, goal_state, boundaries):
  if search_type == 'BFS':
    return BreadthFirstSearch(problem_space=problem_space, initial_state=initial_state, goal_state=goal_state, boundaries=boundaries)
  elif search_type == 'UCS':
    return UniformCostSearch(problem_space=problem_space, initial_state=initial_state, goal_state=goal_state, boundaries=boundaries)
  elif search_type == 'A*':
    return AStarSearch(problem_space=problem_space, initial_state=initial_state, goal_state=goal_state, boundaries=boundaries)
  else:
    return Search(problem_space=problem_space, initial_state=initial_state, goal_state=goal_state, boundaries=boundaries)

def write_search_to_output(s: Search):
  solution = s.search()
  f = open('output.txt', 'w')
  if solution == None:
    f.write('FAIL')
  else:
    f.write(solution)

with open(file_name, 'r') as f:
  search_type, dimensions, entrance, exit, num_locs, *locations = (line.strip() for line in f.readlines())
  problem_space = configure_problem_space(locations, int(num_locs))
  initial_state = to_state(entrance.split(' '))
  goal_state = to_state(exit.split(' '))
  boundaries = to_state(dimensions.split(' '))
  search = determine_search(search_type, problem_space, initial_state, goal_state, boundaries)

  write_search_to_output(search)
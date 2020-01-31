#   Look for #IMPLEMENT tags in this file. These tags indicate what has
#   to be implemented to complete the warehouse domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

import os #for time functions
from search import * #for search engines
from sokoban import SokobanState, Direction, PROBLEMS #for Sokoban specific classes and problems

def sokoban_goal_state(state):
  '''
  @return: Whether all boxes are stored.
  '''
  for box in state.boxes:
    if box not in state.storage:
      return False
  return True

def heur_manhattan_distance(state):
#IMPLEMENT
    '''admissible sokoban puzzle heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    #We want an admissible heuristic, which is an optimistic heuristic.
    #It must never overestimate the cost to get from the current state to the goal.
    #The sum of the Manhattan distances between each box that has yet to be stored and the storage point nearest to it is such a heuristic.
    #When calculating distances, assume there are no obstacles on the grid.
    #You should implement this heuristic function exactly, even if it is tempting to improve it.
    #Your function should return a numeric value; this is the estimate of the distance to the goal.
    manhattan_distance = 0
    for box in state.boxes:
      min_dist = float('inf')
      for storage in state.storage:
        dist = abs(box[0] - storage[0]) + abs(box[1] - storage[1])
        if dist < min_dist:
          min_dist = dist
      manhattan_distance += min_dist
    
    return manhattan_distance




#SOKOBAN HEURISTICS
def trivial_heuristic(state):
  '''trivial admissible sokoban heuristic'''
  '''INPUT: a sokoban state'''
  '''OUTPUT: a numeric value that serves as an estimate of the distance of the state (# of moves required to get) to the goal.'''
  count = 0
  for box in state.boxes:
    if box not in state.storage:
        count += 1
  return count

def heur_alternate(state):
#IMPLEMENT
    '''a better heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    #heur_manhattan_distance has flaws.
    #Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
    #Your function should return a numeric value for the estimate of the distance to the goal.
    h = 0   # heuristic value
    robot_positions = [robot for robot in state.robots]   # Current position of robots
    remaining_storages = [st for st in state.storage if st not in state.boxes]   # Remaining storages for boxes to go into
    remaining_boxes = [box for box in state.boxes if box not in state.storage]   # Remaining boxes that are not in storage yet

    for box in remaining_boxes:
      if not box_is_movable(state, box):
        return float('inf')   # Box is unmovable. State cannot be solved

      # Closest robot to box
      closest_robot = float('inf')
      closest_robot_dist = float('inf')
      # Closest storage to box
      closest_storage = float('inf')
      closest_storage_dist = float('inf')

      # Find closest robot
      for robot in robot_positions:
        dist = abs(box[0] - robot[0]) + abs(box[1] - robot[1])
        if dist < closest_robot_dist:
          closest_robot_dist = dist
          closest_robot = robot

      # Find closest storage
      for storage in remaining_storages:
        dist = abs(box[0] - storage[0]) + abs(box[1] - storage[1])
        if dist < closest_storage_dist:
          closest_storage_dist = dist
          closest_storage = storage

      # Cost to add to heuristic
      cost = closest_robot_dist + closest_storage_dist

      # Increase cost for obstacles in shortest path
      if len(state.obstacles) > 0:
        robot = closest_robot
        storage = closest_storage
        for x in range(min(robot[0], box[0]), max(robot[0], box[0])):
          if (x, robot[1]) in state.obstacles:
            cost += 2
        for y in range(min(robot[1], box[1]), max(robot[1], box[1])):
          if (box[0], y) in state.obstacles:
            cost += 2
        for x in range(min(storage[0], box[0]), max(storage[0], box[0])):
          if (x, box[1]) in state.obstacles:
            cost += 2
        for y in range(min(storage[1], box[1]), max(storage[1], box[1])):
          if (storage[0], y) in state.obstacles:
            cost += 2
      
      # After moving the box, the robot will be at the storage position, so move the robot there
      robot_positions[robot_positions.index(closest_robot)] = closest_storage
      # Remove storage spot from available storages
      remaining_storages.remove(closest_storage)
      # Add cost to heuristic
      h += cost
    return h

def box_is_movable(state, box):
  move_vertical = True    # Box can move vertically
  move_horizontal = True  # Box can move horizontally

  if box[0] == 0 or box[0] == state.width-1:
    # Box is at a vertical edge, so can no longer move horizontally
    move_horizontal = False
    if box[0] == 0 and len([st for st in state.storage if st[0] == 0]) == 0:
      return False  # No storage spots along vertical edge of box. State is unsolvable
    if box[0] == state.width-1 and len([st for st in state.storage if st[0] == state.width-1]) == 0:
      return False  # No storage spots along vertical edge of box. State is unsolvable

  if box[1] == 0 or box[1] == state.height-1:
    # Box is at a horizontal edge, so can no longer move vertically
    move_vertical = False
    if box[1] == 0 and len([st for st in state.storage if st[1] == 0]) == 0:
      return False  # No storage spots along horizontal edge of box. State is unsolvable
    if box[1] == state.height-1 and len([st for st in state.storage if st[1] == state.height-1]) == 0:
      return False  # No storage spots along horizontal edge of box. State is unsolvable

  if (box[0]-1, box[1]) in state.obstacles or (box[0]+1, box[1]) in state.obstacles:
    # There is an obstacle to the left of right of the box, so the box cannot move horizontally
    move_horizontal = False

  if (box[0], box[1]-1) in state.obstacles or (box[0], box[1]+1) in state.obstacles:
    # There is an obstacle above or below the box, so the box cannot move vertically
    move_vertical = False

  if not move_horizontal and not move_vertical:
    return False  # Box cannot move in either direction

  if not move_vertical:
    # Box can supposedly move horizontally. Verify this by checking the boxes to its left or right
    if (box[0]-1, box[1]) in state.boxes:
      # There is a box to the current box's left. Check if that box can move vertically (without considering any boxes
      # that could be vertically adjacent to it)
      if box[1] == 0 or box[1] == state.height-1 or (box[0]-1, box[1]-1) in state.obstacles or (box[0]-1, box[1]+1) in state.obstacles:
        return False  # Current box cannot move because the box to its left cannot move away to free space
    if (box[0]+1, box[1]) in state.boxes:
      # There is a box to the current box's right. Check if that box can move vertically (without considering any boxes
      # that could be vertically adjacent to it)
      if box[1] == 0 or box[1] == state.height-1 or (box[0]+1, box[1]-1) in state.obstacles or (box[0]+1, box[1]+1) in state.obstacles:
        return False  # Current box cannot move because the box to its right cannot move away to free space

  if not move_horizontal:
    # Box can supposedly move vertically. Verify this by checking the boxes above and below
    if (box[0], box[1]-1) in state.boxes:
      # There is a box above the current box. Check if that box can move horizontally (without considering any boxes
      # that could be horizontally adjacent to it)
      if box[0] == 0 or box[0] == state.width-1 or (box[0]-1, box[1]-1) in state.obstacles or (box[0]+1, box[1]-1) in state.obstacles:
        return False  # Current box cannot move because the box above it cannot move away to free space
    if (box[0], box[1]+1) in state.boxes:
      # There is a box below the current box. Check if that box can move horizontally (without considering any boxes
      # that could be horizontally adjacent to it)
      if box[0] == 0 or box[0] == state.width-1 or (box[0]-1, box[1]+1) in state.obstacles or (box[0]+1, box[1]+1) in state.obstacles:
        return False  # Current box cannot move because the box below it cannot move away to free space

  return True   # Box passes all reasonable checks and should be able to move (although not guaranteed)

def heur_zero(state):
    '''Zero Heuristic can be used to make A* search perform uniform cost search'''
    return 0

def fval_function(sN, weight):
#IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
  
    #Many searches will explore nodes (or states) that are ordered by their f-value.
    #For UCS, the fvalue is the same as the gval of the state. For best-first search, the fvalue is the hval of the state.
    #You can use this function to create an alternate f-value for states; this must be a function of the state and the weight.
    #The function must return a numeric f-value.
    #The value will determine your state's position on the Frontier list during a 'custom' search.
    #You must initialize your search engine object as a 'custom' search engine if you supply a custom fval function.
    return sN.gval + weight * sN.hval

def anytime_weighted_astar(initial_state, heur_fn, weight=1., timebound = 10):
#IMPLEMENT
  '''Provides an implementation of anytime weighted a-star, as described in the HW1 handout'''
  '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
  '''OUTPUT: A goal state (if a goal is found), else False'''
  '''implementation of weighted astar algorithm'''
  start = os.times()[0]   # Start Time
  se = SearchEngine('custom', 'full')   # Create search engine
  wrapped_fval_function = (lambda sN : fval_function(sN, weight))   # Wrapped fval function
  se.init_search(initial_state, sokoban_goal_state, heur_fn, wrapped_fval_function)   # Init search engine
  best_solution = se.search(timebound)    # Find initial solution with no pruning
  if not best_solution:
    return False  # Could not find a solution
  new_timebound = timebound - (os.times()[0] - start)   # Update timebound
  while new_timebound > 0 and weight >= 1:    # No need to check for weights below 1
    weight /= 2   # Update weight
    if weight < 1:  # No need to check for weights below 1
      weight = 1
    # Obtain new solution by pruning with the best solution found so far
    new_solution = se.search(new_timebound, (best_solution.gval, best_solution.gval, best_solution.gval))
    if not new_solution:
      # Could not find a better solution. Return best solution found so far
      return best_solution
    best_solution = new_solution  # Update best solution with new, better solution
    if weight == 1:
      return best_solution  # Weight of 1 outputs a provably optimal solution
    new_timebound = timebound - (os.times()[0] - start) # Update timebound
  return best_solution  # Return best solution found so far

def anytime_gbfs(initial_state, heur_fn, timebound = 10):
#IMPLEMENT
  '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
  '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
  '''OUTPUT: A goal state (if a goal is found), else False'''
  '''implementation of weighted astar algorithm'''
  start = os.times()[0]   # Start Time
  se = SearchEngine('best_first', 'full')   # Update timebound
  se.init_search(initial_state, sokoban_goal_state, heur_fn)   # Init search engine
  best_solution = se.search(timebound)    # Find initial solution with no pruning
  if not best_solution:
      return False  # Could not find a solution
  new_timebound = timebound - (os.times()[0] - start)    # Update timebound
  while new_timebound > 0:
    # Obtain new solution by pruning with the best solution found so far
    new_solution = se.search(new_timebound, (best_solution.gval, best_solution.gval, best_solution.gval))
    if not new_solution:
      # Could not find a better solution. Return best solution found so far
      return best_solution
    best_solution = new_solution      # Update best solution with new, better solution
    new_timebound = timebound - (os.times()[0] - start)    # Update timebound
  return best_solution    # Return best solution found so far

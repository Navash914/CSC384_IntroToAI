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
      min_dist = 10000  # Just a large number, interprete as infinity
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
    h = 0
    robot_positions = []
    remaining_storages = []
    robot_positions.extend(state.robots)
    remaining_storages.extend(state.storage)
    for box in state.boxes:
      if box in state.storage:
        continue
      #if not box_is_movable(state, box):
      if unmoveable(state, box):
        return float('inf')
      closest_robot = float('inf')
      closest_robot_dist = float('inf')
      closest_storage = float('inf')
      closest_storage_dist = float('inf')
      for i in range(len(robot_positions)):
        robot = robot_positions[i]
        dist = abs(box[0] - robot[0]) + abs(box[1] - robot[1])
        if dist < closest_robot_dist:
          closest_robot_dist = dist
          closest_robot = i
      for j in range(len(remaining_storages)):
        storage = remaining_storages[j]
        #if storage in state.boxes:
        #  continue
        dist = abs(box[0] - storage[0]) + abs(box[1] - storage[1])
        if dist < closest_storage_dist:
          closest_storage_dist = dist
          closest_storage = j
      #if len(remaining_storages) > 0:
      robot_positions[closest_robot] = remaining_storages[closest_storage]
      remaining_storages.pop(closest_storage)
      h += closest_robot_dist + closest_storage_dist
    return h

#Test:
def unmoveable(state, box):
    ''' Check whether the box is in some place that unable to move.'''

    up_wall = False
    down_wall = False
    left_wall = False
    right_wall = False
    diag_wall = False
    up_box = False
    down_box = False
    left_box = False
    right_box = False
    diag_box = False

    up = (box[0], box[1] - 1)
    down = (box[0], box[1] + 1)
    left = (box[0] - 1, box[1])
    right = (box[0] + 1, box[1])
    diag = (box[0] + 1, box[1] + 1)

    if box[0] == 0 or left in state.obstacles:
        left_wall = True
    elif left in state.boxes:
        left_box = True
    if box[0] == state.width - 1 or right in state.obstacles:
        right_wall = True
    elif right in state.boxes:
        right_box = True
    if box[1] == 0 or down in state.obstacles:
        down_wall = True
    elif down in state.boxes:
        down_box = True
    if box[1] == state.height - 1 or up in state.obstacles:
        up_wall = True
    elif up in state.boxes:
        up_box = True
    if diag in state.boxes:
        diag_box = True
    elif (right_wall or down_wall) or diag in state.obstacles:
        diag_wall = True

    if ((up_wall or down_wall) and (right_wall or left_wall or right_box or left_box))or((right_wall or left_wall) \
       and (up_box or down_box)) or ((down_wall or down_box) and (diag_wall or diag_box) and (right_wall or right_box)):
        return True
    return False

def box_is_movable(state, box):
  return can_move_up(state, box) or can_move_down(state, box) or can_move_left(state, box) or can_move_right(state, box)

def can_move_up(state, box):
  if (box[1] == 0):
    return False
  new_location = (box[0], box[1]-1)
  if (new_location in state.obstacles):
    return False
  if (new_location in state.boxes):
    #return can_move_left(state, new_location) or can_move_right(state, new_location) or can_move_up(state, new_location)
    return False
  return True

def can_move_down(state, box):
  if (box[1] == state.height-1):
    return False
  new_location = (box[0], box[1]+1)
  if (new_location in state.obstacles):
    return False
  if (new_location in state.boxes):
    #return can_move_left(state, new_location) or can_move_right(state, new_location) or can_move_down(state, new_location)
    return False
  return True

def can_move_left(state, box):
  if (box[0] == 0):
    return False
  new_location = (box[0]-1, box[1])
  if (new_location in state.obstacles):
    return False
  if (new_location in state.boxes):
    #return can_move_left(state, new_location) or can_move_up(state, new_location) or can_move_down(state, new_location)
    return False
  return True

def can_move_right(state, box):
  if (box[0] == state.width-1):
    return False
  new_location = (box[0]+1, box[1])
  if (new_location in state.obstacles):
    return False
  if (new_location in state.boxes):
    #return can_move_up(state, new_location) or can_move_right(state, new_location) or can_move_down(state, new_location)
    return False
  return True

def box_in_corner(state, box):
  if box[0] == 0 and box[1] == 0:
    return True
  if box[0] == 0 and box[1] == state.height-1:
    return True
  if box[0] == state.width-1 and box[1] == 0:
    return True
  if box[0] == state.width-1 and box[1] == state.height-1:
    return True
  return False

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

def anytime_weighted_astar(initial_state, heur_fn, weight=5., timebound = 10):
#IMPLEMENT
  '''Provides an implementation of anytime weighted a-star, as described in the HW1 handout'''
  '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
  '''OUTPUT: A goal state (if a goal is found), else False'''
  '''implementation of weighted astar algorithm'''
  start = os.times()[0]
  se = SearchEngine('astar', 'path')
  wrapped_fval_function = (lambda sN : fval_function(sN, weight))
  se.init_search(initial_state, sokoban_goal_state, heur_fn, wrapped_fval_function)
  best_solution = se.search(timebound)
  if not best_solution:
    return False
  new_timebound = timebound - (os.times()[0] - start)
  while new_timebound > 0:
      weight /= 2
      if weight < 1:
        weight = 1
      new_solution = se.search(new_timebound, (best_solution.gval, float('inf'), best_solution.gval))
      if not new_solution:
          return best_solution
      best_solution = new_solution
      if weight == 1:
        return best_solution
      new_timebound = timebound - (os.times()[0] - start)
  return best_solution

def anytime_gbfs(initial_state, heur_fn, timebound = 10):
#IMPLEMENT
  '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
  '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
  '''OUTPUT: A goal state (if a goal is found), else False'''
  '''implementation of weighted astar algorithm'''
  start = os.times()[0]
  se = SearchEngine('best_first', 'path')
  se.init_search(initial_state, sokoban_goal_state, heur_fn)
  best_solution = se.search(timebound)
  if not best_solution:
      return False
  new_timebound = timebound - (os.times()[0] - start)
  while new_timebound > 0:
      new_solution = se.search(new_timebound, (best_solution.gval, best_solution.gval, best_solution.gval))
      if not new_solution:
          return best_solution
      best_solution = new_solution
      new_timebound = timebound - (os.times()[0] - start)
  return best_solution

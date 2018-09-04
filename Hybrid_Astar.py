import matplotlib.pyplot as plt
import math
import numpy as np
import scipy.interpolate as si
import copy
W=5
SPEED=5
class State:
  def __init__(x,y,theta,g,h,f,vr,vl):
    self.x=x
    self.y=y
    self.theta=theta
    self.g=g
    
    self.h=h
    self.f=f
    self.vr=vr
    self.vl=vl

def velocity_augmentation(vel,offset):
  vr=2* vel
  vl=0
  x=(int)(vr//offset)
  lst=[]
  for i in range(x):
    if((vr-vl)!=0):
      omega=vr-vl/W
      k=omega*57.2958
      if(k<30 and k>-30):
        a=[vr,vl,k]
       
        
        lst.append(a)
    vr=vr-offset
    vl=vl+offset
  return lst




def heuristic(x,y,goal):
  a=math.pow((goal.x-x),2)
  b=math.pow((goal.y-y),2)
  return math.sqrt(a+b)


def expand(state, goal):
    next_states = []
    lst=velocity_augmentation(SPEED,0.02)  
    for delta in lst: 
        # Create a trajectory with delta as the steering angle using the bicycle model:

        # ---Begin unicycle model---
        omega = lst[2]
        next_x = state.x + SPEED * cos(theta)
        next_y = state.y + SPEED * sin(theta)
        next_theta = normalize(state.theta + omega)
        # ---End unicycle model-----

        next_g = state.g + 1
        next_h =  heuristic(next_x, next_y, goal)
        next_f=next_g+next_h

        # Create a new State object with all of the "next" values.
        state = State(next_x, next_y, next_theta, next_g,next_h, next_f,lst[0],lst[1])
        next_states.append(state)

    return next_states

def search(grid, start, goal):
    # The opened array keeps track of the stack of States objects we are 
    # searching through.
    opened = []
    # 3D array of zeros with dimensions:
    # (NUM_THETA_CELLS, grid x size, grid y size).
    closed = [[[0 for x in range(grid[0])] for y in range(len(grid))] for cell in range(NUM_THETA_CELLS)]
    # 3D array with same dimensions. Will be filled with State() objects to keep 
    # track of the path through the grid. 
    came_from = [[[0 for x in range(grid[0])] for y in range(len(grid))] for cell in range(NUM_THETA_CELLS)]

    # Create new state object to start the search with.
    x = start.x
    y = start.y
    theta = start.theta
    g = 0
    h = heuristic(start.x, start.y, goal)
    f=g+h
    vr=0
    vl=0
    state = State(x, y, theta, 0,h, f,vr,vl)
    opened.append(state)

    # The range from 0 to 2pi has been discretized into NUM_THETA_CELLS cells. 
    # Here, theta_to_stack_number returns the cell that theta belongs to. 
    # Smaller thetas (close to 0 when normalized  into the range from 0 to 2pi) 
    # have lower stack numbers, and larger thetas (close to 2pi whe normalized)
    # have larger stack numbers.
    stack_number = theta_to_stack_number(state.theta)
    closed[stack_number][index(state.x)][index(state.y)] = 1

    # Store our starting state. For other states, we will store the previous state 
    # in the path, but the starting state has no previous.
    came_from[stack_number][index(state.x)][index(state.y)] = state

    # While there are still states to explore:
    while opened:
        # Sort the states by f-value and start search using the state with the 
        # lowest f-value. This is crucial to the A* algorithm; the f-value 
        # improves search efficiency by indicating where to look first.
        opened.sort(key=lambda state:state.f)
        current = opened.pop(0)

        # Check if the x and y coordinates are in the same grid cell as the goal. 
        # (Note: The idx function returns the grid index for a given coordinate.)
        if (idx(current.x) == goal[0]) and (idx(current.y) == goal.y):
            # If so, the trajectory has reached the goal.
            return path

        # Otherwise, expand the current state to get a list of possible next states.
        next_states = expand(current, goal)
        for next_state in next_states:
            # If we have expanded outside the grid, skip this next_state.
            if next_states is not in the grid:
                continue
            # Otherwise, check that we haven't already visited this cell and
            # that there is not an obstacle in the grid there.
            stack_number = theta_to_stack_number(next_state.theta)
            if closed_value[stack_number][idx(next_state.x)][idx(next_state.y)] == 0 and grid[idx(next_state.x)][idx(next_state.y)] == 0:
                # The state can be added to the opened stack.
                opened.append(next_state)
                # The stack_number, idx(next_state.x), idx(next_state.y) tuple 
                # has now been visited, so it can be closed.
                closed[stack_number][idx(next_state.x)][idx(next_state.y)] = 1
                # The next_state came from the current state, and that is recorded.
                came_from[stack_number][idx(next_state.x)][idx(next_state.y)] = current
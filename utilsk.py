import matplotlib.pyplot as plt
import IPython.display as display
import time
from pyswip import Prolog
from minihack import LevelGenerator
from minihack import RewardManager
import numpy as np
import random

import heapq

actions={
    "north":0,
    "east":1,
    "south":2,
    "west":3,

    "northeast":4,
    "southeast":5,
    "southwest":6,
    "northwest":7,

    "pick":8,
    "open_door":9
    
}

positions={
    "agent":None,
    "key":None,
    "door":None,
    "door_front":None,
    "stairs": None,
}
def initialize():
    
    positions={
    "agent":None,
    "key":None,
    "door":None,
    "door_front":None,
    "stairs":None
    }

    
def is_infront(i,j,i_,j_):
    if (i == i_ and abs(j-j_)==1) or (j == j_ and abs(i-i_)==1):
        return True
    return False    


def heuristic(a, b):
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)
    
def coordinates_to_actions(coordinates):
    actions = []

    for i in range(len(coordinates) - 1):
        current_x, current_y = coordinates[i]
        next_x, next_y = coordinates[i + 1]

        dx = next_x - current_x
        dy = next_y - current_y

        # Determine diagonal directions
        if dx != 0 and dy != 0:
            if dx > 0 and dy > 0:
                actions.append("southeast")
            elif dx > 0 and dy < 0:
                actions.append("southwest")
            elif dx < 0 and dy > 0:
                actions.append("northeast")
            elif dx < 0 and dy < 0:
                actions.append("northwest")
            continue
                
        # Determine the vertical direction
        if dy > 0:
            actions.append("east")
        elif dy < 0:
            actions.append("west")

        # Determine the horizontal direction
        if dx > 0:
            actions.append("south")
        elif dx < 0:
            actions.append("north")

        

    return actions

def astar(array, start, goal):
    if start is None or goal is None:
        raise ValueError("Start and/or goal coordinates cannot be None.")
        
    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]

    close_set = set()

    came_from = {}

    gscore = {start:0}

    fscore = {start:heuristic(start, goal)}

    oheap = []

    heapq.heappush(oheap, (fscore[start], start))
 

    while oheap:

        current = heapq.heappop(oheap)[1]

        if current == goal:

            data = []

            while current in came_from:

                data.append(current)

                current = came_from[current]

            return data

        close_set.add(current)

        for i, j in neighbors:

            neighbor = current[0] + i, current[1] + j

            tentative_g_score = gscore[current] + heuristic(current, neighbor)

            if 0 <= neighbor[0] < array.shape[0]:

                if 0 <= neighbor[1] < array.shape[1]:                

                    if array[neighbor[0]][neighbor[1]] == 1:

                        continue

                else:

                    # array bound y walls

                    continue

            else:

                # array bound x walls

                continue
 

            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):

                continue
 

            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:

                came_from[neighbor] = current

                gscore[neighbor] = tentative_g_score

                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)

                heapq.heappush(oheap, (fscore[neighbor], neighbor))
 
    # return random_direction = random.choice([(x-1, y-1), (x, y-1), (x+1, y-1),(x-1, y),(x+1, y),(x-1, y+1),(x, y+1),(x+1, y+1)])
    raise ValueError("Can't find no path")

#finds a front location in the map for something. for example: a door
def front_location(obs,i,j):
    if obs['chars'][i+1][j]==46:
        return (i+1,j)
    elif obs['chars'][i][j+1]==46:
        return (i,j+1)
    elif obs['chars'][i][j-1]==46:
        return (i,j-1)
    elif obs['chars'][i-1][j]==46:
        return (i-1,j)
    raise ValueError("invalid i,j")
    

def find_borderline_cells(grid):
    # Identify borderline cells
    grid=grid['chars']
    borderline_cells = set()

    for i in range(21):
        for j in range(79):
            #46 is "."
            if grid[i][j] == 46:
                # Check if any adjacent neighbor has a value of 32
                for dx in [-1, 0, 1]:
                    for dy in [-1, 0, 1]:
                        if 0 <= i + dx  and 0 <= j + dy and grid[i + dx][j + dy] == 32:
                            borderline_cells.add((i, j))
                            

    return list(borderline_cells)  


def decide_next_cell_to_explore(grid, random_probability=0.2):
    borderline_cells = find_borderline_cells(grid)
    x,y=positions['agent']
    if not borderline_cells:
        # No borderline cells remaining, choose a random direction
        
        random_direction = random.choice([(x-1, y-1), (x, y-1), (x+1, y-1),(x-1, y),(x+1, y),(x-1, y+1),
                        (x, y+1),
                        (x+1, y+1)])
        
        return random_direction

    # Introduce a random choice with the given probability
    if random.random() < random_probability:
        return random.choice(borderline_cells)

    # Find the closest borderline cell using straight-line distance
    min_distance = float('inf')
    closest_borderline_cell = None

    for cell in borderline_cells:
        distance = heuristic((x,y), cell)
        if distance < min_distance:
            min_distance = distance
            closest_borderline_cell = cell

    return closest_borderline_cell

def process_state(obs: dict, kb: Prolog):
    positions["agent"] = obs['blstats'][1], obs['blstats'][0] 
    for i in range(21):
        for j in range(79):
            if not (obs['screen_descriptions'][i][j] == 0).all():
                obj = bytes(obs['screen_descriptions'][i][j]).decode('utf-8').rstrip('\x00')
                #staircase down
                if obs["glyphs"][i][j] == 2383:
                    kb.assertz(f'in_sight(stairs)')
                    positions["stairs"] = i, j 
                    
                if 'key' in obj:
                    kb.assertz(f'in_sight(key)')
                    positions["key"] = i, j 
                    
                elif 'door' in obj:
                    kb.assertz(f'in_sight(door)')
                    positions["door"] = i, j
                    if positions["door_front"] is None:
                        positions["door_front"]=front_location(obs,i,j)
                    if is_infront(positions["agent"][0],positions["agent"][1],i,j):
                        print("next to door")
                        kb.assertz(f'standing_next(door)')
                    else:
                        kb.retractall('standing_next(door)')
                if 'open' in obj and 'door' in obj:
                    kb.assertz(f'is_open(door)')
                   
    # for i in range(21):
    #     for j in range(79):
    #         obs['chars'][i][j]
                
    
    for obj in obs['inv_strs']:
        obj = bytes(obj).decode('utf-8').rstrip('\x00')
        if 'key' in obj:
            kb.asserta(f'has(key)')
            kb.retractall('in_sight(key)')

                    
    message = bytes(obs['message']).decode('utf-8').rstrip('\x00')
    if 'You see here' in message:
        if 'key' in message:
            kb.assertz('standing_on(key)')
    else:
        kb.retractall('standing_on(key)')
        
def obschar_to_mask(obs):
    # Create a mask for cells with ASCII values "(" or "." or "<" or ">" or "2373(open door) | or 2372 opendoor "-"
    mask = (obs['chars'] == 40) | (obs['chars'] == 46) | (obs['chars'] == 60) |(obs['chars'] == 62) | (obs['glyphs'] == 2373) | (obs['glyphs'] == 2372)

    # mask = np.logical_or(obs['chars'] == 40, obs['chars'] == 46, obs['chars'] == 60, obs['glyphs']==2373)

    # Convert to a new 2D array with 0 or 1
    new_array = np.where(mask, 0, 1)

    return new_array
def perform_action(action, env, kb):
    print(action)
    if action == 'pick':
        action_id = 8
        kb.retractall('standing_on(key)')
        kb.retractall('in_sight(key)')
        
    elif action == 'opendoor':
        action_id = 9
        kb.retractall('has(key)')

    # Movement/Attack/Run/Get_To_Weapon actions
    # in the end, they all are movement in a direction
    elif 'northeast' in action: action_id = 4
    elif 'southeast' in action: action_id = 5
    elif 'southwest' in action: action_id = 6
    elif 'northwest' in action: action_id = 7
    elif 'north' in action: action_id = 0
    elif 'east' in action: action_id = 1
    elif 'south' in action: action_id = 2
    elif 'west' in action: action_id = 3

    # print(f'Action performed: {repr(env.actions[action_id])}')
    obs, reward, done, info = env.step(action_id)
    return obs, reward, done, info
    


# indexes for showing the image are hard-coded
def show_match(states: list):
    image = plt.imshow(states[0][115:275, 480:750])
    for state in states[1:]:
        time.sleep(0.25)
        display.display(plt.gcf())
        display.clear_output(wait=True)
        image.set_data(state[115:275, 480:750])
    time.sleep(0.25)
    display.display(plt.gcf())
    display.clear_output(wait=True)
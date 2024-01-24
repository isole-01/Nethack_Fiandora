import matplotlib.pyplot as plt
import IPython.display as display
import time
import numpy as np
import random
import heapq
from pyswip import Prolog
from minihack import LevelGenerator

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
    "open_door":9,
    
}
positions={
    "agent":None,
    "key":None,
    "door":None,
    "door_front":None,
    "stairs": None,
    "corridor_entrance": None
}

def initialize():
    
    for key in positions:
        positions[key] = None
    
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
    x,y=positions["agent"]
    return -1
    # return [random.choice([(x-1, y-1), (x, y-1), (x+1, y-1),(x-1, y),(x+1, y),(x-1, y+1),(x, y+1),(x+1, y+1)])]
    # raise ValueError("Can't find no path")

#finds a front location in the map for something. for example: a door
#64 is @
def front_location(obs,i,j):
    if obs['chars'][i+1][j] in [46,64]:
        return (i+1,j)
    elif obs['chars'][i][j+1]in [46,64]:
        return (i,j+1)
    elif obs['chars'][i][j-1]in [46,64]:
        return (i,j-1)
    elif obs['chars'][i-1][j]in [46,64]:
        return (i-1,j)
    if obs["chars"] == 35:
        return None
    raise ValueError("invalid i,j")
    

def find_borderline_cells(grid):
    # Identify borderline cells
    grid=grid['chars']
    borderline_cells = set()

    for i in range(21):
        for j in range(79):
            #46 is "." 
            if grid[i][j] == 46 and grid[i][j-1] != 35 and grid[i][j+1] != 35:
                # Check if any adjacent neighbor has a value of 32
                for dx in [-1, 0, 1]:
                    for dy in [-1, 0, 1]:
                        if 0 <= i + dx  and 0 <= j + dy and grid[i + dx][j + dy] == 32:
                            borderline_cells.add((i, j))
            # elif grid[i][j] == 35:
                
                            

    return list(borderline_cells)  


def decide_next_cell_to_explore(grid, random_probability=0.01, eta=0.7):
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

    min_distance = float('inf')
    closest_priority_cell = None

    for cell in borderline_cells:
        straight_line_distance = heuristic((x, y), cell)

        # Calculate average distance to other b_cells
        avg_distance_to_b_cells = sum(heuristic(cell, other_cell) for other_cell in borderline_cells) / len(borderline_cells)

        # Combine the straight-line distance and average distance as criteria
        combined_distance = straight_line_distance + eta*avg_distance_to_b_cells

        if combined_distance < min_distance:
            min_distance = combined_distance
            closest_priority_cell = cell

    return closest_priority_cell

def obschar_to_mask(obs):

    mask = ~((obs['chars'] == ord(' ')) | (obs['chars'] == ord('-')) | (obs['chars'] == ord('|')))

    mask = mask | (obs['glyphs'] == 2373) | (obs['glyphs'] == 2372)
    # Create a mask for cells with ASCII values "(" or "." or "<" or ">" or 35 # or "2373(open door) | or 2372 opendoor "-"  
    # mask = (obs['chars'] == ord('%')) | (obs['chars'] == 40) | (obs['chars'] == 46) | (obs['chars'] == 60) |(obs['chars'] == 62) | (obs['chars'] == 35) | (obs['glyphs'] == 2373) | (obs['glyphs'] == 2372)


    # Convert to a new 2D array with 0 or 1
    new_array = np.where(mask, 0, 1)

    return new_array

def is_wall(obs,i,j):
    obj = bytes(obs['screen_descriptions'][i][j]).decode('utf-8').rstrip('\x00')
    if 'wall' in obj:
        return True
    return False

def near_door(obs):
    i,j=positions["agent"]
    if not positions["door"]:
        return False
    l=heuristic((i,j),positions["door"])
    if l < 1.5:
        return True
    return False




def process_state(obs: dict, kb: Prolog):

    def get_abs_corridor_position():

        def is_rat_gift(cell):
            return (cell not in [vertical_wall, generic_floor, horizontal_wall, corridor_tile, rat])

        vertical_wall = ord('|')
        horizontal_wall = ord('-')
        corridor_tile = ord('#')
        rat = ord('r')
        generic_floor = ord('.')
        
        #print_debug("agent left: " + chr(agent_left) + "   agent right: " + chr(agent_right) + "\n")
        #print_debug("agent top: " + chr(agent_top) + "   agent bottom: " + chr(agent_bottom) + "\n")

        # CORRIDOR POSITIONS ON LEFT________________________
        if ( agent_top_left == vertical_wall and 
            agent_bottom_left == vertical_wall and 
            (agent_right==corridor_tile or agent_right==rat)):
            return "corridor_left"
        elif (agent_top == vertical_wall and 
                agent_bottom == vertical_wall and 
                (agent_right==corridor_tile or (agent_right==rat and already_in_corridor == False))):
            return "corridor_left_minus1"
        elif ( already_in_corridor == False and
                agent_top_right == vertical_wall and 
                agent_bottom_right == vertical_wall and 
                agent_right==rat):
            return "corridor_left_minus2"
        
        # CORRIDOR POSITIONS ON RIGHT________________________
        elif ( agent_top_right == vertical_wall and 
            agent_bottom_right == vertical_wall and 
            (agent_left==corridor_tile or agent_left==rat)):
            return "corridor_right"

        elif (agent_top == vertical_wall and 
                agent_bottom == vertical_wall and 
                (agent_left==corridor_tile or (agent_left==rat and already_in_corridor == False))):
            return "corridor_right_plus1"
        elif ( already_in_corridor == False and
                agent_top_left == vertical_wall and 
                agent_bottom_left == vertical_wall and 
                agent_left==rat):
            return "corridor_right_plus2"
        
        # CORRIDOR POSITIONS ON BOTTOM________________________
        elif ( agent_bottom_left == horizontal_wall and 
            agent_bottom_right == horizontal_wall and 
            (agent_top==corridor_tile or agent_top==rat)):
            return "corridor_bottom"
        elif (agent_right == horizontal_wall and 
                agent_left == horizontal_wall and 
                (agent_top==corridor_tile or (agent_top==rat and already_in_corridor == False))):
            return "corridor_bottom_minus1"
        elif ( already_in_corridor == False and
                agent_top_left == horizontal_wall and 
                agent_top_right == horizontal_wall and 
                agent_top==rat):
            return "corridor_bottom_minus2"
        
        # CORRIDOR POSITIONS ON TOP________________________
        elif ( agent_top_left == horizontal_wall and 
            agent_top_right == horizontal_wall and 
            (agent_bottom==corridor_tile or agent_bottom==rat)):
            return "corridor_top"
        elif (agent_right == horizontal_wall and 
                agent_left == horizontal_wall and 
                (agent_bottom==corridor_tile or (agent_bottom==rat and already_in_corridor == True))):
            return "corridor_top_plus1"
        elif ( already_in_corridor == False and
                agent_bottom_left == horizontal_wall and 
                agent_bottom_right == horizontal_wall and 
                agent_bottom==rat):
            return "corridor_top_plus2"
        
        # CORRIDOR POSITIONS IN THE MIDDLE__________________
        elif ((agent_top == corridor_tile and agent_bottom == corridor_tile) or
            (agent_right == corridor_tile and agent_left == corridor_tile) or

            (agent_right == corridor_tile and agent_left in [rat, generic_floor]) or 
            (agent_right == corridor_tile and is_rat_gift(agent_left)) or 

            (agent_right in [rat, generic_floor] and agent_left == corridor_tile) or
            (is_rat_gift(agent_right) and agent_left == corridor_tile) or

            (agent_top == corridor_tile and agent_bottom in [rat, generic_floor]) or
            (agent_top == corridor_tile and is_rat_gift(agent_bottom)) or

            (agent_top in [rat, generic_floor] and agent_bottom == corridor_tile) or 
            (is_rat_gift(agent_top) and agent_bottom == corridor_tile)):

            return "middle_corridor"

        
        # NOT IN THE CORRIDOR__________________
        else:
            return "not_in_corridor"



    def get_current_place():
        
        if (abs_corr_position == "middle_corridor"):
            return "middle_corridor"
        
        elif (abs_corr_position == "not_in_corridor"):
            return "not_in_corridor"
        
        elif (abs_corr_position in ["corridor_bottom_minus2", "corridor_top_plus2",
                                    "corridor_left_minus2", "corridor_right_plus2"]):
            if already_in_corridor == True:
                return "corridor_end_plus2"
            else: 
                return "corridor_start_minus2"
            
        elif (abs_corr_position in ["corridor_bottom_minus1", "corridor_top_plus1",
                                    "corridor_left_minus1", "corridor_right_plus1"]):
            if already_in_corridor == True:
                return "corridor_end_plus1"
            else: 
                return "corridor_start_minus1"
        
        elif (abs_corr_position in ["corridor_bottom", "corridor_top",
                                    "corridor_left", "corridor_right"]):
            

            try:
                last_extreme_abs = list(kb.query('last_extreme_abs(X)'))[0]
                last_extreme_abs = last_extreme_abs['X']

                last_extreme_rel = list(kb.query('last_extreme_rel(X)'))[0]
                last_extreme_rel = last_extreme_rel['X']
            except Exception as e:
                #print(e)
                last_extreme_rel = None
                last_extreme_abs = None

            kb.retractall("last_extreme_abs(_)")
            kb.asserta(f"last_extreme_abs({abs_corr_position})")
            #print_debug("LAST EXTREME ABS: " + str(abs_corr_position))

            if already_in_corridor == True:
                if (last_extreme_abs == abs_corr_position):
                    #print_debug("PLACE FUNCTION part1")
                    return last_extreme_rel
                    
                elif (last_extreme_rel == "corridor_end"):
                    kb.retractall("last_extreme_rel(_)")
                    kb.asserta("last_extreme_rel(corridor_start)")
                    #print_debug("PLACE FUNCTION part2")
                    return "corridor_start"
                    
                else:
                    kb.retractall("last_extreme_rel(_)")
                    kb.asserta("last_extreme_rel(corridor_end)")
                    #print_debug("PLACE FUNCTION part3")
                    return "corridor_end"
                    
                
            else: 
                return "corridor_start"
        
        else:
            return "error_in_position_detection"
    
    def get_corridor_type():
        if abs_corr_position in ["corridor_left_minus1", "corridor_left_minus2"]:
            return "horizontal_from_left"
        elif abs_corr_position in ["corridor_right_minus1", "corridor_right_minus2"]:
            return "horizontal_from_right"
        elif abs_corr_position in ["corridor_top_minus1", "corridor_top_minus2"]:
            return "vertical_from_top"
        else:
            return "vertical_from_bottom"
        
    def rat_on_agent_dir():
        
        agent_top = obs['chars'][agent_row-1][agent_column]
        agent_bottom =  obs['chars'][agent_row-1][agent_column]
        agent_left = obs['chars'][agent_row][agent_column-1]
        agent_right = obs['chars'][agent_row][agent_column+1]

        #print_debug("\nRat on agent direction!!!\n")

        rat = ord('r')

        if (agent_top == rat):
            return "north"
        elif (agent_bottom == rat):
            return "south"
        elif (agent_left == rat):
            return "west"
        elif (agent_right == rat):
            return "east"
        else:
            return None
        



    num_tot_rats = 4
    # AGENT POSITION_____________________
    agent_row = obs['blstats'][1]
    agent_column = obs['blstats'][0]
    kb.retractall("position(agent,_,_)")
    kb.asserta(f"position(agent, {agent_row}, {agent_column})")
    positions["agent"] = agent_row, agent_column 

    agent_top = obs['chars'][agent_row-1][agent_column]
    agent_bottom =  obs['chars'][agent_row-1][agent_column]
    agent_left = obs['chars'][agent_row][agent_column -1]
    agent_right = obs['chars'][agent_row][agent_column +1]
    agent_top_left = obs['chars'][agent_row-1][agent_column-1]
    agent_top_right = obs['chars'][agent_row-1][agent_column+1]
    agent_bottom_left = obs['chars'][agent_row+1][agent_column-1]
    agent_bottom_right = obs['chars'][agent_row+1][agent_column+1]

    

    num_rats_left = list(kb.query('num_rats_left(X)'))[0]
    num_rats_left = num_rats_left['X']

    fight_state = list(kb.query('fight_state(X)'))[0]
    fight_state = fight_state['X']

    if ((num_rats_left == num_tot_rats) and (fight_state == "off") and
        ord("#") in [agent_bottom, agent_top, agent_right, agent_left]):
        kb.retractall("fight_state(_)")
        kb.asserta("fight_state(on)")
        #print_debug( "fight_state on\n")


    elif ((num_rats_left == 0) and (fight_state == "on") and
        ord(".") in [agent_bottom, agent_top, agent_right, agent_left]): 

        fight_state = "off"
        kb.retractall("fight_state(_)")
        kb.asserta("fight_state(off)")
        #print_debug( "fight_state off\n")
        

    fight_state = list(kb.query('fight_state(X)'))[0]
    fight_state = fight_state['X']


    if (fight_state == "on"):
    # HAS THE AGENT BEEN ALREADY IN THE CORRIDOR?_____________________
        try:
            already_in_corr = list(kb.query('already_in_corridor(X)'))[0]
            #print("ALREADY IN CORRIDOR VALUE: " + str(already_in_corr['X']))
            if (already_in_corr['X'] == "true_"):
                already_in_corridor = True
            elif (already_in_corr['X'] == "false_"):
                already_in_corridor = False
        except Exception as e:
            already_in_corridor = None

        #print("ALREADY IN CORRIDOR VALUE: " + str(already_in_corridor))

        # CURRENT PLACE IN WHICH THE AGENT IS_____________________
        kb.retractall("place(_)")
        #already_in_corridor = prev_place in ["corridor_start", "corridor_end", "middle_corridor"]
        abs_corr_position = get_abs_corridor_position() # nested function
        #print_debug("Abs corridor position: " + abs_corr_position + '\n')
        current_place = get_current_place() # nested function
        kb.asserta(f"place({current_place})")

        #print_debug("current_place: " + current_place + "\n")
        # print("current_place: " + current_place)

        if (current_place in ["corridor_start", "corridor_end", "middle_corridor"]):

            kb.retractall("already_in_corridor(_)")
            kb.asserta("already_in_corridor(true_)")
            already_in_corridor = True
            #print_debug("already in corridor\n")


        # CORRIDOR TYPE_____________________
        #if already_in_corridor == False:
            #print_debug("not already in corridor\n")


        if (current_place in ["corridor_start_minus1", "corridor_start_minus2"] and
            already_in_corridor == False):
            corridor_type = get_corridor_type() # nested function
            

            kb.retractall("corridor_type(_)")
            kb.asserta(f"corridor_type({corridor_type})")

        try:
            forward_dir = list(kb.query('forward_direction(X)'))[0]
            forward_dir = forward_dir['X']
            #print("FORWARD DIRECTION: "+ str(forward_dir))
            backwards_dir = list(kb.query('backwards_direction(X)'))[0]
            backwards_dir = backwards_dir['X']
        except Exception as e:
            #print(e)
            backwards_dir = None
            forward_dir = None

        if (current_place == "corridor_start"):
            kb.retractall("inertia_dir(_)")
            kb.asserta(f"inertia_dir({forward_dir})")
        elif (current_place in ["corridor_end", "corridor_end_plus1"]):
            kb.retractall("inertia_dir(_)")
            kb.asserta(f"inertia_dir({backwards_dir})")

        # NUMBER OF RATS LEFT_____________________
        try: 
            num_rats_left = list(kb.query('num_rats_left(X)'))[0]
            num_rats_left = num_rats_left['X']
            # print(f'>> Current action from Prolog: {action}')
        except Exception as e:
            num_rats_left = -1





        # RAT IN FRONT OF THE AGENT?________________
        kb.retractall("on_agent_dir(_,_)")

        direction = rat_on_agent_dir()
        if (direction != None):
            kb.asserta(f"on_agent_dir({direction}, rat)")

        # DETERMINING IF THE AGENT HAS JUST KILLED A RAT  TODO: change?   
        message = bytes(obs['message']).decode('utf-8').rstrip('\x00')
        if ('kill' in message) and ('rat' in message):       
            num_rats_left -= 1
            kb.retractall("num_rats_left(_)")
            kb.asserta(f"num_rats_left({num_rats_left})")


    else:
        for i in range(21):
            for j in range(79):
                if not (obs['screen_descriptions'][i][j] == 0).all():
                    obj = bytes(obs['screen_descriptions'][i][j]).decode('utf-8').rstrip('\x00')
                    

                    if obs["chars"][i][j] == ord('#'):
                        kb.assertz(f'in_sight(corridor)')
                        if positions["corridor_entrance"] is None:
                            positions["corridor_entrance"]=front_location(obs,i,j)
                        #if positions["corridor_entrance"] == positions["agent"]:
                            #print("next to corridor")
                            #kb.assertz(f'fight_state(on)')

                    #staircase down
                    if obs["glyphs"][i][j] == 2383:
                        kb.assertz(f'in_sight(stairs)')
                        positions["stairs"] = i, j 
                        
                    if 'key' in obj:
                        kb.assertz(f'in_sight(key)')
                        positions["key"] = i, j 

                    # if positions["door"] and positions["door"]==positions["agent"]:
                    #     kb.retractall('standing_next(door)')

                        
                    elif 'door' in obj:
                        kb.assertz(f'in_sight(door)')
                        positions["door"] = i, j
                        if positions["door_front"] is None:
                            positions["door_front"]=front_location(obs,i,j)
                        if positions["door_front"] == positions["agent"]:
                            # print("next to door")
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


def perform_action(action, env, kb,obs):
    # print(action)
    action_id=None
    if action == 'pick':
        action_id = 8
        kb.retractall('standing_on(key)')
        kb.retractall('in_sight(key)')
        
    elif action == 'opendoor':
        action_id = 9
        kb.retractall('has(key)')
        x,y = positions["agent"]
        xd,yd = positions["door"]
        dir=""
        
        if x-xd > 0:
            dir="north"
        elif x-xd <0:
            dir="south"
        elif y-yd>0:
            dir="west"
        elif y-yd<0:
            dir="east"
            
        obs, reward, done, info = env.step(action_id)
        obs, reward, done, info = env.step(actions[dir])
        message = bytes(obs['message']).decode('utf-8').rstrip('\x00')
        if "door open" in message:
            kb.assertz(f'is_open(door)')    
        
        return obs, reward, done, info

    elif near_door(obs) and 'northeast' in action:
        a1,a2=random.choice([(0,1),(1,0)])
        obs, reward, done, info = env.step(a1)
        action_id=a2
    elif near_door(obs) and 'southeast' in action:
        a1,a2=random.choice([(2,1),(1,2)])
        obs, reward, done, info = env.step(a1)
        action_id=a2
    elif near_door(obs) and 'southwest' in action:
        a1,a2=random.choice([(2,3),(3,2)])
        obs, reward, done, info = env.step(a1)
        action_id=a2
    elif near_door(obs) and 'northwest' in action:
        a1,a2=random.choice([(0,3),(3,0)])
        obs, reward, done, info = env.step(a1)
        action_id=a2
        
    # Movement/Attack/Run/Get_To_Weapon actions
    # in the end, they all are movement in a direction
    elif 'northeast' in action:
        if positions["door"] and positions["agent"] == positions["door"] and is_wall(obs,positions["agent"][0]-1,positions["agent"][1]):
            obs, reward, done, info = env.step(1)
            action_id=0
        elif positions["door"] and positions["agent"] == positions["door"] and is_wall(obs,positions["agent"][0],positions["agent"][1]-1):
            obs, reward, done, info = env.step(0)
            action_id=1
        else: action_id = 4
            
    elif 'southeast' in action: 
        if positions["door"] and positions["agent"] == positions["door"] and is_wall(obs,positions["agent"][0],positions["agent"][1]-1):
            obs, reward, done, info = env.step(2)
            action_id=1
        else: action_id = 5
        
    elif 'southwest' in action: 
        action_id = 6
    elif 'northwest' in action: 
        action_id = 7
    elif 'north' in action: action_id = 0
    elif 'east' in action: action_id = 1
    elif 'south' in action: action_id = 2
    elif 'west' in action: action_id = 3


    # print(f'Action performed: {repr(env.actions[action_id])}')
    obs, reward, done, info = env.step(action_id)
    return obs, reward, done, info


def print_debug(content):

    with open("output.txt", 'a') as file:
        file.write(content)

def show_match(states: list,x0,x1,y0,y1):
    image = plt.imshow(states[0][x0:x1, y0:y1])
    for state in states[1:]:
        time.sleep(0.29)
        display.display(plt.gcf())
        display.clear_output(wait=True)
        image.set_data(state[x0:x1, y0:y1])
    time.sleep(0.25)
    display.display(plt.gcf())
    display.clear_output(wait=True)
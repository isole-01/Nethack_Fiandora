import matplotlib.pyplot as plt
import IPython.display as display
import time
from pyswip import Prolog
from minihack import LevelGenerator



def perform_action(action, env):

    # Movement/Attack/Run/Get_To_Weapon actions
    # in the end, they all are movement in a direction
    if 'northeast' in action: action_id = 4
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

def process_state(obs: dict, kb: Prolog):
    try:
        place = list(kb.query('place(X)'))[0]
        place = place['X']
        # print(f'>> Current action from Prolog: {action}')
    except Exception as e:
        place = None

    try:
        num_rats_left = list(kb.query('num_rats_left(X)'))[0]
        num_rats_left = num_rats_left['X']
        # print(f'>> Current action from Prolog: {action}')
    except Exception as e:
        num_rats_left = -1


    kb.retractall("position(agent,_,_)")

    agent_row = obs['blstats'][1]
    agent_column = obs['blstats'][0]
    
    kb.asserta(f"position(agent, {agent_row}, {agent_column})")

    # RAT ON AGENT RIGHT?
    kb.retractall("on_agent_right(_)")

    if (obs['chars'][agent_row][agent_column+1] == ord('r')):
        kb.asserta("on_agent_right(rat)")

    # DETERMINING THE CURRENT PLACE AND THE INERTIA DIRECTIONS
    if (obs['chars'][agent_row-1][agent_column-1] == ord('|') and 
            obs['chars'][agent_row][agent_column-1] == ord('.') and 
            obs['chars'][agent_row-1][agent_column-1] == ord('|')):
        kb.retractall("place(_)")
        kb.asserta("place(corridor_start)")
        kb.retractall("inertia_dir(_)")
        kb.asserta("inertia_dir(east)")


    elif (obs['chars'][agent_row-1][agent_column+1] == ord('|') and 
            obs['chars'][agent_row][agent_column-1] == ord('#') and 
            obs['chars'][agent_row+1][agent_column+1] == ord('|')):
        kb.retractall("place(_)")
        kb.asserta("place(corridor_end)")
        kb.retractall("inertia_dir(_)")
        kb.asserta("inertia_dir(west)")

    elif (obs['chars'][agent_row-1][agent_column] == ord('|') and 
            obs['chars'][agent_row][agent_column-1] == ord('#') and 
            obs['chars'][agent_row+1][agent_column] == ord('|')):
        kb.retractall("place(_)")
        kb.asserta("place(final_room)") 

        if num_rats_left == 6: #(if the fight hasn't already started)
            kb.retractall("inertia_dir(_)")
            kb.asserta("inertia_dir(west)")

    elif (place == 'corridor_start') or (place == 'corridor_end'): # (that's actually the previous place)
        kb.retractall("place(_)")
        kb.asserta("place(corridor_middle)") 


    # DETERMINING IF THE AGENT HAS JUST KILLED A RAT     
    message = bytes(obs['message']).decode('utf-8').rstrip('\x00')
    if ('kill' in message) and ('rat' in message):       
        num_rats_left -= 1
        kb.retractall("num_rats_left(_)")
        kb.asserta(f"num_rats_left({num_rats_left})")


# indexes for showing the image are hard-coded
def show_match(states: list, messages: list):
    plt.figure(figsize=(20, 16))
    image = plt.imshow(states[0][140:225, 345:910])

    i=1
    for state in states[1:]:


        time.sleep(1.0)  #CNG: originally 0.25

        print(messages[i])
        i+=1

        display.display(plt.gcf())
        display.clear_output(wait=True)
        image.set_data(state[140:225, 345:910])
    time.sleep(0.25)
    display.display(plt.gcf())
    display.clear_output(wait=True)



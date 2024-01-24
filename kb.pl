:- dynamic standing_next/1.
:- dynamic has/1.
:- dynamic standing_on/1.
:- dynamic in_sight/1.
:- dynamic enter/1.
:- dynamic is_open/1.





:- dynamic position/3.
:- dynamic place/1.

:- dynamic corridor_type/1.    %(horizontal_from_left, horizontal_from_right,  % vertical_from_top, vertical_from_bottom)
:- dynamic num_rats_left/1.
:- dynamic on_agent_dir/2.
:- dynamic inertia_dir/1.

:- dynamic already_in_corridor/1.

:- dynamic last_extreme_rel/1.
:- dynamic last_extreme_abs/1.

:- dynamic fight_state/1. %(on/off)
                            % off: before the agent enters the corridor and 
                            % again when all the rats are dead and the agent is outside 
                            % of the corridor.

% RULES FOR ACTION SELECTION
% REMEMBER: The action rules are ordered by priority
action(move_to_corridor(Direction)) :- fight_state(on), already_in_corridor(false_),
                                    forward_direction(Direction).

action(get_off_corridor(Direction)) :- 
                            num_rats_left(N), N==0,
                            forward_direction(Direction),
                            fight_state(on).

% attack a rat
% the attack is automatic when you move towards it
action(attack(Direction)) :- on_agent_dir(Direction, rat), fight_state(on).

action(move(Direction)):- place(corridor_start), forward_direction(Direction), fight_state(on).

action(move(Direction)) :- (place(corridor_end_plus1); place(corridor_end)),
                                        num_rats_left(N), N=\=0,
                                        backwards_direction(Direction),
                                        fight_state(on).
                                        
action(move(Direction)) :- inertia_dir(Direction), fight_state(on).

action(go(stairs)) :- in_sight(stairs), is_open(door).
action(enter(door)) :- is_open(door), standing_next(door).
action(opendoor) :- standing_next(door), has(key), \+ is_open(door).
action(pick) :- standing_on(key), \+ has(key).


action(go(corridor)) :- in_sight(corridor), fight_state(off), num_rats_left(N), N==4.

action(explore) :- fight_state(off), num_rats_left(N), N==4.

% If you dont have the key and know where the key is, action: go to the key
action(go(key)) :- \+ has(key), in_sight(key).

% If you know where the door is and you have the key, action: go to the door
action(go(door)) :- in_sight(door), has(key), \+ is_open(door).

% If you dont have the key and you dont know where the key is, or where the door is, action: explore
action(explore) :- (\+ has(key), \+ in_sight(key)) ; (has(key), \+ in_sight(door)); is_open(door) .




% -----------------------------------------------------------------------------------------------
forward_direction(east) :- corridor_type(horizontal_from_left).
forward_direction(south) :- corridor_type(vertical_from_top).
forward_direction(west) :- corridor_type(horizontal_from_right).
forward_direction(north) :- corridor_type(vertical_from_bottom).

backwards_direction(south) :- corridor_type(vertical_from_bottom).
backwards_direction(north) :- corridor_type(vertical_from_top).
backwards_direction(west) :- corridor_type(horizontal_from_left).
backwards_direction(east) :- corridor_type(horizontal_from_right).

% -----------------------------------------------------------------------------------------------
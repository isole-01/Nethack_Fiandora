% place(...)
% possible values for place: 
% 1.initial_room, 2.corridor_start (first #), 
% 3.middle_corridor 
% 4.corridor_end (last #)  5. final_room

% Temp_target 

% fight(...)
% possible values for fight
% 1. not_started, 2.ongoing, 3.ended (6 rats have been killed)

% num_rats_left(...)

% (there are always 6 rats)

% if place(initial_room) -> we should move towards the corridor start
% (we should have a "function" that, given a temporary target)

% if (place(corridor_start) and corridor on our right)  ->
% we should move towards the corridor end

% Once reached the corridor_exit, we invert the direction 

%Taking into account the number of rats 

:- dynamic position/3.
:- dynamic place/1.

:- dynamic corridor_type/1.    %(horizontal_from_left, horizontal_from_right,  % vertical_from_top, vertical_from_bottom)
:- dynamic num_rats_left/1.
:- dynamic on_agent_dir/2.
:- dynamic inertia_dir/1.

:- dynamic already_in_corridor/1.

:- dynamic last_extreme_rel/1.
:- dynamic last_extreme_abs/1.

% RULES FOR ACTION SELECTION
% REMEMBER: The action rules are ordered by priority

action(get_to_corridor_start(Direction)) :- (place(not_in_corridor); place(corridor_start_minus1)),
                                    position(agent, AgentR, AgentC),
                                    position(corridor_start, CorrStartR, CorrStartC),
                                    next_step(AgentR, AgentC, CorrStartR, CorrStartC, Direction),
                                    num_rats_left(N), N==6.

%TODO: change: not generical

action(get_to_final_target(Direction)) :- %TODO: change?
                            num_rats_left(N), N==0,
                            foreward_direction(Direction).

% attack a rat
% the attack is automatic when you move towards it

action(attack(Direction)) :- on_agent_dir(Direction, rat).

action(move(Direction)):- place(corridor_start), foreward_direction(Direction).

action(move(Direction)) :- (place(corridor_end_plus1); place(corridor_end)),
                                        num_rats_left(N), N=\=0,
                                        backwards_direction(Direction). % TODO: substitute condition; 
                                        % \+(N==0) is the correct notation?

action(move(Direction)) :- inertia_dir(Direction).

% -----------------------------------------------------------------------------------------------
foreward_direction(east) :- corridor_type(horizontal_from_left).
foreward_direction(south) :- corridor_type(vertical_from_top).
foreward_direction(west) :- corridor_type(horizontal_from_right).
foreward_direction(north) :- corridor_type(vertical_from_bottom).

backwards_direction(south) :- corridor_type(vertical_from_bottom).
backwards_direction(north) :- corridor_type(vertical_from_top).
backwards_direction(west) :- corridor_type(horizontal_from_left).
backwards_direction(east) :- corridor_type(horizontal_from_right).

% -----------------------------------------------------------------------------------------------


% compute the direction given the starting point and the target position
next_step(R1,C1,R2,C2, D) :-
    ( R1 == R2 -> ( C1 > C2 -> D = west; D = east );
    ( C1 == C2 -> ( R1 > R2 -> D = north; D = south);
    ( R1 > R2 ->
        ( C1 > C2 -> D = northwest; D = northeast );
        ( C1 > C2 -> D = southwest; D = southeast )
    ))).

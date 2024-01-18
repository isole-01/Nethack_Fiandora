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
%

%Taking into account the number of rats 


:- dynamic position/3.
:- dynamic place/1.
:- dynamic num_rats_left/1
:- dynamic on_agent_right/1
:- dynamic inertia_dir/1

% RULES FOR ACTION SELECTION
% REMEMBER: The action rules are ordered by priority

action(get_to_corridor_start(Direction)) :- place(initial_room),
                                    position(agent, AgentR, AgentC),
                                    position(corridor_start, CorrStartR, CorrStartC),
                                    next_step(AgentR, AgentC, CorrStartR, CorrStartC, Direction).


action(get_to_final_target(east)) :- num_rats_left(N), N==0.

% attack a rat
% the attack is automatic when you move towards it
% Requirements:
%   - there must be a rat on the right of the agent 
%       (since, in order to win, the fight has to take place in the corridor,
%       the rats are always at the right of the agent)
action(attack(east)) :- on_agent_right(rat).

action(move(east)):- place(corridor_start).
%inertia_direction(east) :- place(corridor_start).

action(move(west)) :- place(final_room); place(corridor_end).
%inertia_direction(west) :- ...

action(move(Direction)) :- inertia_dir(Direction).

% -----------------------------------------------------------------------------------------------


% compute the direction given the starting point and the target position
next_step(R1,C1,R2,C2, D) :-
    ( R1 == R2 -> ( C1 > C2 -> D = west; D = east );
    ( C1 == C2 -> ( R1 > R2 -> D = north; D = south);
    ( R1 > R2 ->
        ( C1 > C2 -> D = northwest; D = northeast );
        ( C1 > C2 -> D = southwest; D = southeast )
    ))).


%has(agent, _, _) :- fail.

%unsafe_position(_,_) :- fail.
%safe_position(R,C) :- \+ unsafe_position(R,C).

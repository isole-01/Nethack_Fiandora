:- dynamic standing_next/1.
:- dynamic has/1.
:- dynamic standing_on/1.
:- dynamic in_sight/1.
:- dynamic enter/1.
:- dynamic is_open/1.

action(go(stairs)) :- in_sight(stairs), is_open(door).
action(enter(door)) :- is_open(door), standing_next(door).
action(opendoor) :- standing_next(door), has(key), \+ is_open(door).
action(pick) :- standing_on(key), \+ has(key).

% If you dont have the key and know where the key is, action: go to the key
action(go(key)) :- \+ has(key), in_sight(key).

% If you know where the door is and you have the key, action: go to the door
action(go(door)) :- in_sight(door), has(key), \+ is_open(door).

% If you dont have the key and you dont know where the key is, or where the door is, action: explore
action(explore) :- (\+ has(key), \+ in_sight(key)) ; (has(key), \+ in_sight(door)); is_open(door) .



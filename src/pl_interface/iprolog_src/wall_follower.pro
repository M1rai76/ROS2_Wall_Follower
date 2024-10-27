front is 0.
front_left is 1.
left_front is 2.
front_right is 11.

wall_follower :-
	near_start -> drive(0.0, 0.0);
	scan(left_front) > 0.9	-> drive(0.2, 1.5);
	scan(front) < 0.7	-> drive(0.0, -1.5);
	scan(front_left) < 0.6	-> drive(0.3, -1.5);
	scan(front_right) < 0.6	-> drive(0.3, 1.5);
	scan(left_front) > 0.6	-> drive(0.3, 1.5);
	drive(0.3, 0.0).

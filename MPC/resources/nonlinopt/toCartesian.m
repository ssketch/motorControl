function hand = toCartesian( states, params )

% Position in cartesian space
elbowx = params.L1 .* cos( states(1));
elbowy = params.L1 .* sin( states(2));

handx = elbowx + params.L2 .* cos( states(1) + states(2));
handy = elbowy + params.L2 .* sin( states(1) + states(2));

handpos = [handx; handy ];


% Velocity in Cartesian space
elbowx = -params.L1 .* states(3) .* sin( states(1));
elbowy = params.L1 .* states(4) .* cos( states(2));

handx = elbowx - params.L2 .* (states(3) .* sin( states(1) + states(2)) + ...
    states(4) .* sin( states(1) + states(2)));
handy = elbowy + params.L2 .* (states(3) .* cos( states(1) + states(2)) + ...
    states(4) .* cos( states(1) + states(2)));

handvel = [ handx; handy ];

hand = [ handpos; handvel ];
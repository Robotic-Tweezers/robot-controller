theta = zeros(3, 1);
expected_frame = eye(3);
expected_origin = [0, 0, parameters('LENGTH1') + parameters('LENGTH2')]';
[frame, origin] = directKinematics(theta, parameters);
assert(isequal(frame, expected_frame));
assert(isequal(origin, expected_origin));

theta = [pi/2, 0, 0];
expected_frame = [
    [0, -1, 0];
    [1, 0, 0];
    [0, 0, 1];
];
expected_origin = [0, 0, parameters('LENGTH1') + parameters('LENGTH2')]';
[frame, origin] = directKinematics(theta, parameters);
assert(isequal(frame, expected_frame));
assert(isequal(origin, expected_origin));

theta = [0, pi/2, 0];
expected_frame = [
    [0, 0, -1];
    [0, 1, 0];
    [1, 0, 0];
];
expected_origin = [-parameters('LENGTH2'), 0, parameters('LENGTH1')]';
[frame, origin] = directKinematics(theta, parameters);
assert(isequal(frame, expected_frame));
assert(isequal(origin, expected_origin));

theta = [0, 0, pi/2];
expected_frame = [
    [0, -1, 0];
    [1, 0, 0];
    [0, 0, 1];
];
expected_origin = [0, 0, parameters('LENGTH1') + parameters('LENGTH2')]';
[frame, origin] = directKinematics(theta, parameters);
assert(isequal(frame, expected_frame));
assert(isequal(origin, expected_origin));

clear theta;
clear expected_frame;
clear expected_origin;
clear frame;
clear origin;

fprintf("PASS\n");
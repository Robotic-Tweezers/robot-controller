theta = zeros(3, 1);
expected_frame = eye(3);
expected_origin = [0, 0, 25]';
[frame, origin] = direct_kinematics(theta, parameters);
assert(isequal(frame, expected_frame));
assert(isequal(origin, expected_origin));

theta = [pi/2, 0, 0];
expected_frame = [
    [0, -1, 0];
    [1, 0, 0];
    [0, 0, 1];
];
expected_origin = [0, 0, 25]';
[frame, origin] = direct_kinematics(theta, parameters);
assert(isequal(frame, expected_frame));
assert(isequal(origin, expected_origin));

theta = [0, pi/2, 0];
expected_frame = [
    [0, 0, -1];
    [0, 1, 0];
    [1, 0, 0];
];
expected_origin = [-15, 0, 10]';
[frame, origin] = direct_kinematics(theta, parameters);
assert(isequal(frame, expected_frame));
assert(isequal(origin, expected_origin));

theta = [0, 0, pi/2];
expected_frame = [
    [0, -1, 0];
    [1, 0, 0];
    [0, 0, 1];
];
expected_origin = [0, 0, 25]';
[frame, origin] = direct_kinematics(theta, parameters);
assert(isequal(frame, expected_frame));
assert(isequal(origin, expected_origin));

fprintf("PASS\n");
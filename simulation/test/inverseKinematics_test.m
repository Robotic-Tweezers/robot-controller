% Home position
frame03 = [
    [1, 0, 0];
    [0, 1, 0];
    [0, 0, 1]
];
[theta] = inverseKinematics(frame03, pi / 2);
expected = [
    [0, 0, 0];
    [0, 0, 0]
];
assert(isequal(theta, expected));

% +90 deg rotation about i 0
frame03 = [
    [1, 0, 0];
    [0, 0, -1];
    [0, 1, 0]
];
[theta] = inverseKinematics(frame03, pi / 2);
expected = [
    [pi / 2, pi / 2, -pi / 2];
    [-pi / 2, -pi / 2, pi / 2]
];
assert(isequal(round(theta, 4), round(expected, 4)));

% 180 deg rotation about j_0 (singularity)
frame03 = [
    [-1, 0, 0];
    [0, 1, 0];
    [0, 0, -1]
];
[theta] = inverseKinematics(frame03, pi / 2);
expected = [
    [0, pi, 0];
    [0, pi, 0]
];
assert(isequal(theta, expected));

% 45 deg rotations of theta2 only
frame03 = [[1, 0, 1]./sqrt(2);
        [0, 1, 0];
        [-1, 0, 1]./sqrt(2)];
[theta] = inverseKinematics(frame03, pi / 2);
expected = [
    [pi, pi / 4, pi];
    [0, -pi / 4, 0]
];
assert(isequal(round(theta, 4), round(expected, 4)));

clear theta;
clear expected;
clear frame03;

fprintf("PASS\n");
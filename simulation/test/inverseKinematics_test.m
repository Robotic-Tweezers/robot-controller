% Home position
frame3 = [
    [1, 0, 0];
    [0, 1, 0];
    [0, 0, 1]
];
[theta] = inverseKinematics(frame3, pi / 2);
expected = [
    [pi, pi, 0];
    [pi, pi, 0]
];
assert(isequal(round(theta, 6), round(expected, 6)));

% +90 deg rotation about i 0
frame3 = [
    [1, 0, 0];
    [0, 0, -1];
    [0, 1, 0]
];
[theta] = inverseKinematics(frame3, pi / 2);
expected = [
    [-pi / 2, pi / 2, pi / 2];
    [pi / 2, -pi / 2, -pi / 2]
];
assert(isequal(round(theta, 4), round(expected, 4)));

% 180 deg rotation about j_0 (singularity)
frame3 = [
    [-1, 0, 0];
    [0, 1, 0];
    [0, 0, -1]
];
[theta] = inverseKinematics(frame3, pi / 2);
expected = [
    [pi, 0, 0];
    [pi, 0, 0]
];
assert(isequal(round(theta, 4), round(expected, 4)));

% 45 deg rotations of theta2 only
frame3 = [[1, 0, 1]./sqrt(2);
        [0, 1, 0];
        [-1, 0, 1]./sqrt(2)];
[theta] = inverseKinematics(frame3, pi / 2);
expected = [
    [pi, 3 * pi / 4, 0];
    [0, -3 * pi / 4, -pi]
];
assert(isequal(round(theta, 4), round(expected, 4)));

clear theta;
clear expected;
clear frame3;

fprintf("PASS\n");
theta = zeros(3, 1);
base_frame = round(expm(pi * skew3([1 0 0]')), 6);
expected_frame = base_frame * eye(3);
expected_origin = base_frame * [0, 0, length1 + length2]';
[frame, origin] = directKinematics(theta);
assert(isequal(round(frame, 4), round(expected_frame, 4)));
assert(isequal(round(origin, 4), round(expected_origin, 4)));

theta = [pi/2, 0, 0];
expected_frame = base_frame * [
    [0, -1, 0];
    [1, 0, 0];
    [0, 0, 1];
];
expected_origin = base_frame * [0, 0, length1 + length2]';
[frame, origin] = directKinematics(theta);
assert(isequal(round(frame, 4), round(expected_frame, 4)));
assert(isequal(round(origin, 4), round(expected_origin, 4)));

theta = [0, pi/2, 0];
expected_frame = base_frame * [
    [0, 0, -1];
    [0, 1, 0];
    [1, 0, 0];
];
expected_origin = base_frame * [-length2, 0, length1]';
[frame, origin] = directKinematics(theta);
assert(isequal(round(frame, 4), round(expected_frame, 4)));
assert(isequal(round(origin, 4), round(expected_origin, 4)));

theta = [0, 0, pi/2];
expected_frame = base_frame * [
    [0, -1, 0];
    [1, 0, 0];
    [0, 0, 1];
];
expected_origin = base_frame * [0, 0, length1 + length2]';
[frame, origin] = directKinematics(theta);
assert(isequal(round(frame, 4), round(expected_frame, 4)));
assert(isequal(round(origin, 4), round(expected_origin, 4)));

clear theta;
clear expected_frame;
clear expected_origin;
clear frame;
clear base_frame;
clear origin;

fprintf("PASS\n");
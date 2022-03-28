joint_positions = [
    [0, 0, 0];
    [0, pi / 12, 0];
    [0, 2 * pi / 12, 0];
    [0, 3 * pi / 12, 0];
    [0, 4 * pi / 12, 0];
    [0, 5 * pi / 12, 0];
    [0, 6 * pi / 12, 0];
    [pi / 12, 6*pi / 12, 0];
    [2 * pi / 12, 6 * pi / 12, 0];
    [3 * pi / 12, 6 * pi / 12, 0];
    [4 * pi / 12, 6 * pi / 12, 0];
    [5 * pi / 12, 6 * pi / 12, 0];
    [6 * pi / 12, 6 * pi / 12, 0];
    [7 * pi / 12, 6 * pi / 12, 0];
    [8 * pi / 12, 6 * pi / 12, 0];
    [9 * pi / 12, 6 * pi / 12, 0];
    [10 * pi / 12, 6 * pi / 12, 0];
    [11 * pi / 12, 6 * pi / 12, 0];
    [pi, 6 * pi / 12, 0];
    [pi, 5 * pi / 12, 0];
    [pi, 4 * pi / 12, 0];
    [pi, 3 * pi / 12, 0];
    [pi, 2 * pi / 12, 0];
    [pi, 1 * pi / 12, 0];
    [pi, 0, 0];
];

euler_angles = zeros(size(joint_positions, 1), 3);

theta0 = zeros(size(joint_positions, 1), 1);
theta0_ = zeros(size(joint_positions, 1), 1);
theta1 = zeros(size(joint_positions, 1), 1);
theta1_ = zeros(size(joint_positions, 1), 1);
theta2 = zeros(size(joint_positions, 1), 1);
theta2_ = zeros(size(joint_positions, 1), 1);

for i=1:size(joint_positions, 1)
    [frame, origin] = directKinematics(joint_positions(i, :));
    coordinates = [frame origin; zeros(1, 3) 1];
    euler_angles(i, :) = rotm2eul(frame, 'XYZ');
    solutions = inverseKinematics(eul2rotm(euler_angles(i, :), 'XYZ'), pi / 2);
    theta0(i, 1) = solutions(1, 1);
    theta0_(i, 1) = solutions(2, 1);
    theta1(i, 1) = solutions(1, 2);
    theta1_(i, 1) = solutions(2, 2);
    theta2(i, 1) = solutions(1, 3);
    theta2_(i, 1) = solutions(2, 3);
end

disp(euler_angles);
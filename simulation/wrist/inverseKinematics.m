function [theta] = inverseKinematics(end_effector_frame, psi)
%INVERSE_KINEMATICS Calculates all possible joint angles (theta) achieving a desired
%orientation
%   Using Kahan Subproblems and the Denavit-Hartenberg parameters,
%   determines both possible configurations achieving a desired orientation
%   if it exists and returns Nan otherwise.

% Base frame and desired frame
frame0 = round(expm(pi * skew3([1 0 0]')), 6);
frame3 = end_effector_frame;

% i and k unit vectors in frame 0 and 3
i_unit0 = frame0 * [1, 0, 0]';
k_unit0 = frame0 * [0, 0, 1]';
i_unit3 = frame3 * [1, 0, 0]';
k_unit3 = frame3 * [0, 0, 1]';
theta = zeros(2, 3);

% Catch conditions where one solution exists (singularities)
% Occures when frame 1 and 3 have alligned k axes
if (round(skew3(k_unit0) * k_unit3, 6) == 0)
    % determine if theta 2 should be zero or 180 degrees
    if (round(k_unit0' * k_unit3, 3) ~= 1)
        theta(1, 2) = pi;
    end
    % if k 0 and k 3 are in the same direction, we don't need to rotate
    % theta 3
    theta(1, 3) = 0;
    v = expm(psi * skew3(i_unit3)) * k_unit3;
    u = expm(psi * skew3(i_unit0)) * k_unit0;
    theta(1, 1) = kahanProblem2(k_unit0, u, v);
    % Copying the single solution into a second row to ensure the theta
    % matrix is always the same size.
    theta(2, :) = theta(1, :);
    return
end

v = expm(psi * skew3(i_unit3)) * k_unit3;
u = expm(psi * skew3(i_unit0)) * k_unit0;

% Will return two possible solutions for theta 1 and 3
[theta1, theta3] = kahanProblem3(k_unit0, k_unit3, u, v);

theta2 = zeros(2,1);
theta3 = -theta3;

for i=1:size(theta1, 1)
    frame2 = frame3 * expm(-theta3(i) * skew3([0,0,1]'));
    frame1 = frame0 * expm(theta1(i) * skew3([0,0,1]')) * expm(psi * skew3([1,0,0]'));

    i_unit1 = frame1 * [1,0,0]';
    k_unit1 = frame1 * [0,0,1]';
    i_unit2 = frame2 * [1,0,0]';

    theta2(i) = kahanProblem2(k_unit1, i_unit1, i_unit2);
end

theta = [theta1 theta2 theta3];

end


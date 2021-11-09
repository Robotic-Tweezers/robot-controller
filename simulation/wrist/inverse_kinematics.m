function [theta] = inverse_kinematics(end_effector_frame)
%INVERSE_KINEMATICS Calculates all possible thetas achieving a desired
%orientation
%   Detailed explanation goes here

% Base frame and desired frame
frame0 = eye(3); 
frame3 = frame0 * end_effector_frame; 

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
    v = expm((pi / 2) * skew3(i_unit3)) * k_unit3;
    u = expm((pi / 2) * skew3(i_unit0)) * k_unit0;
    theta(1, 1) = kahan_problem2(k_unit0, u, v);
    % Copying the single solution into a second row to ensure the theta
    % matrix is always the same size.
    theta(2, :) = theta(1, :);
    return 
end

v = expm((pi / 2) * skew3(i_unit3)) * k_unit3;
u = expm((pi / 2) * skew3(i_unit0)) * k_unit0;

% Will return two possible solutions for theta 1 and 3
[theta1, theta3] = kahan_problem3(k_unit0, k_unit3, u, v);

theta2 = zeros(2,1);
theta3 = -theta3;

for i=1:size(theta1, 1)
    frame2 = frame3 * expm(-theta3(i) * skew3([0,0,1]')); 
    frame1 = frame0 * expm(theta1(i) * skew3([0,0,1]')) * expm((pi / 2) * skew3([1,0,0]'));

    i_unit1 = frame1 * [1,0,0]';
    k_unit1 = frame1 * [0,0,1]'; 
    i_unit2 = frame2 * [1,0,0]';

    theta2(i) = kahan_problem2(k_unit1, i_unit1, i_unit2);
end

theta = [theta1 theta2 theta3];

end


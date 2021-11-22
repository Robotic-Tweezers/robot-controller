function [jacobian] = jacobian(theta)
%JACOBIAN Calculates the Jacobian matrix corresponding to the current state
%   Jacobian is a matrix in robotics which provides the relation between 
%   joint velocities and end-effector velocities of a robot manipulator.

dh_parameters = denavitHartenbergTable(theta);
end_effector = [
    expm(pi * skew3([1 0 0]')) zeros(3, 1);
    zeros(1, 3) 1;
];
jacobian = zeros(6, 3);
origins = zeros(4, 3);
k_axes = zeros(4, 3);
k_axes(1, 3) = -1;

for i=1:size(dh_parameters, 1)
    d = dh_parameters(i, 2);
    a = dh_parameters(i, 3);
    alpha = dh_parameters(i, 4);
    end_effector = end_effector * denavitHartenbergTransform(theta(i), d, a, alpha);
    origins(i+1, :) = end_effector(1:3, 4);
    k_axes(i+1, :) = end_effector(1:3, 3);
end

for i = 1:size(dh_parameters, 1)
    jacobian(1:3, i) = skew3(k_axes(i, :)') * (origins(4, :)' - origins(i, :)');
    jacobian(4:6, i) = k_axes(i, :);
end

end

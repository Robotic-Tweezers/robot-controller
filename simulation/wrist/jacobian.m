function [jacobian] = jacobian(theta, parameters)
%JACOBIAN Calculates the Jacobian matrix corresponding to the current state
%   Jacobian is a matrix in robotics which provides the relation between 
%   joint velocities and end-effector velocities of a robot manipulator.

dh_parameters = [
    [theta(1), parameters('LENGTH1'), 0, pi / 2 ];
    [theta(2), 0,                     0, -pi / 2];
    [theta(3), parameters('LENGTH2'), 0, 0      ];
];
end_effector = eye(4);
jacobian = zeros(6, 3);
origins = zeros(4, 3);
k_axes = zeros(4, 3);
k_axes(1, 3) = 1; 

for i=1:size(dh_parameters, 1)
    d = dh_parameters(i, 2);
    a = dh_parameters(i, 3);
    alpha = dh_parameters(i, 4);
    end_effector = end_effector * denavit_hartenberg_transform(theta(i), d, a, alpha);
    origins(i+1, :) = end_effector(1:3, 4);
    k_axes(i+1, :) = end_effector(1:3, 3);
end

for i = 1:size(dh_parameters, 1)
    jacobian(1:3, i) = round(skew3(k_axes(i, :)') * (origins(4, :)' - origins(i, :)'), 6);
    jacobian(4:6, i) = round(k_axes(i, :), 6);
end

end

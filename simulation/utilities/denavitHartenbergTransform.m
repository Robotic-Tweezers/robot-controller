function [transform] = denavitHartenbergTransform(theta, d, a, alpha)
%DENAVIT_HARTENBERG_PARAM Accepts the D-H parameters between two frames 
%and outputs the homogenous transform matrix
%   This function relates frames i and i-1 links by creating a homogenous
%   transform matrix consisting of an angular offset about k (theta), and
%   translational offset about k (d), a distance seperation about i (a) 
%   and a twist motion about i (alpha) not to confuse with imaginary unit

i_unit = [1 0 0]';
k_unit = [0 0 1]';

% Calulate using the product of the angular and traslation matricies
% corresponding to each parameter
z_translation = [
    eye(3)      d * k_unit;
    zeros(1, 3) 1
];
z_rotation = [
    expm(theta * skew3(k_unit)) zeros(3, 1);
    zeros(1, 3)                 1
];
i_translation = [
    eye(3)      a * i_unit;
    zeros(1, 3) 1
];
i_rotation = [
    expm(alpha * skew3(i_unit)) zeros(3, 1);
    zeros(1, 3)                 1
];

transform = z_translation * z_rotation * i_translation * i_rotation;

end


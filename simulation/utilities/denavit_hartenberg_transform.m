function [transform] = denavit_hartenberg_transform(theta, d, a, alpha)
%DENAVIT_HARTENBERG_PARAM Accepts the D-H parameters between two frames 
%and outputs the homogenous transform matrix
%   This function relates frames i and i-1 links by creating a homogenous
%   transform matrix consisting of an angular offset about k (theta), and
%   translational offset about k (d), a distance seperation about i (a) 
%   and a twist motion about i (alpha) 
% not to confuse with imaginary unit

i_unit = [1, 0, 0]';
k_unit = [0, 0, 1]';
zero3 = zeros(3, 1);

% Calulate using the product of the angular and traslation matricies
% corresponding to each parameter
angle = [
    [expm(theta * skew3(k_unit)), zero3];
    [zero3',                         1]
];
offset = [
    [eye(3), d * k_unit];
    [zero3',          1]
];
length = [
    [eye(3), a * i_unit];
    [zero3',          1]
];
twist = [
    [expm(alpha * skew3(i_unit)), zero3];
    [zero3',                         1]
];

transform = angle * offset * length * twist;

end


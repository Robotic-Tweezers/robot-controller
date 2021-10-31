function [transform] = denavit_hartenberg_param(params)
%DENAVIT_HARTENBERG_PARAM Accepts the D-H parameters between two frames 
%and outputs the homogenous transform matrix
%   This function relates frames i and i-1 but creating a homogenous
%   transform matrix consisting of an angular offset about k (theta), and
%   offset about k (d), a distance seperation about i (a) and a twist
%   motion about i (alpha) 
% not to confuse with imaginary unit
theta = params(1);
d = params(2);
a = params(3);
alpha = params(4);
i_ = [1, 0, 0]';
k = [0, 0, 1]';
z3 = zeros(3, 1);

% Calulate using the product of the angular and traslation matricies
% corresponding to each parameter
angle = [
    [expm(theta * skew(k)), z3];
    [z3',                   1 ]
]; 
offset = [
    [eye(3), d * k];
    [z3',    1]
];
length = [
    [eye(3), a * i_];
    [z3',         1]
];
twist = [
    [expm(alpha * skew(i_)), z3];
    [z3',                   1 ]
]; 
transform = angle * offset * length * twist;
end


function [theta] = kahanProblem1(s, t)
%KAHAN_PROBLEM1 Rotation about a single axis
%   Finds the angle needed to rotate the vector s, onto the vector t about
%   the vector normal to s and t
u = unit(s) - unit(t);
v = unit(s) + unit(t);
theta = round(2 * atan(norm(u) / norm(v)), 6);

end
function [theta] = kahanProblem4(a, b, c)
%KAHAN_PROBLEM4 Finds the supplementary angle to the one formed by legs a and b
%   Given a triangle with sides a, b and c, finds the compliment of the
%   angle between a and b.
theta = NaN;

if a + b >= c && c >= norm(a - b)
    y_comp = sqrt((a + b) ^ 2 - c ^ 2);
    x_comp = sqrt(c ^ 2 - (a - b) ^ 2);
    theta = round(2 * atan(y_comp / x_comp), 6);
end

end


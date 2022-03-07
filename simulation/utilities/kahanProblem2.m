function [theta] = kahanProblem2(s_unit, u, v)
%KAHAN_PROBLEM2 Rotation about a specified axis to rotate u onto v
%   given a unit vector s (s_unit) find the angle needed to rotate u about
%   s to achieve v
u_unit = unit(u);
v_unit = unit(v);

% If the inner products of u, v and s are not equal, the solution does not
% exist
if round(s_unit' * u_unit, 2) ~= round(s_unit' * v_unit, 2)
    theta = NaN;
    return
end

% temporary calculations
q = skew3(s_unit) * (u_unit - v_unit);
r = skew3(s_unit) * (u_unit + v_unit); 

% Sign of the below equation determines the sign of theta
if (v_unit' * (skew3(s_unit) * (u_unit - v_unit)) < 0)
    theta = round(-2 * atan(norm(q) / norm(r)), 6);
else
    theta = round(2 * atan(norm(q) / norm(r)), 6);
end

end


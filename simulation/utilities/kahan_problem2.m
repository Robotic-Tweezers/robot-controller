function [theta] = kahan_problem2(s_unit, u, v)
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


theta = 2 * atan(norm(skew(s_unit) * (u_unit - v_unit)) / ...
                 norm(skew(s_unit) * (u_unit + v_unit)));

% Sign of the below equation determines the sign of theta
if (v_unit' * (skew(s_unit) * (u_unit - v_unit)) < 0) 
    theta = -theta;
end

theta = round(theta, 6);

end


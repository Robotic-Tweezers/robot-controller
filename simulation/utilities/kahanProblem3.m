function [theta, phi] = kahanProblem3(s_unit, t_unit, u, v)
%KAHAN_PROBLEM3 Finds the angles needed to rotate u around s, v around t to
%achieve the same vector. 
%   Given the equation: 
%   expm(theta * skew3(s_unit)) * u == expm(phi * skew3(t_unit)) * v
%   Finds the angles theta and phi to achieve a common vector (if it
%   exists)

% catch corner case
if (isequal(s_unit, t_unit) && isequal(u, v))
   theta = [0; 0];
   phi = [0; 0];
   return
end

% temporary calculations
st_cross = skew3(s_unit) * t_unit;
st_dot = s_unit' * t_unit;
us_dot = unit(u)' * s_unit;
vt_dot = unit(v)' * t_unit;

alpha = (us_dot - st_dot * vt_dot) / norm(st_cross) ^ 2;
beta = (vt_dot - st_dot * us_dot) / norm(st_cross) ^ 2;

z = alpha * s_unit + beta * t_unit;

if (1 - norm(z) ^ 2 < 0)
    theta = NaN;
    phi = NaN;
    return
end
    
w_unit1 = z + (sqrt(1 - norm(z) ^ 2) * st_cross / norm(st_cross));
w_unit2 = z - (sqrt(1 - norm(z) ^ 2) * st_cross / norm(st_cross));

theta = round([ ...
    kahanProblem2(s_unit, unit(u), w_unit1); ...
    kahanProblem2(s_unit, unit(u), w_unit2); ...
], 6);
phi = round([ ...
    kahanProblem2(t_unit, unit(v), w_unit1); ...
    kahanProblem2(t_unit, unit(v), w_unit2); ...
], 6);

end


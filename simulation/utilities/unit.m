function [unit_vector] = unit(vector)
%NORMAL Calculates the unit vector in the direction of the input vector
%   Computes the unit vector in the direction of vector by dividing the
%   vector by its two-norm
unit_vector = vector ./ norm(vector);
end


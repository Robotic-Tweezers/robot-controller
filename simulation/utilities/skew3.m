function [skew_symmetric_vector] = skew3(vector)
%SKEW Perform the skew operator on a 3x1 vector
%   Creates a skew symmetric matrix using the elements of the input vector.
%   Used heavily in inverse and direct kinematics for wrist

v1 = vector(1);
v2 = vector(2);
v3 = vector(3);

skew_symmetric_vector = [
    0   -v3 v2;
    v3  0   -v1;
    -v2 v1  0
];

end
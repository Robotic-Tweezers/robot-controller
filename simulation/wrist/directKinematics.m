function [end_eff_frame, end_eff_origin] = directKinematics(theta)
%DIRECT_KINEMATICS Computes the 6 DOF position of a spherical wrist end-effector given
%joint states.
%   Used the Denavit Hartenberg convention to calculate the coordinate
%   frame of each consecutive joint and finally the end effector for a
%   spherical wrist.

dh_parameters = denavitHartenbergTable(theta);
end_effector = [
    expm(pi * skew3([1 0 0]')) zeros(3, 1);
    zeros(1, 3) 1;
];

for i=1:size(dh_parameters, 1)
    d = dh_parameters(i, 2);
    a = dh_parameters(i, 3);
    alpha = dh_parameters(i, 4);
    end_effector = end_effector * denavitHartenbergTransform(theta(i), d, a, alpha);
end

end_eff_frame = end_effector(1:3, 1:3);
end_eff_origin = end_effector(1:3, 4);

end

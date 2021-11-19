function [robot] = create_robot()
%CREATE_ROBOT Create a robot object used in controls analysis
%   Detailed explanation goes here

robot = importrobot("models\wrist.sdf", 'DataFormat', 'column');
showdetails(robot);

% (robot,configuration,jointVel,jointTorq,fext)
f_ext = eye(6, 7);
f = forwardDynamics(robot, [0; pi / 2; 0], [1; 1; 1], [1; 1; 1], f_ext);

show(robot,[0; pi / 2; 0],"Collisions","on","Frames","on", 'PreservePlot',false);

end

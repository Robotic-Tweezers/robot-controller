function [robot] = create_robot()
%CREATE_ROBOT Create a robot object used in controls analysis
%   Detailed explanation goes here
%   https://www.mathworks.com/help/robotics/ug/build-a-robot-step-by-step.html
%   https://www.mathworks.com/help/robotics/ug/build-basic-rigid-body-tree-models.html

robot = importrobot("models\wrist.sdf", 'DataFormat', 'column');
showdetails(robot);

% (robot,configuration,jointVel,jointTorq,fext)
f_ext = [0 0 0 0 0 0; 0 0 0 0 0 0]';
f = forwardDynamics(robot, [0], [1], [1], f_ext);

show(robot,"Collisions","on","Frames","off");

end

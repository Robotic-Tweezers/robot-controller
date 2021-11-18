function [robot] = create_robot()
%CREATE_ROBOT Create a robot object used in controls analysis
%   Detailed explanation goes here
%   https://www.mathworks.com/help/robotics/ug/build-a-robot-step-by-step.html
%   https://www.mathworks.com/help/robotics/ug/build-basic-rigid-body-tree-models.html

robot = importrobot("models\wrist.sdf");

showdetails(robot);
show(robot,"Collisions","on","Frames","off");

end

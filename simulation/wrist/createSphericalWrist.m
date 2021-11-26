function robot = createSphericalWrist()
%CREATE_SPHERICAL_WRIST Create a robot object used in controls analysis
%   Loads SDF description of the spherical wrist used for the robot
%   tweezers project.
robot = importrobot("models/sdf/wrist.sdf", 'DataFormat', 'column');
robot.Gravity = [0 0 -9.81]';
show(robot, [0 0 0]', "Collisions", "off", "Frames", "off");

end

function robot = createSphericalWrist()
%CREATE_SPHERICAL_WRIST Create a robot object used in controls analysis
%   Loads SDF description of the spherical wrist used for the robot
%   tweezers project.
robot = importrobot("models/sdf/wrist.sdf", 'DataFormat', 'column');
end

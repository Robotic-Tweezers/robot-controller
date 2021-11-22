function table = denavitHartenbergTable(theta)
%DENAVITHARTENBERGTABLE returns the Denavit Hartenberg parameter table
%   Calculates the Denavit Hartenberg parameters for each frame conversion
%   for a spherical wrist.

table = [
    theta(1) 10 0 pi / 2;
    theta(2) 0  0 -pi / 2;
    theta(3) 15 0 0;
];

end


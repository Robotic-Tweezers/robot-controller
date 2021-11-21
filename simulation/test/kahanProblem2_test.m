s_unit = [
    [1, 1, 1] ./ sqrt(3);
    [1, 0, 0];
    [0, 1, 0];
    [0, 0, 1];
    [-1, -1, -2] ./ sqrt(6)
];
u = [
    [-1, -1, -1];
    [1, 1, 0];
    [1, 0, 0];
    [0, -1, 0];
    [0, 1, 0]
];
v = [
    [-1, 0, 0];
    [1, -1, 0];
    [0, 0, 1];
    [1, 0, 0];
    [0, 1, 0]
];

expected = [0.0, round(pi, 6), round(-pi/2, 6), round(pi/2, 6), 0]; % expected(1) not used

result = kahanProblem2(s_unit(1,1:3)', u(1,1:3)', v(1,1:3)');
assert(isnan(result));

for i = 2:5
    result = kahanProblem2(s_unit(i,1:3)', u(i,1:3)', v(i,1:3)');
    assert(result == expected(i));
end

clear s_unit;
clear u;
clear v;
clear expected;
clear result;

fprintf("PASS\n");
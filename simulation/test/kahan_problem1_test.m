s = [   [1, 1, 1];
        [1, 0, 0];
        [0, 1, 0];
        [0, 0, 1];
        [-1, -1, -2]];

t = [   [-1, -1, -1];
        [1, 1, 0];
        [1, 0, 0];
        [0, 0, 1];
        [0, 1, 0]];

expected = [3.142, round(pi/4, 3), round(pi/2, 3), 0, 1.991];

for i = 1:5
    result = round(kahan_problem1(s(i, :)', t(i, :)'), 3);
    assert(isequal(result, expected(i)));
end

fprintf("PASS\n");
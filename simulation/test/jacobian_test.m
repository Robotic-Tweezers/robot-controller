% TODO: test
jacobian = jacobian([0, 0, 0], parameters);
expected = [
    [0, -15, 0];
    [0, 0, 0];
    [0, 0, 0];
    [0, 0, 0];
    [0, -1, 0];
    [1, 0, 1];
];
assert(isequal(jacobian, expected));

clear jacobian;
clear expected;

fprintf("PASS\n");
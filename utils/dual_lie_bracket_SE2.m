function bracket = dual_lie_bracket_SE2(A, B)

% Calculate Lie brackets for all columns
bracket = [A(3, :).*B(2, :);
    -A(3, :).*B(1, :);
    A(2, :).*B(1, :) - A(1, :).*B(2, :)];

end
f1 = @(x) x.^2;
a1 = -5;
b1 = 5;
tol1 = 1e-6;
max_iter1 = 100;

[x_min1, f_min1, iter1, fevals1] = golden_section_search(f1, a1, b1, tol1, max_iter1);
fprintf('问题1：x* = %.6f, 迭代次数 = %d, 函数调用次数 = %d\n', x_min1, iter1, fevals1);

f2 = @(x) -sin(x) - exp(x/100) + 10;
a2 = 0;
b2 = 10;
tol2 = 1e-6;
max_iter2 = 100;

[x_min2, f_min2, iter2, fevals2] = golden_section_search(f2, a2, b2, tol2, max_iter2);
fprintf('问题2：x* = %.6f, 迭代次数 = %d, 函数调用次数 = %d\n', x_min2, iter2, fevals2);


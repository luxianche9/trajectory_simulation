clear;close all;clc
S_I = Coordinate("地心惯性坐标系", eye(3), eye(3));
S_E = Coordinate("地心固连坐标系", S_I.Axes, eul2rotm([0,0,deg2rad(30)], "XYZ"));

L = eul2rotm([0,0,deg2rad(30)], 'XYZ')';
vec_I = [1; 2; 3];
vec_E = L * vec_I;
vec_E = S_E.Axes * vec_E;

figure;
hold on;
S_I.draw(zeros(1, 3), 'r', 2, 1);
S_E.draw(zeros(1, 3), 'b', 2, 1);
quiver3(0,0,0,vec_I(1), vec_I(2), vec_I(3), 'k');
quiver3(0,0,0,vec_E(1), vec_E(2), vec_E(3), 'g--');
axis equal; view(3);
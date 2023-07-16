function [weld_x, weld_y, weld_z] = line_welding(st, ed)
% The function returns the Cartesian trajectory of the whole welding
% process, start from st = [st_x, st_y, st_z] to ed = [ed_x, ed_y, ed_z].

% Considering the requirements of line welding, we have to: (1) keep the
% welding in line; (2) a constant Cartesian velocity. Therefore, we may
% divide the welding line into several average line segments, where we
% address them as 'steps'.

% The number of the steps can be determined through epxeriments. The more
% steps the welding takes, the smaller its deviation from the welding line 
% and the more stable its velocity is.
steps = 29;
weld_x_seg = (ed(1) - st(1))/steps;   weld_x = zeros(1, steps + 1);
weld_y_seg = (ed(2) - st(2))/steps;   weld_y = zeros(1, steps + 1);
weld_z_seg = (ed(3) - st(3))/steps;   weld_z = zeros(1, steps + 1);

for i = 1 : (steps+1)
    weld_x(i) = st(1) + weld_x_seg*(i-1);
    weld_y(i) = st(2) + weld_y_seg*(i-1);
    weld_z(i) = st(3) + weld_z_seg*(i-1);
end

end


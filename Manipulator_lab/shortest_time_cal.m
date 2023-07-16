function [shortest_time] = shortest_time_cal(weld_traj_x, weld_traj_y, weld_traj_theta1, MTS_shortest_time)
% This function is to the shortest time in which the task can be
% accomplished.

% For the second step, we may use dX_cart=J(theta)*dtheta to find the 
% lowest cartesian velocity of the robot when running at different segments
% of the welding line at |dtheta_1|max = |dtheta_2|max = 50 degrees/s. 

% Since the robot is required to run in a constant speed, that speed is the
% highest cartesian space velocity for the robot to go in a straight line, 
% and thus the shortest time can be obtained.

L1 = 0.4; % Parameters of the SCARA A
welding_y_distance = 0.2;   % from y = 0.3 to y = 0.1

n = length(weld_traj_theta1);
vy_limited_each_segment = zeros(1, n);    % welding velocity at the y-projection in Cartesian space

vtheta_max = 50;    % |dtheta_1|max = |dtheta_2|max = 50 degrees/s

for i = 1 : n
    J = [-weld_traj_y(i), L1*sin(weld_traj_theta1(i)*pi/180) - weld_traj_y(i);...
         weld_traj_x(i), weld_traj_x(i) - L1*cos(weld_traj_theta1(i)/180*pi)];
    J_inv = inv(J);
    coeff_vy_limited_by_vtheta1 = J_inv(1,1)*3 + J_inv(1,2);
    coeff_vy_limited_by_vtheta2 = J_inv(2,1)*3 + J_inv(2,2);
    abs_vy_limited_by_vtheta1 = (vtheta_max/180*pi) / coeff_vy_limited_by_vtheta1;
    abs_vy_limited_by_vtheta2 = (vtheta_max/180*pi) / coeff_vy_limited_by_vtheta2;
    vy_limited_each_segment(i) = -min(abs_vy_limited_by_vtheta1, abs_vy_limited_by_vtheta2);
end

vy_max = min(vy_limited_each_segment);
welding_shortest_time = welding_y_distance / vy_max;
shortest_time = welding_shortest_time + MTS_shortest_time;

end


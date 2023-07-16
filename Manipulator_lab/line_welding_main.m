% MTS stands for "move to start (welding)", 
% that is start from joint [60, -40, 0.1] to cartesian [0.1, 0.3, 0]
MTS_st_joint = [60, -40, 0.1];  % joint coordinates where the robot always starts
weld_st_cart = [0.1, 0.3, 0];   % cartesian coordinates where welding starts
weld_ed_cart = [-0.5, 0.1, 0];  % cartesian coordiantes where welding ends 

[weld_st_theta1, weld_st_theta2, weld_st_L3] = Ki(weld_st_cart(1), weld_st_cart(2), weld_st_cart(3));
weld_st_joint = [weld_st_theta1, weld_st_theta2, weld_st_L3];

% Trajectory of MTS under two kinds of coordinates
[MTS_traj_theta1, MTS_traj_theta2, MTS_traj_L3, MTS_shortest_time] = MTS(MTS_st_joint, weld_st_joint);
[MTS_traj_x, MTS_traj_y, MTS_traj_z] = Kf(MTS_traj_theta1, MTS_traj_theta2, MTS_traj_L3);

% Trajectory of line welding under two kinds of coordinates
[weld_traj_x, weld_traj_y, weld_traj_z] = line_welding(weld_st_cart, weld_ed_cart);
[weld_traj_theta1, weld_traj_theta2, weld_traj_L3] = Ki(weld_traj_x, weld_traj_y, weld_traj_z);

% Trajectory of the whole process under two kinds of coordinates
traj_joint = transpose([MTS_traj_theta1, weld_traj_theta1;...
              MTS_traj_theta2, weld_traj_theta2;...
              MTS_traj_L3, weld_traj_L3]);
traj_cart = transpose([MTS_traj_x, weld_traj_x;...
             MTS_traj_y, weld_traj_y;...
             MTS_traj_z, weld_traj_z]);

% check the welding
draw_robot = 2; X_cart = rob_sim(traj_joint, draw_robot);
check_weld(X_cart)

% calculate the shortest time for the whole MTS-and-welding process
shortest_time = shortest_time_cal(weld_traj_x, weld_traj_y, weld_traj_theta1, MTS_shortest_time);

% plot the trajectory as a function of time for joint space (traj_joint)
time_steps = length(traj_joint(:, 1));
time_interval = 0.1;    % the time increment from one row to the next is 0.1s
t = 0 : time_interval : (time_steps-1)*time_interval;
figure(2);
subplot(2,1,1); xlabel('t/s'); ylabel('rad');
plot(t, traj_joint(:,1), 'r');           % theta1 of traj_joint
set(gca, 'xtick', 0:0.5:7);
set(gca, 'ytick', -50:25:150);
hold on; plot(t, traj_joint(:,2), 'g');  % theta2 of traj_joint
legend('\theta_1', '\theta_2');
title('\theta_1, \theta_2 of time-trajectory for joint space');
subplot(2,1,2); xlabel('t/s'); ylabel('m');
plot(t, traj_joint(:,3), 'b'); ylim([0, 0.3]);  % L3 of traj_joint
legend('L_3');
title('L_3 of time-trajectory for joint space');

% plot the trajectory as a function of time for cartesian space (traj_cart)
figure(3); xlabel('t/s'); ylabel('m');
plot(t, traj_cart(:,1), 'r');   % x of traj_cart
set(gca, 'xtick', 0:0.5:7);
set(gca, 'ytick', -0.6:0.1:0.8);
hold on
plot(t, traj_cart(:,2), 'g');   % y of traj_cart
plot(t, traj_cart(:,3), 'b');   % z of traj_cart
legend('x', 'y', 'z');
title('x, y, z of time-trajectory for cartesian space');
hold off

function [MTS_theta1, MTS_theta2, MTS_L3, MTS_shortest_time] = MTS(st, ed)
% MTS stands for "move to start (welding)", that is start from joint
% st = [st_theta1, st_theta2, st_L3] to ed = [ed_theta1, ed_theta2, ed_L3];

% [MTS_theta1, MTS_theta2, MTS_L3] is the joint space trajectory for the
% whole MTS process.

% Considering that no fixed trajectory is required during MTS, and we want
% to take the shortest time from the start of the simulation, thus during
% MTS, SCARA A should reach its highest velocity, that is 50 degrees per
% second for theta1 and theta2, and 0.1 meters per second for L3.

% In the trajectory for rob_sim, the time increment from one row to the
% next is 0.1s, thus v1 = v2 = 50/10 = 5, v3 = 0.1/10 = 0.01.
if ed(1) >= st(1)
    v1 = 5;
else v1 = -5;
end

if ed(2) >= st(2)
    v2 = 5;
else v2 = -5;
end

if ed(3) >= st(3)
    v3 = 0.01;
else v3 = -0.01;
end

step_theta1 = ceil((ed(1) - st(1))/v1);    % the shortest steps for theta1 during MTS
step_theta2 = ceil((ed(2) - st(2))/v2);    % ... for theta2
step_L3 = ceil((ed(3) - st(3))/v3);    % ... for L3

step = max([step_theta1, step_theta2, step_L3]);  % step_theta1 may be unequal to step_theta2 and step_L3

MTS_theta1 = zeros(1, step+1); MTS_theta2 = zeros(1, step+1); MTS_L3 = zeros(1, step+1);

for i = 1 : (step+1)
    if i <= step_theta1
        MTS_theta1(i) = st(1) + v1*(i-1);
    else
        MTS_theta1(i) = ed(1);
    end
    if i <= step_theta2
        MTS_theta2(i) = st(2) + v2*(i-1);
    else
        MTS_theta2(i) = ed(2);
    end
    % Before the end effector of SCARA A reaches the point weld_st_cart =
    % [0.1, 0.3, 0], the welding tool should not reach the plane z = 0, or
    % the robot will start welding outside the welding line.
    if i <= (step - step_L3)
        MTS_L3(i) = st(3);
    else
        MTS_L3(i) = st(3) + v3*(i-(step+1-step_L3));
    end
end

MTS_shortest_time = 0.1 * (step+1); % the time increment from one row to the next is 0.1s

end


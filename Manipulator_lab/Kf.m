function [x, y, z] = Kf(theta1, theta2, L3)
% Forward Kinematics Function
% The joint space coordinates are [theta1, theta2, L3], while the
% respective Cartesian coordinates are [x, y, z]

L1 = 0.4; L2 = 0.4; d1 = 0.2; % Parameters of the SCARA A

n1 = length(theta1); n2 = length(theta2); n3 = length(L3);
if n1 == n2 && n2 == n3
    n = n1;
else
    error('the length of theta1, theta2 and L3 are unequal!')
end

x = zeros(1, n); y = zeros(1, n); z = zeros(1, n);

for i = 1:n
    x(i) = L1 * cos(theta1(i)*pi/180) + L2 * cos((theta1(i)+theta2(i))*pi/180);
    y(i) = L1 * sin(theta1(i)*pi/180) + L2 * sin((theta1(i)+theta2(i))*pi/180);
    z(i) = d1 - L3(i);
end

end


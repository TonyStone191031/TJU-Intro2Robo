function [theta1, theta2, L3] = Ki(x, y, z)
% Inverse Kinematics Function
% The Cartesian coordinates are [x, y, z], while the respective joint space
% coordinates are [theta1, theta2, L3]

L1 = 0.4; L2 = 0.4; d1 = 0.2; % Parameters of the SCARA A

n1 = length(x); n2 = length(y); n3 = length(z);
if n1 == n2 && n2 == n3
    n = n1;
else
    error('the length of x, y and z are unequal!')
end

theta1 = zeros(1, n); theta2 = zeros(1, n); L3 = zeros(1, n);

for i = 1:n
    theta2(i) = acos((x(i)*x(i) + y(i)*y(i) - L1*L1 - L2*L2) / (2*L1*L2)) * 180/pi;
    theta1(i) = acos((L1*x(i)+L2*x(i)*cos(theta2(i)*pi/180)+L2*y(i)*sin(theta2(i)*pi/180)) ...
                      / (x(i)*x(i)+y(i)*y(i))) * 180/pi;
    L3(i) = d1 - z(i);
end

end


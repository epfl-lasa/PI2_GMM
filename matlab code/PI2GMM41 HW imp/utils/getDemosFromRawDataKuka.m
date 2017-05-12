x = VarName1;
y = VarName2;
z = VarName3;
q1 = VarName4;
q2 = VarName5;
q3 = VarName6;
q0 = VarName7;

figure
plot(y,z)
figure
hold on
plot3(x,y,z)
axis equal
view(90,0)


d = zeros(length(x),3);
for i = 1:length(x)
    % Rotation matrix from quaternion : in Diebel 2006 "Representing
    % Attitude: ...
RotQ = [q0(i)^2+q1(i)^2-q2(i)^2-q3(i)^2 2*(q1(i)*q2(i)+q0(i)*q3(i)) 2*(q1(i)*q3(i)-q0(i)*q2(i));
    2*(q1(i)*q2(i)-q0(i)*q3(i)) q0(i)^2-q1(i)^2+q2(i)^2-q3(i)^2 2*(q2(i)*q3(i)+q0(i)*q1(i));
    2*(q1(i)*q3(i)+q0(i)*q2(i)) 2*(q2(i)*q3(i)-q0(i)*q1(i)) q0(i)^2-q1(i)^2-q2(i)^2+q3(i)^2];


d(i,:) = RotQ'*[1 0 0]';   % [0 0 1] is the main direction of the tool wrt the reference frame of the wrist.
% Z = RotQ^T*z' , where z is a vector in the global coordinate and z' the
% same vector in the body-fixed coorinate (of the shovel). 
end
d = d*0.001; % scaling the arrows for the plotting
quiver3(x(1:20:end),y(1:20:end),z(1:20:end), d(1:20:end,1),d(1:20:end,2),d(1:20:end,3))


%%%%%%%%%%%%%%               
% Euler Angle Sequence 1,2,3 (first 3Z, 2Y, 1X , corresponding to Fixed 
% Angle sequence first X then Y then Z   alpha = psi, beta = theta, gamma =
% phi . a set of X-Y-Z fixed angles is exaclty equivalent to the same set
% of ZYX Euler angles. This result holds in general such that three
% rotations about the three axes of a fixed frame define the same
% orientation as the same trhee rotations taken in the oposite order about
% the three axes of a moving frame (same value around same axis)
alpha = atan2(2*(q0.*q1 + q2.*q3), 1-2*(q1.^2+q2.^2));
alpha = alpha + (alpha < 0) .* 2*pi;
alphaDeg = alpha*180/pi;
beta  = asin(2*(q0.*q2 - q3.*q1));
betaDeg = beta*180/pi;
gamma = atan2(2*(q0.*q3 + q1.*q2), 1-2*(q2.^2+q3.^2));
gammaDeg = gamma*180/pi;

figure
subplot(3,1,1)
plot(alphaDeg)
title('alpha degrees')
subplot(3,1,2)
plot(betaDeg)
title('beta degrees')
subplot(3,1,3)
plot(gammaDeg)
title('gamma degrees')


demo(1,:) = y;
demo(2,:) = z;
demo(3,:) = beta;

offset(1) = mean(x);
offset(4) = mean(alpha);
offset(6) = mean(gamma);
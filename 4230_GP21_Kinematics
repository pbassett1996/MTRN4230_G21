clear all;
clc;

%--------------------
% Forward Kinematics
%--------------------

%d - the "depth" along the previous joint's z axis
%θ (theta) - the rotation about the previous z (the angle between the common normal and the previous x axis)
%a - the radius of the new origin about the previous z (the length of the common normal)
%α (alpha) - the rotation about the new x axis (the common normal) to align the old z to the new z.

% ai = distance from Zi to Zi+1 measured along Xi
% αi = angle from Zi to Zi+1 measured about Xi
% di = distance from Xi−1 to Xi measured along Zi
% θi = angle from Xi−1 to Xi measured about Zi

theta = zeros(1,6); a = zeros(1,6); d = zeros(1,6); alpha = zeros(1,6);  %Links

%[rad]          [m]              [m]              [rad]
theta(1) = 0;   a(1) = 0;        d(1) = 0.1625;   alpha(1) = pi/2;  %Link 1
theta(2) = 0;   a(2) = -0.425;   d(2) = 0;        alpha(2) = 0;     %Link 2
theta(3) = 0;   a(3) = -0.3922;  d(3) = 0;        alpha(3) = 0;     %Link 3
theta(4) = 0;   a(4) = 0;        d(4) = 0.1333;   alpha(4) = pi/2;  %Link 4
theta(5) = 0;   a(5) = 0;        d(5) = 0.0997;   alpha(5) = -pi/2; %Link 5
theta(6) = 0;   a(6) = 0;        d(6) = 0.0996;   alpha(6) = 0;     %Link 6


for i = 1:6
T(:,:,i) = [cos(theta(i)), -sin(theta(i))*cos(alpha(i)), sin(theta(i))*sin(alpha(i)), a(i)*cos(theta(i));
           sin(theta(i)), cos(theta(i))*cos(alpha(i)),  -cos(theta(i))*sin(alpha(i)), a(i)*sin(theta(i));
           0,             sin(alpha(i)),                cos(alpha(i)),                d(i);
           0,             0,                            0,                            1                ];
end

T0_6 = T(:,:,1)*T(:,:,2)*T(:,:,3)*T(:,:,4)*T(:,:,5)*T(:,:,6) %Transformation Matrix from frame 6 to frame 0
% manipulator endpoint with respect to base is the last column of matrix.

x = T0_6(1,4)
y = T0_6(2,4)
z = T0_6(3,4)

%--------------------
% Inverse Kinematics
%--------------------
% Final Pos =
 x_f = -0.83431; %-0.8172 %-0.83431;
 y_f = 0.11041; %-0.2329 %0.11041;
 z_f = 0.45821; %0.628 %0.45821;
 P0_6 = [x_f; y_f; z_f];
 
 %The coordinate frame of the tool will always have the z-axis facing
 %downwards towards the object.
 % Therefore the rotation from the base coordinates to the tool frame is
 % 180 degrees about the base frames x axis.
 
 %Rotation about x-axis
 Rx = [1, 0, 0; 0, cos(-pi), -sin(-pi); 0, sin(-pi), cos(-pi)];
 T0_6 = [Rx, P0_6; 0, 0, 0, 1];
 
% Determining Theta(1).
    % θ1 = distance from X0 to X1 measured along Z1
    % Need to determine the location of frame 5 (wrist frame) in relation
    % to base frame; 0P5
    
    %P0_5 can be obtained from P0_6, it is a fixed distance -d(6) from joint 6
    %along the z axis
 
    P0_5 = T0_6*[0; 0; -d(6); 1];
    
%Two possible configurations for theta(1)
    
%theta(1) = atan2(P0_5(2), P0_5(1))+ acos(d(4)/sqrt(P0_5(1)^2+P0_5(2)^2))+pi/2;
theta(1) = atan2(P0_5(2), P0_5(1))- acos(d(4)/sqrt(P0_5(1)^2+P0_5(2)^2))+pi/2

% Determining Theta(5).
theta(5) = +acos((P0_6(1)*sin(theta(1))-P0_6(2)*cos(theta(1))-d(4))/d(6));
%theta(5) = -acos((P0_6(1)*sin(theta(1))-P0_6(2)*cos(theta(1))-d(4))/d(6));

% Determining Theta(6).
%Redundant

% Determining Theta(3).
T5_6 = getTMatrix(theta(6), a(6), d(6), alpha(6));
T4_5 = getTMatrix(theta(5), a(5), d(5), alpha(5));
T0_1 = getTMatrix(theta(1), a(1), d(1), alpha(1));

T6_5 = inv(T5_6);
T0_5 = T0_6*T6_5;

T5_4 = inv(T4_5);
T0_4 = T0_5*T5_4;

T1_4 = T0_4*T0_1;
P1_4 = T1_4(1:3,4);

P1_4xz = sqrt(P1_4(1)^2+P1_4(3)^2);

theta(3) = +acos((abs(P1_4xz)^2-(a(2))^2-(a(3))^2)/(2*a(2)*a(3)));

%Finding theta(2)
theta(2) = atan2(-P1_4(3),-P1_4(1))-asin((-a(3)*sin(theta(3)))/abs(P1_4xz))

%Finding theta(4)

T3_4 = getTMatrix(theta(4), a(4), d(4), alpha(4));

X3_4 = T3_4(1:3,1);

theta(4) = atan2(X3_4(2), X3_4(1));


function T = getTMatrix(theta, a, d, alpha)

    T = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha),  a*cos(theta);
         sin(theta), cos(theta)*cos(alpha),  -cos(theta)*sin(alpha), a*sin(theta);
         0,          sin(alpha),             cos(alpha),             d;
         0,          0,                      0,                      1          ];
end


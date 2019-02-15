%% 1.1 Robot simulation:
open('sim1_2.slx');

% 1.2 Robot Animation:
sim('sim1_2');

x1= l1.*cos(theta1);
y1=l1.*sin(theta1);
x2=l1.*cos(theta1)+l2.*cos(theta1+theta2);
y2=l1.*sin(theta1)+l2.*sin(theta1+theta2);

figure(1);
    for i = 1:20:size(x1,1)
    plot([0 x1(i) x2(i)],[0 y1(i) y2(i)],'-o','linewidth',3);
    axis([-1.1 1.1 -0.85 0.85])
    title(['Time: ' num2str(round(tout(i)))])
    xlabel('x(m)');
    ylabel('y(m)');
    pause(1e-6)
    end
    

% 1.3 observation of chaos:
sim('sim1_2');
sim('sim1_3');

% 1.3.1 plot of angle vs time
figure(2);
plot(time,theta1,time,theta2);
hold on;
plot(time,theta1_3,time,theta2_3);
xlabel('Time(s)');
ylabel('Theta(rad)');
title('Angles vs Time with different initial conditions');
legend('theta1','theta2','theta3','theta4');

%  1.3.2 animaiton plot
x3= l1.*cos(theta1_3);
y3=l1.*sin(theta1_3);
x4=l1.*cos(theta1_3)+l2.*cos(theta1_3+theta2_3);
y4=l1.*sin(theta1_3)+l2.*sin(theta1_3+theta2_3);

figure(3);
for i = 1:20:size(x1,1)
    plot([0 x1(i) x2(i)],[0 y1(i) y2(i)],'-o','linewidth',3); 
    axis([-1.1 1.1 -0.85 0.85])
    title(['Time: ' num2str(round(tout(i)))])
    
    hold on;
    plot([0 x3(i) x4(i)],[0 y3(i) y4(i)],'-or','linewidth',3);
    axis([-1.1 1.1 -0.85 0.85])
    title(['Time: ' num2str(round(tout(i)))])
    xlabel('x(m)');
    ylabel('y(m)');
    pause(1e-6);
    hold off;
end

% function inside the plant:
% function theta_dot_dot = fcn(T,theta,theta_dot)
% l1 =0.5;    %m
% l2 = 0.3;   %m
% r1 = 0.25;  %m
% r2 = 0.15;  %m
% m1 = 0.5;   %kg
% m2 = 0.3;   %kg
% I1 = 0.0104;%kg-m^2
% I2 = 0.0022;%kg-m^2
% c1 = 0;     %N.m.s/rad
% c2 = 0;     %N.m.s/rad
% g = 9.81;   %m/s^2
% 
% alpha = I1+I2+m1*r1^2+m2*(l1^2+r2^2);
% beta = m2*l1*r2;
% gamma = I2+m2*r2^2;
% 
% theta1 = theta(1);
% theta2 = theta(2);
% theta1_dot = theta_dot(1);
% theta2_dot = theta_dot(2);
% 
% M =[alpha+2*beta.*cos(theta2) gamma+beta.*cos(theta2); gamma+beta.*cos(theta2) gamma];
% C = [c1-beta.*theta2_dot.*sin(theta2) -beta.*(theta1_dot+theta2_dot).*sin(theta2); beta*theta1_dot.*sin(theta2) c2];
% N = [(m1*r1+m2*l1)*g.*cos(theta1)+m2*r2*g.*cos(theta1+theta2); m2*r2*g.*cos(theta1+theta2)];
% 
% theta_dot_dot = inv(M)*(T-C*theta_dot-N);
% end

%% 2.1 control design using feedback linearization:


%% 2.2 implementation of controller:
open('sim2_2.slx');

% 2.2.1 Running a basic feedback simulation:
figure(4);
sim('sim2_2');
plot(time,theta);
xlabel('Time(S)');
ylabel('Theta(rad)');
title('Angle vs Time');
legend('theta1','theta2');
fprintf('yes, controller does it job as expected \n \n');

% 2.2.2 Impact of actuator saturaiton:
figure(5);
sim('sim2_2_2');
plot(time,theta);
xlabel('Time(S)');
ylabel('Theta(rad)');
title('Impact of actuator saturation - Angle vs Time');
legend('theta1','theta2');
fprintf('There is change in theta values at the start and end of step');

% 2.2.3 Impacts of parametric uncertainity:
figure(6);
sim('sim2_2_3');
plot(time,theta);
xlabel('Time(S)');
ylabel('Theta(rad)');
title('Impacts of parametric uncertainity - Angle vs Time');
legend('theta1','theta2');
fprintf('theta1, theta2 have small difference for 1- 5 seconds \n \n');

% 2.2.4 Trajectory filtering:
figure(7);
si=2;
sim('sim2_2_4');
plot(time,theta);
xlabel('Time(S)');
ylabel('Theta(rad)');
title('Trajectory filtering - Angle vs Time');
legend('theta1','theta2');
fprintf('theta1 and theta 2 are same for 1-5seconds, with given lambda, sigma values, parameters mismatch is mitigated \n \n');


% funciton in the controller:
% function T = fcn3(theta_des,theta_des_dot,theta_des_dot_dot, theta, theta_dot)
% l1 =0.5;    %m
% l2 = 0.3;   %m
% r1 = 0.25;  %m
% r2 = 0.15;  %m
% m1 = 0.5;   %kg
% m2 = 0.3;   %kg
% I1 = 0.0104;%kg-m^2
% I2 = 0.0022;%kg-m^2
% c1 = 0;     %N.m.s/rad
% c2 = 0;     %N.m.s/rad
% g = 9.81;   %m/s^2
% 
% alpha = I1+I2+m1*r1^2+m2*(l1^2+r2^2);
% beta = m2*l1*r2;
% gamma = I2+m2*r2^2;
% 
% theta1 = theta(1);
% theta2 = theta(2);
% theta1_dot = theta_dot(1);
% theta2_dot = theta_dot(2);
% 
% M =[alpha+2*beta.*cos(theta2) gamma+beta.*cos(theta2); gamma+beta.*cos(theta2) gamma];
% C = [c1-beta.*theta2_dot.*sin(theta2) -beta.*(theta1_dot+theta2_dot).*sin(theta2); beta*theta1_dot.*sin(theta2) c2];
% N = [(m1*r1+m2*l1)*g.*cos(theta1)+m2*r2*g.*cos(theta1+theta2); m2*r2*g.*cos(theta1+theta2)];
% 
% lambda=10;
% 
% K1=[2*lambda; 2*lambda];
% K2=[lambda^2;lambda^2];
% 
% e=theta_des-theta;
% 
% e_dot=theta_des_dot-theta_dot;
% 
% e_dot_dot=-K1.*e_dot-K2.*e;
% 
% T=M*theta_des_dot_dot-M*e_dot_dot+C*theta_dot+N;
% 
% end


%% 3. End effector position control by incorportating inverse kinematics
open('sim3_1_2.slx');

% 3.2 line tracking:
figure(9);
si=2;
sim('sim3_1_2');
plot(xd,yd,x,y);
xlabel('xd, x in meters');
ylabel('yd, y in meters');
title('Desired and actual end effector positions');
legend('desired','actual');
fprintf('yes it tracks the desired line \n \n');

% 3.3 Improvement of performance:
figure(10);
si=5;
sim('sim3_1_3');
plot(xd,yd,x,y);
xlabel('xd, x in meters');
ylabel('yd, y in meters');
title('Desired and actual end effector positions with sigma=5');
legend('desired','actual');

figure(11);
si=10;
sim('sim3_1_3');
plot(xd,yd,x,y);
xlabel('xd, x in meters');
ylabel('yd, y in meters');
title('Desired and actual end effector positions with sigma=10');
legend('desired','actual');
fprintf('Due to the change of transfer funcitons, there is improvement of inputs to the controller, so tracking performance is increased.\n \n')

% 3.4 vertical line tracking:
figure(12);
si=10;
sim('sim3_1_4');
plot(xd,yd,x,y);
xlabel('xd, x in meters');
ylabel('yd, y in meters');
title('Desired and actual end effector positions - vertical line tracing');
fprintf('vertical line tracking accuracy is not same as horizontal line tracking accuracy \n \n')

% 3.5 vertical line tracking on other side:
figure(13);
si=10;
sim('sim3_1_5');
plot(xd,yd,x,y);
xlabel('xd, x in meters');
ylabel('yd, y in meters');
title('Desired and actual end effector positions - vertical line tracking on other side');
legend('desired','actual');
fprintf('Because of minus sign in the step function we got this. \n \n')

% funciton inside inverse kinematics:
% function [theta1_des,theta2_des]= fcn(xd,yd,theta)
% l1 =0.5;    %m
% l2 = 0.3;   %m
% 
% theta2=theta(2);
% theta2_des=acos(((xd^2+yd^2-l1^2-l2^2)/(2*l1*l2)));
% theta1_des=atan2(yd,xd)-atan2(l2*sin(theta2),(l1+l2*cos(theta2)));
% 
% end

% funciton inside forward kinematics:
% function [x,y]= fcn(theta)
% l1 =0.5;    %m
% l2 = 0.3;   %m
% r1 = 0.25;  %m
% r2 = 0.15;  %m
% m1 = 0.48;   %kg
% m2 = 0.32;   %kg
% I1 = 0.0100;%kg-m^2
% I2 = 0.0024;%kg-m^2
% c1 = 0;     %N.m.s/rad
% c2 = 0;     %N.m.s/rad
% g = 9.81;   %m/s^2
% 
% alpha = I1+I2+m1*r1^2+m2*(l1^2+r2^2);
% beta = m2*l1*r2;
% gamma = I2+m2*r2^2;
% 
% theta1 = theta(1);
% theta2 = theta(2);
% 
% x= l1*cos(theta1)+l2*cos(theta1+theta2);
% y= l1*sin(theta1)+l2*sin(theta1+theta2);
% end


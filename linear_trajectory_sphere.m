clear all;
%load variables
load('trajectory_data.mat');
%% Robot Creation
% Initial configuration guessed from teach function
theta=[0,0,0,180,90,-90]/180*pi;
% DH parameters
a2=0.4318;
a3=0.0202;
d3=0.1244;
d4=0.4318;
% DH table [theta, d, a, alpha]
Link1=Link([theta(1),0,0,0],'modified');
Link2=Link([theta(2),0,0,-pi/2],'modified');
Link3=Link([theta(3),d3,a2,0],'modified');
Link4=Link([theta(4),d4,a3,-pi/2],'modified');
Link5=Link([theta(5),0,0,pi/2],'modified');
Link6=Link([theta(6),0,0,-pi/2],'modified');
Links=[Link1,Link2,Link3,Link4,Link5,Link6];
% Create robot
PUMA_cw=SerialLink(Links,'name','PUMA');

%% Workspace
%Determine reachable workspace. a,e = first and last reachable points
a=1;
for p = 1:29
    x = pos_static_trajectory(1,p);
    y = pos_static_trajectory(2,p);
    
    if (abs(((x^2+y^2)-a2^2-d4^2)/(2*a2*d4))>1)
        abs(((x^2+y^2)-a2^2-d4^2)/(2*a2*d4));
        a = a + 1;
    else
        break
    end
end
e=a-1;
for p = a:29
    x = pos_static_trajectory(1,p);
    y = pos_static_trajectory(2,p);
    
    if (abs(((x^2+y^2)-a2^2-d4^2)/(2*a2*d4))<1)
        abs(((x^2+y^2)-a2^2-d4^2)/(2*a2*d4));
        e = e + 1;
    else
        break
    end
end

%% Transformation matrices and trajectories

for p = a:e;
    %Creates homogeneous transformation matrix for each point
    T{p,1}= transl(pos_static_trajectory(:,p));
    if p==a
        T_Traj = T{p,1};
    else
        T_Traj = cat(3,T_Traj,T{p,1});
    end
end

%% Joint angles

%Empty matrix of angles
q_init = NaN(a-1,6);
%Calculate first pose with inverse kinematics
q_start=PUMA_cw.ikine(T{a,1},theta);
q_init=[q_init;q_start];

%Calculate all the poses for every point in the trajectory
for p = a+1:e;
            q_init_temp=PUMA_cw.ikine(T{p,1},q_start);
            q_start = q_init_temp;
            q_init = [q_init;q_init_temp];
end
time = [0:size(q_init)-a];
%% Plotting

%Plots the Puma moving over all points
figure

for i=1:size(q_init,1)-a;
        %Plot the robot
        PUMA_cw.plot(q_init(a-1+i,:));
        %Plot the trajectory
        x = reshape(T_Traj(1,4,1:i),i,1,1);
        y = reshape(T_Traj(2,4,1:i),i,1,1);
        z = reshape(T_Traj(3,4,1:i),i,1,1);
        hold on;
        plot3(x,y,z,'.r');
        %Draw centre of sphere
        plot3(1,0,0,'.b');
        hold off; 
end

figure('units','normalized','position',[0 0 700 200])
r=2;
c=3;
subplot(r,c,1)
plot(time,q_init(a:end,1),'r--')
xlabel('Time [s]')
ylabel('Theta_1 [rad]')
subplot(r,c,2)
plot(time,q_init(a:end,2),'g--')
xlabel('Time [s]')
ylabel('Theta_2 [rad]')
subplot(r,c,3)
plot(time,q_init(a:end,3),'b--')
xlabel('Time [s]')
ylabel('Theta_3 [rad]')
subplot(r,c,4)
plot(time,q_init(a:end,4),'b--')
xlabel('Time [s]')
ylabel('Theta_4 [rad]')
subplot(r,c,5)
plot(time,q_init(a:end,5),'b--')
xlabel('Time [s]')
ylabel('Theta_5 [rad]')
subplot(r,c,6)
plot(time,q_init(a:end,6),'b--')
xlabel('Time [s]')
ylabel('Theta_6 [rad]')
clear all;
%load variables
load('trajectory_data.mat');

%% Robot Creation
% Initial configuration (guessed using the teach function)
theta=[0,0,-90,180,90,-180]/180*pi;
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
    x = pos_beat_trajectory(1,p);
    y = pos_beat_trajectory(2,p);
    
    if (abs(((x^2+y^2)-a2^2-d4^2)/(2*a2*d4))>1)
        abs(((x^2+y^2)-a2^2-d4^2)/(2*a2*d4));
        a = a + 1;
    else
        break
    end
end
e=a-1;
for p = a:29
    x = pos_beat_trajectory(1,p);
    y = pos_beat_trajectory(2,p);
    
    if (abs(((x^2+y^2)-a2^2-d4^2)/(2*a2*d4))<1)
        abs(((x^2+y^2)-a2^2-d4^2)/(2*a2*d4));
        e = e + 1;
    else
        break
    end
end

%% Initialisation
%Create empy arrays
q_init = NaN(a-1,6);
npoints = NaN(a-1,1);
vel_vec = NaN(a-1,3);
rot_vel = NaN(a-1,3);
tr = NaN(a-1,6);
thdot = NaN(a-1,6);
T_traj = NaN(4,4,a-1);
jac_total = NaN(a-1*6,6);
S=zeros(1,a);
%Initialise velocity, dt and ds (time and space intervals)
vel = 0.1;
dt = 0.05;
ds = vel*dt;

%% Transformation matrices and trajectories
%Calculate tranformation matrix for all reachable points
for p = a:e 
    rotation = angvec2r(alpha(p),vector);
    T{p,1}= rt2tr(rotation, pos_beat_trajectory(:,p));   
end
%Interpolation
for p=a:e-1 
    %Calculate number of interpolate points in the interval between two points
    npoints(p) = floor(norm(pos_beat_trajectory(:,p+1)-pos_beat_trajectory(:,p))./ds);
    %Error
    leftover(p) = norm(pos_beat_trajectory(:,p+1)-pos_beat_trajectory(:,p))-npoints(p)*ds;

    %Calculate interpolated trajectory
    T_traj_temp = ctraj(T{p,1},T{p+1,1},linspace(0,1,npoints(p)));
    T_traj = cat(3,T_traj,T_traj_temp(:,:,1:end-1));
end

%% Velocities
%Calculating velocities between each pair of interpolated points
count=a;
while count < size(T_traj,3)
    %Difference in position and rotation between two transformations
    tr_temp = tr2delta(T_traj(:,:,count),T_traj(:,:,count+1))';
    tr = [tr;tr_temp];
    %Calculate unit vector pointing in the direction of next point
    d_temp = (T_traj(1:3,4,count+1)-T_traj(1:3,4,count))';
    d_temp = d_temp/norm(d_temp);
    %Calculate velocity vector pointing in that direction
    vel_vec_temp = d_temp*vel;
    vel_vec = [vel_vec;vel_vec_temp];
    %Calculate rotational velocity   
    rot_vel_temp = tr(count,:)./dt;
    rot_vel_temp = rot_vel_temp(4:6);
    rot_vel = [rot_vel;rot_vel_temp];
    
    count = count + 1;   
end

vel_tot = [vel_vec rot_vel];

%% Calculating joint angles
%Use inverse kinematics to calculate initial configuration with initial guess
q_start=PUMA_cw.ikine(T_traj(:,:,a),theta);
q_init=[q_init;q_start];

%Obtain subsequent joint angles using the jacobian
for z = a+1:size(T_traj,3)-1
        
        jac = PUMA_cw.jacob0(q_init(z-1,:));
        q_init_temp = q_init(z-1,:) + ((pinv(jac)*vel_tot(z-1,:)').*dt)';
        %Check with forward kinematics how close we are to desired pose
        T_temp = PUMA_cw.fkine(q_init_temp);
        error = tr2delta(T_temp,T_traj(:,:,z));
        %Repeat until error is minimised
        while norm(error(4:6)) > 0.00005
            jac = PUMA_cw.jacob0(q_init(z-1,:));
            q_init_temp = q_init(z-1,:) + ((pinv(jac)*vel_tot(z-1,:)').*dt)';
            T_temp = PUMA_cw.fkine(q_init_temp);
            error = tr2delta(T_temp,T_traj(:,:,z));
        end
        q_init = [q_init;q_init_temp];
        jac_total = [jac_total;jac];
        % Store rotational velocity (to double check whether it's the same as in
        % the rot_vel matrix)
        thdot_temp = (q_init(z,:)-q_init(z-1,:))/dt;
        thdot = [thdot;thdot_temp];
    
end

%Create time vector
time=[0:dt:size(q_init,1)*dt];

%% Plotting

%Plots the Puma moving over all points
figure

for i=1:100:size(q_init,1)-a;
        %Plot robot
        PUMA_cw.plot(q_init(a-1+i,:));
        %Plot trajectory
        x = reshape(T_traj(1,4,1:i),i,1,1);
        y = reshape(T_traj(2,4,1:i),i,1,1);
        z = reshape(T_traj(3,4,1:i),i,1,1);
        hold on;
        plot3(x,y,z,'.r');
        %Draw centre of sphere
        plot3(1,0,0,'.b');
        %Plot radial lines towards centre to check normality to surface
        %plot3([1,T_traj(1,4,i)],[0,T_traj(2,4,i)],[0,T_traj(3,4,i)])
        hold off; 
end

% Plots graphs

%Plot end effector velocity
figure('units','normalized','position',[0 0 700 200])
plot(time(1:end-1),vel_vec(:,1),'b--',time(1:end-1),vel_vec(:,2),'r--',time(1:end-1),sqrt(vel_vec(:,1).^2+(vel_vec(:,2).^2)),'g--')
leg = legend('Vx','Vy','|Vtot|');
set(leg,'FontSize',14);
xlabel('Time [s]')
ylabel('V [rad/s]')
title('Velocity of end effector');

% Plot joint angles
figure('units','normalized','position',[10 -10 700 200])
r=2;
c=3;
subplot(r,c,1)
plot(time(1,1:end-1),q_init(:,1),'r--')
xlabel('Time [s]')
ylabel('Theta_1 [rad]')
subplot(r,c,2)
plot(time(1,1:end-1),q_init(:,2),'g--')
xlabel('Time [s]')
ylabel('Theta_2 [rad]')
title('Joint angles vs. time');
subplot(r,c,3)
plot(time(1,1:end-1),q_init(:,3),'b--')
xlabel('Time [s]')
ylabel('Theta_3 [rad]')
subplot(r,c,4)
plot(time(1,1:end-1),q_init(:,4),'b--')
xlabel('Time [s]')
ylabel('Theta_4 [rad]')
subplot(r,c,5)
plot(time(1,1:end-1),q_init(:,5),'b--')
xlabel('Time [s]')
ylabel('Theta_5 [rad]')
subplot(r,c,6)
plot(time(1,1:end-1),q_init(:,6),'b--')
xlabel('Time [s]')
ylabel('Theta_6 [rad]')
% Plot joint velocities
figure('units','normalized','position',[10 -10 700 200])

subplot(r,c,1)
plot(time(1:end-2),thdot(:,1),'r--')
xlabel('Time [s]')
ylabel('dTheta_1/dt [rad/s]')
subplot(r,c,2)
plot(time(1:end-2),thdot(:,2),'g--')
xlabel('Time [s]')
ylabel('dTheta_2/dt [rad/s]')
title('Joint angular velocities vs. time');
subplot(r,c,3)
plot(time(1:end-2),thdot(:,3),'b--')
xlabel('Time [s]')
ylabel('dTheta_3/dt [rad/s]')
subplot(r,c,4)
plot(time(1:end-2),thdot(:,4),'r--')
xlabel('Time [s]')
ylabel('dTheta_4/dt [rad/s]')
subplot(r,c,5)
plot(time(1:end-2),thdot(:,5),'g--')
xlabel('Time [s]')
ylabel('dTheta_5/dt [rad/s]')
subplot(r,c,6)
plot(time(1:end-2),thdot(:,6),'b--')
xlabel('Time [s]')
ylabel('dTheta_6/dt [rad/s]')



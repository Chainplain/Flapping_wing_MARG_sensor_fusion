%%% developed by chainplain 2022年11月15日 
clear all;
DarkRed = [0.7,0.2,0.2];
PatchRed = [0.5,0.25,0.25];
LighGrayRed = [0.9,0.5,0.5];

DarkRed_2 = [0.5,0.3,0.3];
PatchRed_2 = [0.8,0.6,0.6];
LighGrayRed_2 = [0.95,0.55,0.75];

DarkGreen = [0.2,0.7,0.2];
PatchGreen = [0.25,0.5,0.25];
LighGrayGreen = [0.5,0.9,0.5];

DarkGreen_2 = [0.3,0.5,0.3];
PatchGreen_2 = [0.6,0.8,0.6];
LighGrayGreen_2 = [0.55,0.95,0.65];

DarkBlue = [0.2,0.2 ,0.7];
PatchBlue = [0.25,0.25, 0.5];
LighGrayBlue = [0.5,0.5,0.9];

DarkBlue_2 = [0.3,0.3,0.5];
PatchBlue_2 = [0.6,0.6,0.8];
LighGrayBlue_2 = [0.55,0.7,0.95];

LighGray = [0.8,0.8,0.8];
x_tick =0 + [0 250 500 750 1000 1250 1500 1750];
% x_tick = [0 2000 4000 6000 8000 10000 12000];

Experiment_name_list = {'4th_proposed'; '4th_tau1'; '4th_tau2'};
Freq = '_13Hz_DW_';
AngularVelocity = '0_n05pi_0';
%0_n05pi_0
Rear_name = '_Attitude_Tracking_File.mat';
LineWidth = 1;
Emph = 1.5;
BasisRotation = [1, 0, 0;...
                 0, 0, 1;...
                 0,-1, 0]';%Because the inertia rotation matrix in webots
             %is y pointing up

for i = 1 : 3
    load([Experiment_name_list{i}, Freq, AngularVelocity, Rear_name]);
    Length =  size(Total_body_rotation_list,1);
    % Total_body_rotation_in_Euler_list = zeros(Length,3);

    Total_body_rotation_list_a = permute( Total_body_rotation_list,[2,3,1]);
    for j = 1 : Length
        Total_body_rotation_list_a(:,:,j) = BasisRotation * Total_body_rotation_list_a(:,:,j);
    end
    Total_body_rotation_in_Euler_list_S{i} = rotm2eul(Total_body_rotation_list_a);
    
    Total_desired_rotation_list_a = permute( Total_desired_rotation_list,[2,3,1]);
     for j = 1 : Length
        Total_desired_rotation_list_a(:,:,j) = BasisRotation * Total_desired_rotation_list_a(:,:,j);
    end
    Total_desired_rotation_in_Euler_list_S{i} = rotm2eul(Total_desired_rotation_list_a);
    
    Total_body_angular_velocity_list_S{i} = Total_body_angular_velocity_list;
    Total_Angular_velocity_filtered_list_S{i} = Total_Angular_velocity_filtered_list;
    
    Total_desired_angular_velocity_list_S{i} = Total_desired_angular_velocity_list;
    
    Total_psi_rotation_error_list_S{i} = Total_psi_rotation_error_list;
    Total_pitch_input_S{i} = Total_pitch_input;
    Total_roll_input_S{i} = Total_roll_input;
    Total_yaw_input_S{i} = Total_yaw_input;
    
     for j = 1 : Length
        Total_wing_torque(j,:) = ((BasisRotation' *Total_body_rotation_list_a(:,:,j))' * Total_wing_torque(j,:)')';
        Total_wing_force(j,:) = ((BasisRotation' *Total_body_rotation_list_a(:,:,j))' * Total_wing_force(j,:)')';
        Total_rudder_torque(j,:) = ((BasisRotation' *Total_body_rotation_list_a(:,:,j))' * Total_rudder_torque(j,:)')';
        Total_rudder_force(j,:) = ((BasisRotation' *Total_body_rotation_list_a(:,:,j))' * Total_rudder_force(j,:)')';
        Total_tail_torque(j,:) = ((BasisRotation' *Total_body_rotation_list_a(:,:,j))' * Total_tail_torque(j,:)')';
        Total_tail_force(j,:) = ((BasisRotation' *Total_body_rotation_list_a(:,:,j))' * Total_tail_force(j,:)')';
    end
    
    
    Total_wing_torque_S{i} = Total_wing_torque;
    Total_wing_force_S{i} = Total_wing_force;
    Total_rudder_torque_S{i} = Total_rudder_torque;
    Total_rudder_force_S{i} = Total_rudder_force;
    Total_tail_torque_S{i} = Total_tail_torque;
    Total_tail_force_S{i} = Total_tail_force;
end

figure;


subplot(7,1,1);
hold on;
plot(Total_psi_rotation_error_list_S{1},"-",'LineWidth',LineWidth* Emph,'Color',LighGray * 0.4);
% plot(Total_psi_rotation_error_list_S{2},":",'LineWidth',LineWidth,'Color',LighGray* 0.6);
% plot(Total_psi_rotation_error_list_S{3},"-.",'LineWidth',LineWidth,'Color',LighGray* 0.8);
axis([x_tick(1),x_tick(end),-0.02,1.4])
set(gca,'xtick',x_tick);
set(gca,'ytick',[0,  0.7,  1.4]);
grid on;
h = gca;
set(h,'LineWidth',1,'GridLineStyle','--','GridAlpha',1,'GridColor',[0.9,0.9,0.9])
hold off;

subplot(7,1,2);
hold on;
plot(Total_body_angular_velocity_list_S{1}(:,1),"-",'LineWidth',LineWidth * Emph,'Color',PatchRed_2 * 1.1);
plot(Total_Angular_velocity_filtered_list_S{1}(:,1),"-",'LineWidth',LineWidth * Emph,'Color',LighGrayRed* 0.4);
% plot(Total_body_angular_velocity_list_S{2}(:,1),":",'LineWidth',LineWidth,'Color',LighGrayRed* 0.6);
% plot(Total_body_angular_velocity_list_S{3}(:,1),"-.",'LineWidth',LineWidth,'Color',LighGrayRed* 0.8);
plot(Total_desired_angular_velocity_list_S{1}(:,1),"--",'LineWidth',LineWidth ,'Color',LighGrayRed* 0.6);
% legend('Raw data','Filtered','Desired');
set(gca,'xtick',x_tick);
set(gca,'ytick',[-5, 0 5]);
axis([x_tick(1) x_tick(end) -5 5]);
grid on;
h = gca;
set(h,'LineWidth',1,'GridLineStyle','--','GridAlpha',1,'GridColor',[0.9,0.9,0.9])
hold off;

subplot(7,1,3);
hold on;
plot(Total_body_angular_velocity_list_S{1}(:,2),"-",'LineWidth',LineWidth* Emph,'Color',PatchGreen_2 * 1.1);
plot(Total_Angular_velocity_filtered_list_S{1}(:,2),"-",'LineWidth',LineWidth* Emph,'Color',PatchGreen_2 * 0.5);
% plot(Total_body_angular_velocity_list_S{2}(:,2),":",'LineWidth',LineWidth ,'Color',LighGrayGreen* 0.6);
% plot(Total_body_angular_velocity_list_S{3}(:,2),"-.",'LineWidth',LineWidth ,'Color',LighGrayGreen* 0.8);
plot(Total_desired_angular_velocity_list_S{1}(:,2),"--",'LineWidth',LineWidth ,'Color',LighGrayGreen* 0.6);
% legend('Raw data','Filtered','Desired');
set(gca,'xtick',x_tick);
set(gca,'ytick',[-10, 0, 10]);
axis([x_tick(1) x_tick(end) -10 10]);
grid on;
h = gca;
set(h,'LineWidth',1,'GridLineStyle','--','GridAlpha',1,'GridColor',[0.9,0.9,0.9])
hold off;

subplot(7,1,4);
hold on;
plot(Total_body_angular_velocity_list_S{1}(:,3),"-",'LineWidth',LineWidth* Emph,'Color',PatchBlue_2 * 1.1);
plot(Total_Angular_velocity_filtered_list_S{1}(:,3),"-",'LineWidth',LineWidth* Emph,'Color',PatchBlue_2 * 0.5);
% plot(Total_body_angular_velocity_list_S{2}(:,3),":",'LineWidth',LineWidth ,'Color',LighGrayBlue* 0.6);
% plot(Total_body_angular_velocity_list_S{3}(:,3),"-.",'LineWidth',LineWidth ,'Color',LighGrayBlue* 0.8);
plot(Total_desired_angular_velocity_list_S{1}(:,3),"--",'LineWidth',LineWidth ,'Color',LighGrayBlue* 0.6);
% legend('Raw data','Filtered','Desired');
set(gca,'xtick',x_tick);
set(gca,'ytick',[-20, 0 20]);
axis([x_tick(1) x_tick(end) -20 20]);
grid on;
h = gca;
set(h,'LineWidth',1,'GridLineStyle','--','GridAlpha',1,'GridColor',[0.9,0.9,0.9])
hold off;

subplot(7,1,5);
hold on;
plot(Total_roll_input_S{1},"-",'LineWidth',LineWidth ,'Color',PatchRed_2 * 0.8);
plot(Total_pitch_input_S{1},"-",'LineWidth',LineWidth ,'Color',PatchGreen_2 * 0.8);
plot(Total_yaw_input_S{1},"-",'LineWidth',LineWidth ,'Color',PatchBlue_2 * 0.8);
% legend('Pitch input','Roll input','Yaw input');
set(gca,'xtick',x_tick);
set(gca,'ytick',[-1, 0 1]);
axis([x_tick(1) x_tick(end) -1 1]);
grid on;
h = gca;
set(h,'LineWidth',1,'GridLineStyle','--','GridAlpha',1,'GridColor',[0.9,0.9,0.9])
hold off;

subplot(7,1,6);
hold on;
Force = Total_wing_force_S{1} + Total_rudder_force_S{1} + Total_tail_force_S{1};

plot(Force(:,1),"-",'LineWidth',LineWidth ,'Color',DarkRed * 1.1);
plot(Force(:,2),"-",'LineWidth',LineWidth ,'Color',DarkGreen * 1.1);
plot(Force(:,3),"-",'LineWidth',LineWidth ,'Color',DarkBlue * 1.1);
% legend('Pitch input','Roll input','Yaw input');
set(gca,'xtick',x_tick);
set(gca,'ytick',[-1, 0 1]);
axis([x_tick(1) x_tick(end) -1 1]);
grid on;
h = gca;
set(h,'LineWidth',1,'GridLineStyle','--','GridAlpha',1,'GridColor',[0.9,0.9,0.9])
hold off;

subplot(7,1,7);
hold on;
Torque = Total_wing_torque_S{1} + Total_rudder_torque_S{1} + Total_tail_torque_S{1};
plot(Torque(:,1),"-",'LineWidth',LineWidth ,'Color',DarkRed);
plot(Torque(:,2),"-",'LineWidth',LineWidth ,'Color',DarkGreen);
plot(Torque(:,3),"-",'LineWidth',LineWidth ,'Color',DarkBlue);
% legend('Pitch input','Roll input','Yaw input');
set(gca,'xtick',x_tick);
set(gca,'ytick',[-0.1, 0 0.1]);
axis([x_tick(1) x_tick(end) -0.1 0.1]);
grid on;
h = gca;
set(h,'LineWidth',1,'GridLineStyle','--','GridAlpha',1,'GridColor',[0.9,0.9,0.9])
hold off;



set(gcf,'Position',[400,100,600,700]);
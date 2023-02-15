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
x_tick =0 + [0 100 200 300 400] * 5;
% x_tick = [0 2000 4000 6000 8000 10000 12000];

Experiment_name = 'SensorFusion_';
AngularVelocity = 'DW_0_0_0';
%0_n05pi_0
Rear_name = '_Attitude_Tracking_File.mat';
load([Experiment_name, AngularVelocity, Rear_name])
LineWidth = 0.8;
Emph = 1.5;
BasisRotation = [1, 0, 0;...
                 0, 0, 1;...
                 0,-1, 0]';%Because the inertia rotation matrix in webots
             %is y pointing up

Colors = [LighGrayRed; LighGrayGreen_2; LighGrayBlue_2]


figure;
for i =1:3
subplot(3,1,i);
hold on;
plot(Total_omega_meas(:,i),"-",'LineWidth',LineWidth,'Color',Colors(i,:));
plot(Total_omega_meas_F(:,i),"-",'LineWidth',LineWidth,'Color',Colors(i,:) * 0.4);
set(gca,'xtick',x_tick);
% set(gca,'ytick',[-30,  0,  30]);
% set(gca,'YLim',[-30 30])
grid on;
h = gca;
set(h,'LineWidth',1,'GridLineStyle','--','GridAlpha',1,'GridColor',[0.9,0.9,0.9])
hold off;
end

set(gcf,'unit','centimeters','position',[10 10 5 10])







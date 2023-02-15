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

Experiment_name = 'SensorFusion_Drop_';
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

MidnightBlue = [25, 25,112] /255;
DarkGreen = [0 100 0] /255;
HotPink4 = [139 58 98] /255;
MediumPurple4 = [93 71 139] /255;
LightSlateGray = [119 136 153] /255;
Firebrick2 = [238 44 44]/255;
DarkGoldenrod = [184 134 11]/255;
DarkKhaki     = [189 183 107]/255;
	
Colors = [LightSlateGray; MidnightBlue; DarkGreen; HotPink4; MediumPurple4];

Total_body_rotation_a = permute( Total_body_rotation,[2,3,1]);
Total_body_quat = rotm2quat(Total_body_rotation_a);

length = size(Total_body_rotation,1);
% for i = 1 : length
%     if (Total_Attitude_filter_quat(i,1) <0)
%         Total_Attitude_filter_quat(i,:) = -Total_Attitude_filter_quat(i,:);
%     end
% end
% 
% for i = 1 : length
%     if (Total_body_quat(i,1) <0)
%         Total_body_quat(i,:) = -Total_body_quat(i,:);
%     end
% end
% 
% for i = 1 : length
%     if (Total_Attitude_filter_A15MF(i,1) <0)
%         Total_Attitude_filter_A15MF(i,:) = -Total_Attitude_filter_A15MF(i,:);
%     end
% end
% 
% for i = 1 : length
%     if (Total_Attitude_filter_CMF_quat(i,1) <0)
%         Total_Attitude_filter_CMF_quat(i,:) = -Total_Attitude_filter_CMF_quat(i,:);
%     end
% end
% 
for i = 1 : length
    if (Total_Attitude_filter_EKF_quat(i,1) <0)
        Total_Attitude_filter_EKF_quat(i,:) = -Total_Attitude_filter_EKF_quat(i,:);
    end
end

% Total_Attitude_filter_quat(Total_Attitude_filter_quat(:,1) <0) = -Total_Attitude_filter_quat(Total_Attitude_filter_quat(:,1) <0);
% Total_body_quat(Total_body_quat(:,3) <0) = -Total_body_quat(Total_body_quat(:,3) <0);
% Total_Attitude_filter_A15MF(Total_Attitude_filter_A15MF(:,1) <0) = -Total_Attitude_filter_A15MF(Total_Attitude_filter_A15MF(:,1) <0);
% Total_Attitude_filter_CMF_quat(Total_Attitude_filter_CMF_quat(:,1) <0) = -Total_Attitude_filter_CMF_quat(Total_Attitude_filter_CMF_quat(:,1) <0);
% Total_Attitude_filter_EKF_quat(Total_Attitude_filter_EKF_quat(:,1) <0) = -Total_Attitude_filter_EKF_quat(Total_Attitude_filter_EKF_quat(:,1) <0);
% Total_body_quat(Total_body_quat(:,1) <0) = -Total_body_quat(Total_body_quat(:,1) <0);

% Total_Attitude_filter_quat = quatmultiply(Total_Flapper_Osc_learner_quat, Total_Attitude_filter_quat);

% Total_Attitude_filter_quat = quatmultiply( Total_Flapper_Osc_learner_quat, Total_Attitude_filter_quat);

figure;
for i =1:4
subplot(4,1,i);
hold on;
plot(Total_body_quat(:,i),"--",'LineWidth',LineWidth,'Color',LightSlateGray);
plot(Total_Attitude_filter_A15MF(:,i),"-.",'LineWidth',LineWidth,'Color',MidnightBlue);
plot(Total_Attitude_filter_CMF_quat(:,i),"-",'LineWidth',LineWidth,'Color',DarkGreen);
plot(Total_Attitude_filter_EKF_quat(:,i),"-",'LineWidth',LineWidth,'Color',MediumPurple4);
plot(Total_Attitude_filter_quat(:,i),"-",'LineWidth',LineWidth * Emph,'Color',Firebrick2);
if i==1
    legend('Ground truth', 'Verboom', 'Madgwick', 'EKF', 'Proposed', 'orientation', 'horizontal')
end
set(gca,'xtick',x_tick);
% set(gca,'ytick',[-30,  0,  30]);
% set(gca,'YLim',[-30 30])
grid on;
h = gca;
set(h,'LineWidth',1,'GridLineStyle','--','GridAlpha',1,'GridColor',[0.9,0.9,0.9])
hold off;
end

set(gcf,'unit','centimeters','position',[5 5 12 12])

Total_Attitude_filter_A15MF_dual = Total_Attitude_filter_A15MF;
Total_Attitude_filter_CMF_quat_dual = Total_Attitude_filter_CMF_quat;
Total_Attitude_filter_EKF_quat_dual = Total_Attitude_filter_EKF_quat;
Total_Attitude_filter_quat_dual = Total_Attitude_filter_quat;

Total_Attitude_filter_A15MF_dual(:,2:4) = -Total_Attitude_filter_A15MF_dual(:,2:4);
Total_Attitude_filter_CMF_quat_dual(:,2:4) = -Total_Attitude_filter_CMF_quat_dual(:,2:4);
Total_Attitude_filter_EKF_quat_dual(:,2:4) = -Total_Attitude_filter_EKF_quat_dual(:,2:4);
Total_Attitude_filter_quat_dual(:,2:4) = -Total_Attitude_filter_quat_dual(:,2:4);

A15MF_error_quat = quatmultiply(Total_Attitude_filter_A15MF_dual, Total_body_quat);
CMF_error_quat   = quatmultiply(Total_Attitude_filter_CMF_quat_dual, Total_body_quat);
EKF_error_quat   = quatmultiply(Total_Attitude_filter_EKF_quat_dual, Total_body_quat);
Pro_error_quat   = quatmultiply(Total_Attitude_filter_quat_dual, Total_body_quat);

A15MF_error = zeros(1, length);
CMF_error = zeros(1, length);
EKF_error = zeros(1, length);
Pro_error = zeros(1, length);
for i = 1 : length
    A15MF_error(i) = A15MF_error_quat(i,2:4) * A15MF_error_quat(i,2:4)';
    CMF_error(i) = CMF_error_quat(i,2:4) * CMF_error_quat(i,2:4)';
    EKF_error(i) = EKF_error_quat(i,2:4) * EKF_error_quat(i,2:4)';
    Pro_error(i) = Pro_error_quat(i,2:4) * Pro_error_quat(i,2:4)';
end

disp(['RMS_of_Verboom_error: ',num2str(rms(A15MF_error(500:end)))])
disp(['RMS_of_Madgwick_error: ',num2str(rms(CMF_error(500:end)))])
disp(['RMS_of_EKF_error: ',num2str(rms(EKF_error(500:end)))])
disp(['RMS_of_Pro_error: ',num2str(rms(Pro_error(500:end)))])

figure;
% subplot(2,1,1);
% hold on;
% plot(A15MF_error,"-",'LineWidth',LineWidth *1,'Color',LightSlateGray);
% plot(Pro_error,"-",'LineWidth',LineWidth *1,'Color',HotPink4);
% grid on;
% h = gca;
% set(h,'LineWidth',1,'GridLineStyle','--','GridAlpha',1,'GridColor',[0.9,0.9,0.9])
% legend('Ground truth', 'Estimated')
% hold off;



% subplot(2,1,2);
hold on;
plot(Total_Flapper_Freq,"--",'LineWidth',LineWidth *3,'Color',LightSlateGray);
plot(Total_Flapper_Osc_learner_F_est,"-",'LineWidth',LineWidth *1,'Color',HotPink4);
grid on;
h = gca;
set(h,'LineWidth',1,'GridLineStyle','--','GridAlpha',1,'GridColor',[0.9,0.9,0.9])
legend('Ground truth', 'Estimated')
hold off;
set(gcf,'unit','centimeters','position',[5 5 12 5])

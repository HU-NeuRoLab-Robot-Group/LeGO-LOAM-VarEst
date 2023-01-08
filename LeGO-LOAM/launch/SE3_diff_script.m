clear, close all,

%% asdasd

T = csvread('global_pose.csv');

[T_size, ~] = size(T);

for i=1:T_size
    if uint64(T(i,1))>1564719187494920492
       T = T(1:i,:);
       break;
    end
end

[T_size, ~] = size(T);

pose_bag = rosbag('diff_odom_mulran.bag');

pose_map = select(pose_bag,'Topic','/aft_mapped_to_init');
pose_flat = select(pose_bag,'Topic','/flat_odometry');
pose_cov = select(pose_bag,'Topic','/cov_odometry');
% pose_integ = select(pose_bag,'Topic','/integrated_to_init_with_covariance');

pose_map_msg = readMessages(pose_map,'DataFormat','struct');
pose_flat_msg = readMessages(pose_flat,'DataFormat','struct');
pose_cov_msg = readMessages(pose_cov,'DataFormat','struct');
% pose_integ_msg = readMessages(pose_integ,'DataFormat','struct');

exp_time = 10^9;

[map_size, ~] = size(pose_map_msg);
[flat_size, ~] = size(pose_flat_msg);
[cov_size, ~] = size(pose_cov_msg);
% [integ_size, ~] = size(pose_integ_msg);

MAP = zeros(3,map_size);
FLAT = zeros(3,flat_size);
COV = zeros(3,1);
% INTEG = zeros(3,integ_size);

MAP_t = zeros(1,map_size,'uint64');
FLAT_t = zeros(1,flat_size,'uint64');
COV_t = zeros(1,1,'uint64');
% INTEG_t = zeros(1,integ_size,'uint64');

for i = 1: map_size
   MAP(1,i) = pose_map_msg{i}.Pose.Pose.Position.X;
   MAP(2,i) = pose_map_msg{i}.Pose.Pose.Position.Y;
   MAP(3,i) = pose_map_msg{i}.Pose.Pose.Position.Z;
   MAP_t(1,i) = uint64(pose_map_msg{i}.Header.Stamp.Sec)*10^9+uint64(pose_map_msg{i}.Header.Stamp.Nsec);
   if (MAP_t(1,i) > 1564719187494920492)
       MAP = MAP(:,1:i);
       MAP_t = MAP_t(:,1:i);
      break; 
   end
end

MAP = transpose(unique(transpose(MAP),'stable','rows'));
[~,map_size] = size(MAP);

for i = 1: flat_size
   FLAT(1,i) = pose_flat_msg{i}.Pose.Pose.Position.X;
   FLAT(2,i) = pose_flat_msg{i}.Pose.Pose.Position.Y;
   FLAT(3,i) = pose_flat_msg{i}.Pose.Pose.Position.Z;
   FLAT_t(1,i) = uint64(pose_flat_msg{i}.Header.Stamp.Sec)*10^9+uint64(pose_flat_msg{i}.Header.Stamp.Nsec);
%    if((i>1)&&(FLAT(4,i) == FLAT(4,i-1)))
%        FLAT(4,i) = FLAT(4,i)+1;
%    end
   if (FLAT_t(1,i) > 1564719187494920492)
       FLAT = FLAT(:,1:i);
       FLAT_t = FLAT_t(:,1:i);
      break; 
   end
end
% FLAT = transpose(unique(transpose(FLAT),'stable','rows'));
% FLAT_t = transpose(unique(transpose(FLAT_t),'stable','rows'));

for i = 1: cov_size
    t = (uint64(pose_cov_msg{i}.Header.Stamp.Sec)*10^9)+uint64(pose_cov_msg{i}.Header.Stamp.Nsec);
    if (i==1)
       COV = [pose_cov_msg{i}.Pose.Pose.Position.X;
                   pose_cov_msg{i}.Pose.Pose.Position.Y;
                   pose_cov_msg{i}.Pose.Pose.Position.Z];
       COV_t = t;
       k = 1;        
    elseif((i>1)&&(t <=COV_t(1,k)))
        continue;
    else
       COV = [COV  [pose_cov_msg{i}.Pose.Pose.Position.X;
                   pose_cov_msg{i}.Pose.Pose.Position.Y;
                   pose_cov_msg{i}.Pose.Pose.Position.Z]];
       COV_t = [COV_t t];
       k = k+1;
    end
   if (COV_t(1,k) > 1564719187494920492)
       COV = COV(:,1:k);
       COV_t = COV_t(:,1:k);
      break; 
   end
end

figure(1)
subplot(2,2,1)
plot(COV(3,:),COV(1,:),'LineWidth',2)
hold on, box on,
plot(FLAT(3,:),FLAT(1,:))
plot(MAP(3,:),MAP(1,:))
plot(-T(:,9)+T(1,9),T(:,5)-T(1,5))
%plot(INTEG(3,:),INTEG(1,:))
title('XY graph'),xlabel('x'), ylabel('y'),
legend('EKF with Hausdorff','EKF','Map Trajectory','Ground Truth')

subplot(2,2,2)
plot(COV_t(1,:)-MAP_t(1,1),COV(3,:),'LineWidth',2)
hold on, box on,
plot(FLAT_t(1,:)-MAP_t(1,1),FLAT(3,:))
plot(MAP_t(1,:)-MAP_t(1,1),MAP(3,:))
plot(T(:,1)-double(MAP_t(1,1)),-T(:,9)+T(1,9))
%plot(INTEG(4,:),INTEG(3,:))
title('t vs X graph'),xlabel('t'), ylabel('x'),
legend('EKF with Hausdorff','EKF','Map Trajectory','Ground Truth')

subplot(2,2,3)
plot(COV_t(1,:)-MAP_t(1,1),COV(1,:),'LineWidth',2)
hold on, box on,
plot(FLAT_t(1,:)-MAP_t(1,1),FLAT(1,:))
plot(MAP_t(1,:)-MAP_t(1,1),MAP(1,:))
plot(T(:,1)-double(MAP_t(1,1)),T(:,5)-T(1,5))
%plot(INTEG(4,:),INTEG(1,:))
title('t vs Y graph'),xlabel('t'), ylabel('y'),
legend('EKF with Hausdorff','EKF','Map Trajectory','Ground Truth')

subplot(2,2,4)
plot(COV_t(1,:)-MAP_t(1,1),COV(2,:),'LineWidth',2)
hold on, box on,
plot(FLAT_t(1,:)-MAP_t(1,1),FLAT(2,:))
plot(MAP_t(1,:)-MAP_t(1,1),MAP(2,:))
plot(T(:,1)-double(MAP_t(1,1)),T(:,13)-T(1,13))
%plot(INTEG(4,:),INTEG(2,:))
title('t vs Z graph'),xlabel('t'), ylabel('z'),
legend('EKF with Hausdorff','EKF','Map Trajectory','Ground Truth')

%% interp1 map

Flat = zeros(4,map_size);
Flat(1,:) = interp1(double(FLAT_t(1,:)),FLAT(1,:),double(MAP_t(1,:)),'spline');
Flat(2,:) = interp1(double(FLAT_t(1,:)),FLAT(2,:),double(MAP_t(1,:)),'spline');
Flat(3,:) = interp1(double(FLAT_t(1,:)),FLAT(3,:),double(MAP_t(1,:)),'spline');
Flat(4,:) = double(MAP_t(1,:));

Cov = zeros(4,map_size);
Cov(1,:) = interp1(double(COV_t(1,:)),COV(1,:),double(MAP_t(1,:)),'spline');
Cov(2,:) = interp1(double(COV_t(1,:)),COV(2,:),double(MAP_t(1,:)),'spline');
Cov(3,:) = interp1(double(COV_t(1,:)),COV(3,:),double(MAP_t(1,:)),'spline');
Cov(4,:) = double(MAP_t(1,:));

cov_error = zeros(2,map_size);
for i = 1:map_size
    cov_error(1,i) = rms(sqrt((Cov(1,1:i)-MAP(1,1:i)).^2+(Cov(2,1:i)-MAP(2,1:i)).^2+(Cov(3,1:i)-MAP(3,1:i)).^2));
    cov_error(2,i) = sqrt((Cov(1,i)-MAP(1,i)).^2+(Cov(2,i)-MAP(2,i)).^2+(Cov(3,i)-MAP(3,i)).^2);
end

flat_error = zeros(2,map_size);
for i = 1:map_size
    flat_error(1,i) = rms(sqrt((Flat(1,1:i)-MAP(1,1:i)).^2+(Flat(2,1:i)-MAP(2,1:i)).^2+(Flat(3,1:i)-MAP(3,1:i)).^2));
    flat_error(2,i) = sqrt((Flat(1,i)-MAP(1,i)).^2+(Flat(2,i)-MAP(2,i)).^2+(Flat(3,i)-MAP(3,i)).^2);
end
figure(2)
subplot(2,2,1)
plot(MAP_t(1,:),cov_error(1,:))
hold on, box on,
plot(MAP_t(1,:),flat_error(1,:))
title('RMS Error wrt. Map vs Time(n)'),xlabel('t'), ylabel('rms(e)'),
legend('EKF with Hausdorff','EKF')
subplot(2,2,2)
plot(MAP_t(1,:),cov_error(2,:))
hold on, box on,
plot(MAP_t(1,:),flat_error(2,:))
title('Absolute Error wrt. Map vs Time(ns)'),xlabel('t'), ylabel('abs(e)'),
legend('EKF with Hausdorff','EKF')

%% interp1 GT

T(:,2:end) = T(:,2:end) - T(1,2:end);

Flat = zeros(4,T_size);
Flat(1,:) = interp1(double(FLAT_t(1,:)),FLAT(1,:),double(T(:,1)),'spline');
Flat(2,:) = interp1(double(FLAT_t(1,:)),FLAT(2,:),double(T(:,1)),'spline');
Flat(3,:) = interp1(double(FLAT_t(1,:)),FLAT(3,:),double(T(:,1)),'spline');
Flat(4,:) = double(T(:,1));

Cov = zeros(4,T_size);
Cov(1,:) = interp1(double(COV_t(1,:)),COV(1,:),double(T(:,1)),'spline');
Cov(2,:) = interp1(double(COV_t(1,:)),COV(2,:),double(T(:,1)),'spline');
Cov(3,:) = interp1(double(COV_t(1,:)),COV(3,:),double(T(:,1)),'spline');
Cov(4,:) = double(T(:,1));

cov_error_gt = zeros(2,T_size);
for i = 1:T_size
    cov_error_gt(1,i) = rms(sqrt((Cov(1,1:i)-transpose(T(1:i,5))).^2+(Cov(2,1:i)-transpose(T(1:i,13))).^2+(Cov(3,1:i)+transpose(T(1:i,9)).^2)));
    cov_error_gt(2,i) = sqrt((Cov(1,i)-T(i,5)).^2+(Cov(2,i)-T(i,13)).^2+(Cov(3,i)+T(i,9)).^2);
end

flat_error_gt = zeros(2,T_size);
for i = 1:T_size
    flat_error_gt(1,i) = rms(sqrt((Flat(1,1:i)-transpose(T(1:i,5))).^2+(Flat(2,1:i)-transpose(T(1:i,13))).^2+(Flat(3,1:i)+transpose(T(1:i,9)).^2)));
    flat_error_gt(2,i) = sqrt((Flat(1,i)-T(i,5)).^2+(Flat(2,i)-T(i,13)).^2+(Flat(3,i)+T(i,9)).^2);
end
subplot(2,2,3)
plot(T(:,1)- double(MAP_t(1)),cov_error_gt(1,:))
hold on, box on,
plot(T(:,1)- double(MAP_t(1)),flat_error_gt(1,:))
title('RMS Error wrt Ground Truth vs Time(ns)'),xlabel('t'), ylabel('rms(e)'),
legend('EKF with Hausdorff','EKF')
subplot(2,2,4)
plot(T(:,1)- double(MAP_t(1)),cov_error_gt(2,:))
hold on, box on,
plot(T(:,1)- double(MAP_t(1)),flat_error_gt(2,:))
title('Absolute Error wrt Ground Truth vs Time(ns)'),xlabel('t'), ylabel('abs(e)'),
legend('EKF with Hausdorff','EKF')
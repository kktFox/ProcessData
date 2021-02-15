
% bagInfo = rosbag('info','filePath')
% filePath = fullfile('matlab','bagname');
% bagSelect = rosbag(filePath);
bag = rosbag('filepath');
bagselect1 = select(bag,'Topic','/topic1');
topic1 = readMessages(bagselect1,'DataFormat','struct');
bagselect2 = select(bag,'Topic','/topic2');
topic2 = readMessages(bagselect2,'DataFormat','struct');

% points1_x = cellfun(@(m) double(m.Pose.Pose.Position.X),topic1);
% points1_y = cellfun(@(m) double(m.Pose.Pose.Position.Y),topic1);
% points2_x = cellfun(@(m) double(m.Pose.Pose.Position.X),topic2);
% points2_y = cellfun(@(m) double(m.Pose.Pose.Position.Y),topic2);


%figure;
% set(0,'defaultfigurecolor','w');
% %plot(points1_y,points1_x,'b',points2_y,points2_x,'r','LineWidth',2);
% 
% xlabel('y/m');
% ylabel('x/m');
% title('');
%legend('','');
%figure('Color','white');
% x = 1:1:37836;
% y = 1:1:38202;
% plot(x,points1_x,'Color',[0 0.4470 0.7410],'LineWidth',2);
% hold on;
% plot(y,points2_x,'Color',[0.8500 0.3250 0.0980],'LineWidth',2)

% xy1 = timeseries(bagselect1,'Pose.Pose.Position.X','Pose.Pose.Position.Y');
% xy1.time = xy1.time -1613024429;
% xy1.data(:,1) = xy1.data(:,1) - 697150;
% xy1.data(:,2) = xy1.data(:,2) - 4065400;
% 
% xy2 = timeseries(bagselect2,'Pose.Pose.Position.X','Pose.Pose.Position.Y');
% xy2.time = xy2.time -1613024429;
% xy2.data(:,1) = xy2.data(:,1) - 697150;
% xy2.data(:,2) = xy2.data(:,2) - 4065400;
% 
% plot(xy1.time,xy1.data(:,1),'blue',xy2.time,xy2.data(:,1),'red','LineWidth',2)
% grid on;
% 
% figure;
% plot(xy1.time,xy1.data(:,2),'blue',xy2.time,xy2.data(:,2),'red','LineWidth',2);
% grid on;
timestart = 1613116718;
q1 = timeseries(bagselect1,'Pose.Pose.Orientation.W','Pose.Pose.Orientation.X','Pose.Pose.Orientation.Y','Pose.Pose.Orientation.Z');
q2 = timeseries(bagselect2,'Pose.Pose.Orientation.W','Pose.Pose.Orientation.X','Pose.Pose.Orientation.Y','Pose.Pose.Orientation.Z');
qua1 = [q1.data(:,1) q1.data(:,2) q1.data(:,3) q1.data(:,4)];
qua2 = [q2.data(:,1) q2.data(:,2) q2.data(:,3) q2.data(:,4)];
[y1, p1, r1] = quat2angle(qua1,'ZYX');
[y2, p2, r2] = quat2angle(qua2,'ZYX');
angles1 = [(q1.time - timestart) y1 r1 p1];
angles2 = [(q2.time - timestart) y2 r2 p2];

maxindex = 707;
diffs = [];
ry =[];
my =[];

%Linear Interpolation
for i=1:1:maxindex
     k1= find(angles1(:,1)-i>0,1);
     k2= find(angles2(:,1)-i>0,1);
     rtk_yaw = angles1(k1-1,2) + (angles1(k1,2) - angles1(k1-1,2)) / (angles1(k1,1) - angles1(k1-1,1)) * (i - angles1(k1-1,1));
     msf_yaw = angles2(k2-1,2) + (angles2(k2,2) - angles2(k2-1,2)) / (angles2(k2,1) - angles2(k2-1,1)) * (i - angles2(k2-1,1));
     ry = [ry; rtk_yaw];
     my = [my; msf_yaw];
     diffs = [diffs;(rtk_yaw - msf_yaw)];
end
diffs = abs(diffs) * 180 / pi;
maxvalue = max(diffs);
minvalue = min(diffs) ;
meanvalue= mean(abs(diffs));
medianvalue = median(abs(diffs));
stdvalue = std(diffs);
rmsevalue = sqrt(mean((diffs).^2));
j = 1:1:maxindex;
set(0,'defaultfigurecolor','w');
plot(c,diffs,'LineWidth',2);
ylabel('yaw/^o');
xlabel('time/s');
title('yaw');
grid on;


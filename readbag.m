
% bagInfo = rosbag('info','/home/xx/xx.bag')
% filePath = fullfile('matlab','xx.bag');
% bagSelect = rosbag(filePath);
bag = rosbag('/home/xx/xx.bag');
bagselect1 = select(bag,'Topic','/nav_odom');
nav_odom = readMessages(bagselect1,'DataFormat','struct');
bagselect2 = select(bag,'Topic','/xx/filteredodometry');
nonloc = readMessages(bagselect2,'DataFormat','struct');

% points1_x = cellfun(@(m) double(m.Pose.Pose.Position.X),nav_odom);
% points1_y = cellfun(@(m) double(m.Pose.Pose.Position.Y),nav_odom);
% points2_x = cellfun(@(m) double(m.Pose.Pose.Position.X),nonloc);
% points2_y = cellfun(@(m) double(m.Pose.Pose.Position.Y),nonloc);
% points1_x = points1_x - 697150;
% points1_y = points1_y - 4065400;
% points2_x = points2_x - 697150 + 700000;
% points2_y = points2_y - 4065400 + 4000000;

timestart = 1616833473;
maxindex = 215;

q1 = timeseries(bagselect1,'Pose.Pose.Orientation.W','Pose.Pose.Orientation.X','Pose.Pose.Orientation.Y','Pose.Pose.Orientation.Z');
q2 = timeseries(bagselect2,'Pose.Pose.Orientation.W','Pose.Pose.Orientation.X','Pose.Pose.Orientation.Y','Pose.Pose.Orientation.Z');
qua1 = [q1.data(:,1) q1.data(:,2) q1.data(:,3) q1.data(:,4)];
qua2 = [q2.data(:,1) q2.data(:,2) q2.data(:,3) q2.data(:,4)];
[y1, p1, r1] = quat2angle(qua1,'ZYX');
[y2, p2, r2] = quat2angle(qua2,'ZYX');
angles1 = [(q1.time - timestart) y1 r1 p1];
angles2 = [(q2.time - timestart) y2 r2 p2];





%处理x,y
x1tmp = timeseries(bagselect1,'Pose.Pose.Position.X','Pose.Pose.Position.Y');
x2tmp = timeseries(bagselect2,'Pose.Pose.Position.X','Pose.Pose.Position.Y');
x1 = [(x1tmp.time - timestart) (x1tmp.data(:,1) - 697150) (x1tmp.data(:,2) - 4065400)];
x2 = [(x2tmp.time - timestart) (x2tmp.data(:,1) - 697150) (x2tmp.data(:,2) - 4065400)];


dxy = [];
dx = [];
dy = [];
rtkxs = [];
rtkys = [];
msfxs = [];
msfys = [];
meanxy = [];
% x方向误差
for i=1:1:maxindex
     k1= find(x1(:,1)-i>0,1);
     k2= find(x2(:,1)-i>0,1);
     rtkx = x1(k1-1,2) + (x1(k1,2) - x1(k1-1,2)) / (x1(k1,1) - x1(k1-1,1)) * (i - x1(k1-1,1));
     msfx = x2(k2-1,2) + (x2(k2,2) - x2(k2-1,2)) / (x2(k2,1) - x2(k2-1,1)) * (i - x2(k2-1,1));
     rtkxs = [rtkxs; rtkx];
     msfxs = [msfxs; msfx];
     dx = [dx;(rtkx - msfx)];
end
% y方向误差
for i=1:1:maxindex
     k1= find(x1(:,1)-i>0,1);
     k2= find(x2(:,1)-i>0,1);
     rtky = x1(k1-1,3) + (x1(k1,3) - x1(k1-1,3)) / (x1(k1,1) - x1(k1-1,1)) * (i - x1(k1-1,1));
     msfy = x2(k2-1,3) + (x2(k2,3) - x2(k2-1,3)) / (x2(k2,1) - x2(k2-1,1)) * (i - x2(k2-1,1));
     rtkys = [rtkys; rtky];
     msfys = [msfys; msfy];
     dy = [dy;(rtky - msfy)];
end
%xy 平移误差
dxy = sqrt(dx .* dx + dy .* dy); 

% 平移误差可视化
j = 1:1:maxindex;
set(0,'defaultfigurecolor','w');
figure(1);
plot(j,dx,'LineWidth',2);
ylabel('x方向误差/m');
xlabel('时间/s');
title('x方向误差');
grid on;
figure(2);
plot(j,dy,'LineWidth',2);
ylabel('y方向误差/m');
xlabel('时间/s');
title('y方向误差');
grid on;
figure(3);
plot(j,dxy,'LineWidth',2);
ylabel('平移误差/m');
xlabel('时间/s');
title('平移误差');
grid on;




diffs = [];
ry =[];
my =[];
for i=1:1:maxindex
     k1= find(angles1(:,1)-i>0,1);
     k2= find(angles2(:,1)-i>0,1);
     rtk_yaw = angles1(k1-1,2) + (angles1(k1,2) - angles1(k1-1,2)) / (angles1(k1,1) - angles1(k1-1,1)) * (i - angles1(k1-1,1));
     msf_yaw = angles2(k2-1,2) + (angles2(k2,2) - angles2(k2-1,2)) / (angles2(k2,1) - angles2(k2-1,1)) * (i - angles2(k2-1,1));
     ry = [ry; rtk_yaw];
     my = [my; msf_yaw];
     diffs = [diffs;(rtk_yaw - msf_yaw)];
end
diffs = diffs * 180 / pi;

maxvalue = max(abs(diffs));
minvalue = min(abs(diffs));
meanvalue= mean(abs(diffs));
medianvalue = median(abs(diffs));
stdvalue = std(diffs);
rmsevalue = sqrt(mean((diffs).^2));

%yaw可视化
j = 1:1:maxindex;
set(0,'defaultfigurecolor','w');
figure(4);
plot(j,diffs,'LineWidth',2);
ylabel('yaw角误差/^o');
xlabel('时间/s');
title('yaw角误差');
grid on;


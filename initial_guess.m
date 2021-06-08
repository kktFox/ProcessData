%%时间戳开始时间是1970年，查询bag时间开始节点，减去开始时间，使时间从0开始。
time_start = 1610438341;
start = 100;
maxind = 11000;
step = 100;
errx = [];
erry = [];
erryaw = [];
errxy = [];
%% 提取数据成矩阵形式

%lidar匹配得到的位置姿态信息
lidar_odom_yaw = NDTOMPodom.VarName2;
lidar_odom_x = NDTOMPodom.VarName3;
lidar_odom_y = NDTOMPodom.VarName4;

%一次航迹推算得到的初始位姿
DR_yaw = NDTOMPDRodom.VarName2;
DR_x = NDTOMPDRodom.VarName3;
DR_y = NDTOMPDRodom.VarName4;
dr = [NDTOMPDRodom.VarName1-1610438341, DR_x, DR_y, DR_yaw];

%RTK数据
rtk_yaw = rtkodom.VarName2;
rtk_x = rtkodom.VarName3;
rtk_y = rtkodom.VarName4;
rtk = [rtkodom.VarName1-1610438341, rtk_x, rtk_y, rtk_yaw];

y1 = linspace(0,1,length(NDTOMPDRodom.VarName4));
y2 = linspace(0,1,length(NDTOMPodom.VarName4));
y3 = linspace(0,1,length(rtkodom.VarName4));

dyaw = (lidar_odom_yaw - DR_yaw) *180/pi;
dx = lidar_odom_x - DR_x;
dy = lidar_odom_y - DR_y;

set(0,'defaultfigurecolor','w');
%{
plot(y1,NDTOMPDRodom.VarName4,'r','LineWidth',2);
hold on;
plot(y2,NDTOMPodom.VarName4,'g','LineWidth',2);
hold on;
plot(y3,rtkodom.VarName4,'b','LineWidth',2);
xlabel('匹配序列');
ylabel('误差/m');
title('平移误差');
grid on;
hold off;

figure;
plot(dy,'r','LineWidth',1);
xlabel('匹配序列');
ylabel('误差/m');
title('y方向误差');
grid on;

figure;
plot(dx,'b','LineWidth',1);
xlabel('匹配序列');
ylabel('误差/m');
title('x方向误差');
grid on;


figure;
plot(dyaw);
xlabel('匹配序列');
ylabel('误差/m');
title('角度误差');
grid on;
figure;
boxplot(dyaw);
%}

%% x,y,translation error
for i=start:step:maxind
    k1 = find(drt(:,1) - rtkt(i,1) > 0, 1);
    drx = dr(k1-1,2) + (dr(k1,2) - dr(k1-1,2)) / (dr(k1,1) - dr(k1-1,1)) * (rtk(i) - dr(k1-1,1));
    dry = dr(k1-1,3) + (dr(k1,3) - dr(k1-1,3)) / (dr(k1,1) - dr(k1-1,1)) * (rtk(i) - dr(k1-1,1));
    errx = [errx ;rtk(i,1) rtk(i,2)-drx];
    erry = [erry ;rtk(i,1) rtk(i,3)-dry];
    errxy = [errxy;rtk(i,1) sqrt((rtk(i,2)-drx)^2+(rtk(i,3)-dry)^2)]
end
figure;
plot(errx(:,1),errx(:,2),'r','LineWidth',1);
xlabel('匹配序列');
ylabel('误差/m');
title('x方向误差');
grid on;

figure;
plot(erry(:,1),erry(:,2),'b','LineWidth',1);
xlabel('匹配序列');
ylabel('误差/m');
title('y方向误差');
grid on;

figure;
plot(errxy(:,1),errxy(:,2),'g','LineWidth',1);
xlabel('匹配序列');
ylabel('误差/m');
title('translation error');
grid on;
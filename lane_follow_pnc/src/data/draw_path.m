%% 读取数据
[x0,y0]=textread("./dp_path.txt",'%n%n');
[x1,y1] = textread("./dp_interpolation.txt", '%n%n');
[x2,y2] = textread("./qp_path.txt", '%n%n');

%% 绘制路径图形
figure(1)
% dp_path
p1 = plot(x0+1, y0+1, LineWidth =2,LineStyle="--", Color='r');
hold on;
% dp_interpolation
p2 = plot(x1+1, y1+1, LineWidth =2, LineStyle=":", Color="g");
hold on;
% % qp_path
% % qp_upper
% plot(x2+1, y2+1+1.5, LineWidth =2, LineStyle="-.", Color='b');
% hold on;
% p3 = plot(x2+1, y2+1, LineWidth =2, LineStyle="-.", Color='b');
% hold on;
% % qp_lower
% plot(x2+1, y2-0.5, LineWidth =2, LineStyle="-.", Color='b');
% hold on;

%% 绘制障碍物和路径边界
%road_boundary
%road_x
plot([1,50], [6, 6]);
hold on;
%road_y
plot([0, 50], [-5, -5]);
hold on;
% obstacle
% rectangle(‘Position’,[x,y,w,h]); %在指定位置绘制矩形，其中x,y为矩形左下角坐标，w,h分别为矩形的长和宽。
% ‘edgecolor’,‘r’ 表示边框颜色是红色；
% ‘facecolor’,‘g’ 表示面内填充颜色为绿色。
rectangle('Position',[14 1 1 1],'edgecolor','k','facecolor','g','linewidth',2) 
axis equal

xlim([0,50]);
ylim([-9,10]);
%% 设置图例
% legend([p1,p2,p3], "dp", "interpolation", "qp");
legend([p1,p2], "dp", "interpolation");
xlabel('X/m');
ylabel('Y/m');


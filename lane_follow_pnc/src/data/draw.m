[x,y]=textread("./CubeTown.txt",'%n%n');
[smooth_x, smooth_y] = textread("./smooth.txt", '%n%n');

figure(1)
plot(x, y, LineStyle="--", Color='r');
hold on;
plot(smooth_x, smooth_y, LineStyle=":", Color="g");
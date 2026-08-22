data = readmatrix('data.csv');

x = data(:,2);
y = data(:,1);

h = 0.02;

vel = (y(1:end) - [0; y(1:end-1)])/h; % °/s
vel = vel*(pi/180); % rad/s
vel = vel*(60/(2*pi)); % rpm


t = (0:length(vel)-1)*h;

plot(t,vel)
plot(t,y)
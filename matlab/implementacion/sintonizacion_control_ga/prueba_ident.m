data = readmatrix('data_motor_1.txt');

x = data(:,3);
y = data(:,2);

Ts = 0.01;
t = (0:length(x)-1)*Ts;

figure()
plot(data(:,1),data(:,2));

K = 133.1107;
Tp1 = 0.19508;

G = tf(K,[Tp1, 1]);

ys = lsim(G, x, t);

figure()
plot(t, y)
hold on
plot(t, ys)
hold off

e = y - ys;
figure()
plot(t,e)
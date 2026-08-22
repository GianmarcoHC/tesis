data = readmatrix('datamotor.csv');
tiempo = data(:,1);
x = data(:,2);
y = data(:,3);

vel = (y(1:end) - [0;y(1:end-1)])/0.02;
vel = [0;vel];
vel = vel*60/360;

% 0.563947047745652 | 0.064097907765495

% Parámetros del sistema
K = 0.563947047745652;    % Ganancia
Tau = 0.064097907765495;  % Constante de tiempo (segundos)

% Definir la función de transferencia
s = tf('s');
H = K / (Tau * s + 1);

% Crear entrada escalón manualmente
t = 0:0.02:5;      % Vector de tiempo (de 0 a 10s con paso de 0.1s)
u = 255*ones(size(t)); % Entrada escalón unitario

% Simular la respuesta usando lsim()
[y, t_out] = lsim(H, u, t);

% Graficar la respuesta
figure;
plot(t_out, y, 'b', 'LineWidth', 2);
hold on;
plot(t_out, 255*K*ones(size(t_out)), 'r--', 'LineWidth', 1.5); % Línea de valor final
plot([0;tiempo]/1000, vel, 'k', 'LineWidth', 1.5); % Línea de datos experimentales
hold off;
grid on;
xlabel('Tiempo (s)');
ylabel('Salida');
title('Respuesta al Escalón de un Sistema de Primer Orden');
legend('Salida', 'Valor final K');

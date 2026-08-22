tiempo = out.tiempo;
vel = out.velocity; % 
pwm = 12*ones(size(vel)); %
%% MODELO DE PRIMER ORDEN
% Parámetros del sistema
K = P1.Kp;    % Ganancia
tau1 = P1.Tp1;  % Constante de tiempo (segundos)

% Definir la función de transferencia
s = tf('s');
H1 = K / (tau1 * s + 1);

% Crear entrada escalón manualmente
t = 0:0.01:5;      % Vector de tiempo (de 0 a 10s con paso de 0.1s)
u = 100*ones(size(t)); % Entrada escalón unitario

% Simular la respuesta usando lsim()
[y, t_out] = lsim(H1, u, t);

% Graficar la respuesta
figure(1);
plot(t_out, y, 'b', 'LineWidth', 2);
hold on;
plot(t_out, 100*K*ones(size(t_out)), 'r--', 'LineWidth', 1.5); % Línea de valor final
plot(tiempo/1000, vel, 'k', 'LineWidth', 1.5); % Línea de datos experimentales
hold off;
grid on;
xlabel('Tiempo (s)');
ylabel('Salida');
title('Respuesta al Escalón de un Sistema de Primer Orden');
legend('Salida', 'Valor final K');

%% MODELO DE SEGUNDO ORDEN
% Parámetros del sistema
K = P2.Kp;    % Ganancia
tau1 = P2.Tp1;  % Constante de tiempo (segundos)
tau2 = P2.Tp2;  % Constante de tiempo (segundos)

% Definir la función de transferencia
s = tf('s');
H2 = K / (tau1 * s + 1) / (tau2 * s + 1);

% Crear entrada escalón manualmente
t = 0:0.01:5;      % Vector de tiempo (de 0 a 10s con paso de 0.1s)
u = 100*ones(size(t)); % Entrada escalón unitario

% Simular la respuesta usando lsim()
[y, t_out] = lsim(H2, u, t);

% Graficar la respuesta
figure(2);
plot(t_out, y, 'b', 'LineWidth', 2);
hold on;
plot(t_out, 100*K*ones(size(t_out)), 'r--', 'LineWidth', 1.5); % Línea de valor final
plot(tiempo/1000, vel, 'k', 'LineWidth', 1.5); % Línea de datos experimentales
hold off;
grid on;
xlabel('Tiempo (s)');
ylabel('Salida');
title('Respuesta al Escalón de un Sistema de Primer Orden');
legend('Salida', 'Valor final K');

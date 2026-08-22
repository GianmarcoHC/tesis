
% Parámetros PID
Kp = 9.9518;
Ki = 0.0031;
Kd = 0.4645;

Kp = 7.2698;
Ki = 0.0088;
Kd = 0.4826;

% Modelo de planta
% Entrada: -1 a 1 (duty cycle PWM)
% Salida: °/seg
gain = 360/60; % Convertir de rpm a °/seg
Kp_p = 128.39 / 100; % dividir por 100 % para pwm de -100 a 100
Tp1 = 0.10067;
num = Kp_p * gain;
den = [Tp1 1 0];

% Tiempo de muestreo
Ts = 0.02;

% Convertir a espacio de estados y discretizar
[A, B, C, D] = tf2ss(num, den);
sys_d = c2d(ss(A, B, C, D), Ts, 'zoh');
[Ad, Bd, Cd, Dd] = ssdata(sys_d);

% Simulación de 20 segundos
t_end = 20;
t = 0:Ts:t_end;
N = length(t);

% Referencias escalonadas
omega_r = [360];
step_time = [0];
ref = zeros(size(t));
for i = 1:length(step_time)
    ref(t >= step_time(i)) = omega_r(i);
end

% Inicialización
x = zeros(size(Ad,1),1);
u = zeros(size(t));
y = zeros(size(t));
e = zeros(size(t));
e_prev = 0;
ie = 0;

DE = zeros(size(t));
IE = zeros(size(t));

% Inicialización de criterios
itse = 0;
ise = 0;

% Simulación discreta con controlador PID
for k = 1:N
    y(k) = Cd * x + Dd * u(k);
    e(k) = ref(k) - y(k);

    % Término derivativo e integral
    de = (e(k) - e_prev)/Ts;
    e_prev = e(k);
    ie = ie + e(k)*Ts;

    DE(k) = de;
    IE(k) = ie;

    % Señal de control PID
    u_pid = Kp*e(k) + Ki*ie + Kd*de;

    % Saturación de control
    u(k) = max(min(u_pid, 100), -100);
    u(k) = round(u(k), 2);
    
    % Actualización del sistema
    x = Ad * x + Bd * u(k);

    if abs(u(k)) < 9 % Deadzone del motor
        x = Ad * x;
    end

    % Calcular criterio ITSE
    itse = itse + Ts*(t(k)*(e(k)^2));
    ise = ise + Ts*((e(k))^2);
end

J = ise;

figure(1)
plot(t, y);
grid on; xlabel('t [s]'); ylabel('posición [°]'); title('Respuesta en posición');

figure(2)
%plot(Kp*e+Ki*IE+Kd*DE)
plot(u)

% 
% ise = ise_0 + Ts*((e(k))^2);
% ise_0 = ise; 
% 
% iae= iae_0 + Ts*abs(e(k));
% iae_0 = iae; 
% 
% itse= itse_0 + Ts*((e(k))^2)*t(k);
% itse_0 = itse; 
% 
% itae = itae_0 + Ts*abs(e(k))*t(k);
% itae_0 = itae;
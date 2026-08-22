clear; clc; close all;

% Parámetros del algoritmo genético
opts = optimoptions('ga', ...
    'PlotFcn', @gaplotbestf, ...
    'Display', 'final', ...
    'PopulationSize', 200, ...
    'CreationFcn', @gacreationuniform, ...
    'SelectionFcn', @selectionstochunif, ...
    'FitnessScalingFcn', @fitscalingrank, ...
    'EliteCount', 3, ...
    'CrossoverFcn', @crossoverscattered, ...
    'MutationFcn', @mutationadaptfeasible, ...
    'FunctionTolerance', 1e-6, ...
    'MaxStallGenerations', 50, ...
    'MaxGenerations', 420);

% Llamada al algoritmo genético
lb = [0 0 0];    % límites inferiores [Kp Ki Kd]
ub = [10 10 10];    % límites superiores (ajustables)
tic
[x, fval] = ga(@(X) objective(X), 3, [], [], [], [], lb, ub, [], opts);
toc

fprintf('\nResultados óptimos:\n');
fprintf('Kp = %.4f\n', x(1));
fprintf('Ki = %.4f\n', x(2));
fprintf('Kd = %.4f\n', x(3));
fprintf('Valor de la función objetivo (ITSE): %.6f\n', fval);


%% --- Función objetivo ---
function J = objective(X)
    % Parámetros PID
    Kp = X(1);
    Ki = X(2);
    Kd = X(3);

    % Modelo de planta
    % Entrada: -100 a 100 (duty cycle PWM)
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
    t_end = 4;
    t = 0:Ts:t_end;
    N = length(t);

    % Referencias escalonadas
    omega_r = [360 360*2 360*3 360*4 360*5];
    step_time = [0 1 2 3 4]*5;
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
        itse = itse + Ts * (t(k) * (e(k)^2));
        ise = ise + Ts*((e(k))^2);
    end

    % Valor final de la función objetivo
    J = ise;
end

x = [9.563 0.002 0.4449]
%% --- Simulación con los Kp Ki Kd óptimos ---
Kp = x(1);
Ki = x(2);
Kd = x(3);

%% Simular sistema PID
[t, y, ref, u] = sim_PID(Kp, Ki, Kd);

%% Extraer la parte correspondiente al último escalón
ref_final = ref(end);
step_start = find(ref == ref_final, 1, 'first');  % inicio del último cambio

t_step = t(step_start:end) - t(step_start);
y_step = y(step_start:end);
ref_step = ref(step_start:end);

% Calcular métricas
metrics = step_metrics(t_step, y_step, ref_step);

fprintf('\n--- Métricas del sistema ---\n');
fprintf('Sobreimpulso: %.2f %%\n', metrics.overshoot);
fprintf('Tiempo de establecimiento: %.3f s\n', metrics.settling_time);
fprintf('Tiempo de subida: %.3f s\n', metrics.rise_time);
fprintf('Error en estado estacionario: %.3f\n', metrics.ess);


%% Graficar resultados
figure(1);
plot(t, ref, '--k', 'LineWidth', 1.5); hold on;
plot(t, y, 'LineWidth', 2);
legend('Referencia','Salida');
xlabel('Tiempo (s)');
ylabel('Posición (°)');
%title('Respuesta del sistema con PID sintonizado');
grid on;

figure(2);
plot(t, u, 'LineWidth', 1.5);
xlabel('Tiempo (s)');
ylabel('Duty (%)');
%title('Señal de control');
grid on;


function [t, y, ref, u] = sim_PID(Kp, Ki, Kd)

    % ------ MISMO MODELO QUE EN TU FUNCIÓN OBJETIVO ------
    gain = 360/60; % Convertir de rpm a °/s
    Kp_p = 128.39 / 100;
    Tp1 = 0.10067;
    num = Kp_p * gain;
    den = [Tp1 1 0];
    
    Ts = 0.02;
    [A, B, C, D] = tf2ss(num, den);
    sys_d = c2d(ss(A,B,C,D), Ts, 'zoh');
    [Ad, Bd, Cd, Dd] = ssdata(sys_d);

    % -------- SIMULACIÓN --------
    t_end = 1.2;
    t = 0:Ts:t_end;
    N = length(t);

    % Referencias escalonadas
    omega_r = [360 720 1080 1440 1800];
    step_time = [0 1 2 3 4] * 5;
    
    ref = zeros(size(t));
    for i = 1:length(step_time)
        ref(t >= step_time(i)) = omega_r(i);
    end

    x = zeros(size(Ad,1),1);
    y = zeros(size(t));
    u = zeros(size(t));
    e_prev = 0;
    ie = 0;

    for k = 1:N
        y(k) = Cd*x + Dd*u(k);
        e = ref(k) - y(k);

        % PID discreto
        de = (e - e_prev)/Ts;
        e_prev = e;
        ie = ie + e*Ts;

        u_pid = Kp*e + Ki*ie + Kd*de;

        % Saturación
        u(k) = max(min(u_pid,100), -100);

        % Actualización de estados
        x = Ad*x + Bd*u(k);

        % Deadzone del motor
        if abs(u(k)) < 9
            x = Ad*x;
        end
    end
end


function metrics = step_metrics(t, y, ref)
    % Toma el ÚLTIMO escalón aplicado
    ref_final = ref(end);
    y_final = y(end);

    % --- Error en estado estacionario ---
    ess = ref_final - y_final;

    % --- Sobreimpulso (en %) ---
    ymax = max(y);
    overshoot = ((ymax - ref_final) / ref_final) * 100;

    % --- Tiempo de establecimiento (al 2%) ---
    tol = 0.02 * abs(ref_final);
    idx = find(abs(y - ref_final) <= tol, 1, 'first');
    if isempty(idx)
        settling_time = NaN;
    else
        settling_time = t(idx);
    end

    % --- Tiempo de subida (10% a 90%) ---
    y10 = 0.1 * ref_final;
    y90 = 0.9 * ref_final;

    idx10 = find(y >= y10, 1, 'first');
    idx90 = find(y >= y90, 1, 'first');

    if isempty(idx10) || isempty(idx90)
        rise_time = NaN;
    else
        rise_time = t(idx90) - t(idx10);
    end

    metrics.overshoot = overshoot;
    metrics.ess = ess;
    metrics.settling_time = settling_time;
    metrics.rise_time = rise_time;
end

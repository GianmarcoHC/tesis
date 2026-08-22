%% 
Kp = 1/2;%1/4; %0.02
Kd = 1/2;%1/2; %0.01
Ki = 1/2;%1/2; %0.01
x = [Kp;Kd;Ki];
A = objetive(x)

tic
format shortG;
rng default;
opts = optimoptions('ga',...
    'PlotFcn', @gaplotbestf,...
    'Display', 'final',...
    'Populationsize', 200,...
    'Creationfcn', @gacreationuniform, ...
        'Selectionfcn',@selectionstochunif, ...
        'FitnessScalingFcn', @fitscalingrank, ...
        'EliteCount', 3,...
    'CrossoverFcn', @crossoverscattered,...
    'MutationFcn', @mutationadaptfeasible, ...
    'FunctionTolerance', 1e-6,...
'MaxStallGenerations', 50,...
'Maxgeneration', 420);
toc

[x, fval] = ga(@objective, 3, [], [], [], [], [0 0 0], [2 1 1], [], opts)


function J = objetive(X, netc)

    % Parámetros PID
    Kp = X(1);
    Kd = X(2);
    Ki = X(3);

    % Configuración
    Ts = 0.01;
    t_end = 20;
    t = 0:Ts:t_end;

    % Referencia escalonada
    ref = zeros(size(t));
    ref(t>=0 & t<4)  = 1500;
    ref(t>=4 & t<8)  = 2000;
    ref(t>=8 & t<12) = 2500;
    ref(t>=12 & t<16)= 3000;
    ref(t>=16)       = 3500;

    % Inicialización
    y = zeros(size(t));
    u = zeros(size(t));
    e = zeros(size(t));
    e_prev = 0;
    ie = 0;

    % Buffers para delays (2 pasos de historia)
    u_buf = num2cell(zeros(1, netc.numInputDelays));
    y_buf = num2cell(zeros(1, netc.numLayerDelays));

    % Simulación
    for k = 2:length(t)

        % Error
        e(k) = ref(k) - y(k-1);

        % PID
        de = (e(k) - e_prev);
        e_prev = e(k);
        ie = ie + e(k);

        u(k) = Kp*e(k) + Kd*de + Ki*ie;

        % Actualizar buffers
        u_buf = [{u(k)} u_buf(1:end-1)];
        y_buf = [{y(k-1)} y_buf(1:end-1)];

        % Evaluar red (pasando historial completo)
        out = netc(u_buf, y_buf);
        y(k) = out{1};   % si es red con 1 sola salida


    end

    % Índice de desempeño (ITAE)
    J = sum(abs(e).*t);

end



%% 
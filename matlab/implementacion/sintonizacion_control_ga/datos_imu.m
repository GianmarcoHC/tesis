% limpiar
clear; clc; close all;

%% 1. Cargar datos
data = load('datos_imu.mat');   % ajusta el nombre si hace falta
out  = data.out;

%% Datos de la cámara
% orientación: 3 x 1 x N -> 3 x N
ori = squeeze(out.orientacion);   % [roll; pitch; yaw]
t   = out.tout(:);             % N x 1

roll_camara = ori(1,:);
pitch_camara = ori(2,:);

N = numel(t);

%% Datos de IMU
imu = out.imu;    

roll_imu  = imu(:,1);   % primera columna
pitch_imu = imu(:,2);   % segunda columna



%% Gráficas separadas
figure; hold on;
plot(t, roll_camara, 'LineWidth', 1.5); hold on;
plot(t, roll_imu, 'LineWidth', 1.5);
xlabel('Tiempo [s]');
ylabel('Ángulo [°]');
legend('Roll cámara','Roll IMU');
grid on;

figure; hold on;
plot(t, pitch_camara, 'LineWidth', 1.5); hold on;
plot(t, pitch_imu, 'LineWidth', 1.5);
xlabel('Tiempo [s]');
ylabel('Ángulo [°]');
legend('Pitch cámara','Pitch IMU');
grid on;


%% ================== ERRORES ==================
% Errores muestra a muestra
err_roll  = roll_imu  - roll_camara';
err_pitch = pitch_imu - pitch_camara';

N = length(err_roll);

mae_roll = mean(abs(err_roll),'all')
mae_pitch = mean(abs(err_pitch),'all')

rmse_roll = sqrt(mean(err_roll.^2,'all'))
rmse_pitch = sqrt(mean(err_pitch.^2,'all'))



% init_motor_v2.m - ARSIV: sensor asamasindaki parametreler (surtunme ve kompanzasyon oncesi)
% Orijinal kopya klasor duzenlenirken kayboldu, icerigi bu kayittan yeniden olusturuldu.
% Guncel parametreler icin models/init_motor.m dosyasina bakin.

% --- Motor ---
motor.J  = 0.03;    % kg.m^2
motor.B  = 0.01;    % N.m.s/rad
motor.KT = 0.5;     % N.m/A
motor.R  = 8;       % Ohm
motor.L  = 0.45;    % H
motor.KB = 0.5;     % V.s/rad

% --- Konum sensoru (enkoder) ---
sensor.q_enc     = 2*pi/4096;                          % 1024 CPR x 4
sensor.noise_std = 0.001;                              % rad
sensor.noise_Ts  = 0.001;                              % s
sensor.noise_cov = sensor.noise_std^2 * sensor.noise_Ts;
sensor.seed      = 23341;                              % tekrarlanabilirlik
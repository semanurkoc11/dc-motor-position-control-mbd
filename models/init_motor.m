% init_motor.m - DC motor, sensor ve kontrolcu parametreleri

% --- Motor ---
motor.J     = 0.03;    % kg.m^2
motor.B     = 0.01;    % N.m.s/rad
motor.KT    = 0.5;     % N.m/A
motor.R     = 8;       % Ohm
motor.L     = 0.45;    % H
motor.KB    = 0.5;     % V.s/rad
motor.Tc    = 0.05;    % N.m, Coulomb surtunme torku (durma torkunun ~%7'si, varsayim)
motor.w_eps = 0.01;    % rad/s, tanh yumusatma hizi

% --- Konum sensoru (enkoder) ---
sensor.q_enc     = 2*pi/4096;                          % 1024 CPR x 4
sensor.noise_std = 0.001;                              % rad
sensor.noise_Ts  = 0.001;                              % s
sensor.noise_cov = sensor.noise_std^2 * sensor.noise_Ts;
sensor.seed      = 23341;                              % tekrarlanabilirlik

% --- Kontrolcu: surtunme kompanzasyonu ---
ctrl.ff_oran = 1;                                          % 0: kapali, 1: nominal Tc'nin tamami
ctrl.Vc      = ctrl.ff_oran * motor.Tc * motor.R / motor.KT;  % V
ctrl.e0      = 0.01;                                       % rad, run_encoder_test ile secildi
% --- Kontrolcu: PID (kazanclar PID Tuner, N sensor testi ile secildi) ---
ctrl.Kp   = 11.2299151129167;
ctrl.Ki   = 7.23044369863269;
ctrl.Kd   = 4.16930526092153;
ctrl.N    = 50;      % turev filtresi (run_sensor_test ile secildi)
ctrl.Vmax = 12;      % V, surucu/besleme siniri

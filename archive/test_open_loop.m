model = 'close_pid_dc_motor_posit_velocity';
load_system(model);
init_motor;

tm = 'dc_motor_test';
if bdIsLoaded(tm), close_system(tm,0); end
new_system(tm); open_system(tm);
add_block('simulink/Sources/Step',  [tm '/Step'], 'Time','0','Before','0','After','1');
add_block([model '/dc_motor'],      [tm '/dc_motor']);
add_block('simulink/Sinks/To Workspace', [tm '/pos_out'], ...
    'VariableName','pos','SaveFormat','Timeseries');
add_line(tm,'Step/1','dc_motor/1');
add_line(tm,'dc_motor/1','pos_out/1');
set_param(tm,'InitFcn','init_motor;','StopTime','10');

out = sim(tm);
t = out.pos.Time;  p = out.pos.Data;

% Simulink'ten hız: konumun son 2 saniyedeki eğimi
i0 = find(t >= 8, 1);
w_sim = (p(end)-p(i0)) / (t(end)-t(i0));

% Teorik hız
w_th = motor.KT/(motor.R*motor.B + motor.KT*motor.KB);
fprintf('w_ss teorik = %.4f rad/s\nw_ss Simulink = %.4f rad/s\n', w_th, w_sim);

% State-space ile konum karşılaştırması
A = [-motor.R/motor.L, -motor.KB/motor.L, 0;
    motor.KT/motor.J, -motor.B/motor.J,  0;
    0, 1, 0];
sys = ss(A, [1/motor.L;0;0], eye(3), 0);
ts = linspace(0,10,2001);
x = step(sys, ts);
plot(t,p,'b', ts,x(:,3),'r--'); grid on;
legend('Simulink','State-space'); xlabel('s'); ylabel('konum (rad)');
title('Açık çevrim 1 V adım cevabı: model doğrulama');
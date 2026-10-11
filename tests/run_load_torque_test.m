% run_load_torque_test.m - REQ-09: kapali cevrimde yuk torku
% Motor 50 rad'da dururken t = 10 s'de yuk torku uygulanir.
% Modele dokunmaz: yuk SimulationInput ile yalnizca o kosu icin verilir.
model = 'dc_motor_position_control';
load_system(model);
init_motor;
lt  = [model '/Load_Torque'];
ref = 50;  t_d = 10;
amps = [0.1 0.2 0.4 0.6 0.7 0.8];          % N.m
R = nan(numel(amps), 6);

figure; hold on; grid on;
for i = 1:numel(amps)
    in = Simulink.SimulationInput(model);
    in = setModelParameter(in, 'StopTime', '30');
    in = setBlockParameter(in, [model '/Voltage_Disturbance'], 'After', '0');   % voltaj bozucusu yok
    in = setBlockParameter(in, lt, 'Time',   num2str(t_d));
    in = setBlockParameter(in, lt, 'Before', '0');
    in = setBlockParameter(in, lt, 'After',  num2str(amps(i)));
    out = sim(in);

    L   = out.logsout;
    pos = L.getElement('position_true_rad').Values;
    mv  = L.getElement('motor_voltage_V').Values;
    t = pos.Time;  e = ref - pos.Data(:);

    R(i,1) = amps(i);
    R(i,2) = max(abs(e(t >= t_d)));                 % en buyuk sapma
    R(i,3) = abs(mean(e(t >= 25)));                 % kalici hata (son 5 s ort.)
    R(i,4) = recov(t, e, 0.05, t_d);                % 0.05 rad bandina donus
    R(i,5) = mean(mv.Data(mv.Time >= 25));          % olculen kalici motor voltaji
    R(i,6) = amps(i) * motor.R / motor.KT;          % teorik tutma voltaji
    plot(t, pos.Data, 'DisplayName', sprintf('%.1f N.m', amps(i)));
    fprintf('%d/%d bitti: %.1f N.m\n', i, numel(amps), amps(i));
end
yline(ref, 'k--', 'HandleVisibility', 'off');
xline(t_d, 'r:', 'HandleVisibility', 'off');
xlim([8 30]); legend('Location', 'southwest');
xlabel('Zaman (s)'); ylabel('Konum (rad)'); title('Yuk torku testi (t = 10 s)');
exportgraphics(gcf, fullfile('results', 'yuk_torku_testi.png'), 'Resolution', 200);

T = array2table(R, 'VariableNames', {'Yuk_Nm', 'MaxSapma_rad', 'KaliciHata_rad', ...
    'Toparlanma_0p05_s', 'Vmotor_olculen_V', 'Vmotor_teori_V'});
disp(T)

function tr = recov(t, e, band, t_d)
idx = find(abs(e) > band & t >= t_d, 1, 'last');
if isempty(idx),        tr = 0;      % bandin disina hic cikmadi
elseif idx == numel(t), tr = NaN;    % banda hic donmedi
else,                   tr = t(idx) - t_d;
end
end
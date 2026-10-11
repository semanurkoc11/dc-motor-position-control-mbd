% run_disturbance_test.m - REQ-06b: voltaj bozucusu altinda kalici hata
% Modele dokunmaz: bozucu SimulationInput ile yalnizca bu kosu icin verilir.
model = 'dc_motor_position_control';
load_system(model);
dist = [model '/Voltage_Disturbance'];
ref = 50;  t_d = 10;  amp = 3;          % referans (rad), bozucu zamani (s), genligi (V)

in = Simulink.SimulationInput(model);
in = setModelParameter(in, 'StopTime', '25');
in = setBlockParameter(in, dist, 'Time',   num2str(t_d));
in = setBlockParameter(in, dist, 'Before', '0');
in = setBlockParameter(in, dist, 'After',  num2str(amp));
in = setBlockParameter(in, [model '/Load_Torque'], 'After', '0');   % yuk yok
out = sim(in);

L   = out.logsout;
pos = L.getElement('position_true_rad').Values;
pid = L.getElement('pid_out_V').Values;
pd  = resample(L.getElement('pid_minus_dist_V').Values, pid.Time);
mv  = resample(L.getElement('motor_voltage_V').Values,  pid.Time);

% --- Konum olcumleri ---
t = pos.Time;  e = ref - pos.Data;
k = t >= t_d;
dev  = max(abs(e(k)));                                  % en buyuk sapma
band = 0.02*dev;                                        % sapmanin %2'si
idx  = find(abs(e) > band & t >= t_d, 1, 'last');       % son band disi an
if isempty(idx),        trec = 0;                       % bandin disina hic cikmadi
elseif idx == numel(t), trec = NaN;                     % banda hic donmedi
else,                   trec = t(idx) - t_d;
end
fprintf('En buyuk sapma = %.3f rad\n', dev);
fprintf('Toparlanma (sapmanin %%2 bandi) = %.2f s\n', trec);
fprintf('Final konum hatasi (son 5 s ort.) = %.5f rad\n', abs(mean(e(t >= 20))));

% --- Voltaj olcumleri ---
kk = pid.Time >= t_d;
fprintf('pid_out_V (son 5 s ort.) = %.3f V (beklenen ~%g V)\n', mean(pid.Data(pid.Time >= 20)), amp);
fprintf('max|pid_out - pid_minus_dist - %g| = %.4f V (bozucu dogru mu?)\n', ...
    amp, max(abs(pid.Data(kk) - pd.Data(kk) - amp)));
fprintf('max|pid_minus_dist - motor_voltage| = %.4f V (dis doyma etkisi)\n', ...
    max(abs(pd.Data(kk) - mv.Data(kk))));

% --- Grafik ---
figure;
subplot(2,1,1); plot(t, pos.Data); yline(ref,'k--'); xline(t_d,'r:');
grid on; ylabel('konum (rad)'); title(sprintf('%g V voltaj bozucusu, t = %g s', amp, t_d));
subplot(2,1,2); plot(pid.Time, pid.Data); hold on; plot(mv.Time, mv.Data,'--');
xline(t_d,'r:'); ylim([-15 15]); grid on; ylabel('V'); xlabel('s');
legend('pid\_out\_V','motor\_voltage\_V');
exportgraphics(gcf, fullfile('results', 'bozucu_testi.png'), 'Resolution', 200);
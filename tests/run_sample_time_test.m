% run_sample_time_test.m - kontrolcu ornekleme suresi (Ts) taramasi (REQ-11)
% Ts buyudukce adim cevabi ve voltaj titresimi nasil degisir; gereksinimler hangi Ts'de bozulur?
% Sensor gurultusu 1 ms'de uretilmeye devam eder: sensor degismez, kontrolcu seyrek okur.
% Modele dokunmaz: Ts yalnizca o kosu icin SimulationInput ile verilir.
model = 'dc_motor_position_control';
load_system(model);
init_motor;
pid = [model '/PID Controller'];
zoh = [model '/Controller_Sample'];
ref = 50;  tt = (15:0.001:20)';                      % titresim penceresi (REQ-07 ile ayni)
Ts_list = [0.001 0.002 0.005 0.01 0.02 0.05 0.1 0.2 0.5];   % s

n = numel(Ts_list);  M = nan(n, 8);  P = cell(n, 1);
for i = 1:n
    Ts = Ts_list(i);
    in = Simulink.SimulationInput(model);
    in = setModelParameter(in, 'StopTime', '20');
    in = setBlockParameter(in, [model '/Voltage_Disturbance'], 'After', '0');
    in = setBlockParameter(in, [model '/Load_Torque'], 'After', '0');
    in = setBlockParameter(in, pid, 'SampleTime', num2str(Ts));
    in = setBlockParameter(in, zoh, 'SampleTime', num2str(Ts));
    out = sim(in);

    L   = out.logsout;
    pos = L.getElement('position_true_rad').Values;
    mv  = L.getElement('motor_voltage_V').Values;
    S   = stepinfo(pos.Data(:), pos.Time, ref);
    v   = resample(mv, tt).Data(:);
    ess = abs(ref - pos.Data(end));
    umax = max(abs(mv.Data));
    vac = std(v);  vtep = max(abs(v - mean(v)));
    ok  = ess < 0.5 && S.Overshoot < 5 && S.SettlingTime < 5 && ...
        umax <= ctrl.Vmax + 1e-6 && vac < 0.5 && vtep < 1.5;   % REQ-02..05 ve 07a
    M(i,:) = [Ts*1000, S.Overshoot, S.SettlingTime, ess, umax, vac, vtep, ok];
    P{i} = pos;                                                    % grafik icin sakla
    fprintf('%d/%d bitti: Ts = %g ms\n', i, n, Ts*1000);
end

T = array2table(M, 'VariableNames', {'Ts_ms', 'Asim_pct', 'Yerlesme_s', 'ess_rad', ...
    'Umax_V', 'Vac_V', 'Vac_tepe_V', 'PASS'});
T.PASS = logical(T.PASS);
disp(T)

% ===================== Grafik =====================
f  = figure('Position', [100 100 1100 750], 'Color', 'w');
tl = tiledlayout(f, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
mavi = [0.00 0.45 0.74];  kirmizi = [0.85 0.10 0.10];  gri = [0.65 0.65 0.65];

% (a) Konum: anlamli 4 Ts degeri
sec  = [0.001 0.01 0.1 0.2];                         % nominal, REQ-11a, son PASS, ilk FAIL
renk = [0.00 0.30 0.70          % koyu mavi
    0.00 0.62 0.45          % yesil
    0.93 0.55 0.00          % turuncu
    0.85 0.10 0.10];        % kirmizi
ax1 = nexttile(tl, [1 2]);  hold(ax1, 'on');  grid(ax1, 'on');
for j = 1:numel(sec)
    i = find(abs(Ts_list - sec(j)) < 1e-12, 1);
    if T.PASS(i), drm = 'PASS'; else, drm = 'FAIL'; end
    plot(ax1, P{i}.Time, P{i}.Data(:), 'Color', renk(j,:), 'LineWidth', 2, ...
        'DisplayName', sprintf('Ts = %g ms  (%s)', sec(j)*1000, drm));
end
yline(ax1, ref, 'k--', 'HandleVisibility', 'off');
xlim(ax1, [2 12]);  ylim(ax1, [ref-5 ref+5]);
xlabel(ax1, 'Zaman (s)');  ylabel(ax1, 'Konum (rad)');
legend(ax1, 'Location', 'southeast');
title(ax1, '(a) Hedef civarinda konum', 'FontWeight', 'normal');

% (b) Asim - Ts (tum degerler)
ok = T.PASS;
ax2 = nexttile(tl);  hold(ax2, 'on');  grid(ax2, 'on');
plot(ax2, T.Ts_ms, T.Asim_pct, '-', 'Color', gri, 'LineWidth', 1.2, 'HandleVisibility', 'off');
plot(ax2, T.Ts_ms(ok),  T.Asim_pct(ok),  'o', 'MarkerSize', 9, 'MarkerFaceColor', mavi, ...
    'MarkerEdgeColor', 'none', 'DisplayName', 'PASS');
plot(ax2, T.Ts_ms(~ok), T.Asim_pct(~ok), 'o', 'MarkerSize', 9, 'MarkerFaceColor', kirmizi, ...
    'MarkerEdgeColor', 'none', 'DisplayName', 'FAIL');
yline(ax2, 5, 'k--', 'REQ-03 siniri', 'HandleVisibility', 'off', 'LabelHorizontalAlignment', 'left');
xline(ax2, 10, ':', 'REQ-11a', 'HandleVisibility', 'off');
set(ax2, 'XScale', 'log');  xlim(ax2, [0.7 700]);
xlabel(ax2, 'Ornekleme suresi Ts (ms)');  ylabel(ax2, 'Asim (%)');
legend(ax2, 'Location', 'northwest');
title(ax2, '(b) Asim: 200 ms''de bile sinirin altinda', 'FontWeight', 'normal');

% (c) Voltaj titresimi - Ts (tum degerler)
ax3 = nexttile(tl);  hold(ax3, 'on');  grid(ax3, 'on');
plot(ax3, T.Ts_ms, T.Vac_V, '-', 'Color', gri, 'LineWidth', 1.2, 'HandleVisibility', 'off');
plot(ax3, T.Ts_ms(ok),  T.Vac_V(ok),  'o', 'MarkerSize', 9, 'MarkerFaceColor', mavi, ...
    'MarkerEdgeColor', 'none', 'HandleVisibility', 'off');
plot(ax3, T.Ts_ms(~ok), T.Vac_V(~ok), 'o', 'MarkerSize', 9, 'MarkerFaceColor', kirmizi, ...
    'MarkerEdgeColor', 'none', 'HandleVisibility', 'off');
yline(ax3, 0.5, 'k--', 'REQ-07a siniri', 'HandleVisibility', 'off', 'LabelHorizontalAlignment', 'left');
xline(ax3, 10, ':', 'REQ-11a', 'HandleVisibility', 'off');
set(ax3, 'XScale', 'log', 'YScale', 'log');  xlim(ax3, [0.7 700]);
xlabel(ax3, 'Ornekleme suresi Ts (ms)');  ylabel(ax3, 'Voltaj titresimi, std (V)');
title(ax3, '(c) Titresim: once azaliyor, 200 ms''de sicriyor', 'FontWeight', 'normal');

title(tl, 'Kontrolcu ornekleme suresi taramasi (REQ-11)');
set([ax1 ax2 ax3], 'FontSize', 11);
exportgraphics(f, fullfile('results', 'ornekleme_taramasi.png'), 'Resolution', 200);
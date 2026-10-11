% run_encoder_test.m - REQ-07: gurultusuz (gercekci) enkoder ve kompanzasyon e0 etkisi
model = 'dc_motor_position_control';
load_system(model);
qn  = [model '/Encoder_Quantizer'];
nz  = [model '/Sensor_Noise'];
pid = [model '/PID Controller'];
ie0 = [model '/Friction_Comp/inv_e0'];
vc  = [model '/Friction_Comp/Vc'];

%      ad                           gurultu cov          kompanzasyon   e0 (rad)
C = { 'enkoder, komp. kapali',      '0',                 '0',           '0.005'
    'enkoder, e0=0.005',          '0',                 'ctrl.Vc',     '0.005'
    'enkoder, e0=0.01',           '0',                 'ctrl.Vc',     '0.01'
    'enkoder, e0=0.02',           '0',                 'ctrl.Vc',     '0.02'
    'enk+gurultu, komp. kapali',  'sensor.noise_cov',  '0',           '0.005'
    'enk+gurultu, e0=0.005',      'sensor.noise_cov',  'ctrl.Vc',     '0.005'
    'enk+gurultu, e0=0.01',       'sensor.noise_cov',  'ctrl.Vc',     '0.01'
    'enk+gurultu, e0=0.02',       'sensor.noise_cov',  'ctrl.Vc',     '0.02' };

ref = 50;  tt = (15:0.001:20)';          % duragan pencere
n = size(C,1);  M = zeros(n, 6);  V = cell(n, 1);
for i = 1:n
    in = Simulink.SimulationInput(model);
    in = setModelParameter(in, 'StopTime', '20');
    in = setBlockParameter(in, qn,  'QuantizationInterval', 'sensor.q_enc');
    in = setBlockParameter(in, nz,  'Cov', C{i,2});
    in = setBlockParameter(in, pid, 'N', 'ctrl.N');
    in = setBlockParameter(in, vc,  'Gain', C{i,3});
    in = setBlockParameter(in, ie0, 'Gain', ['1/' C{i,4}]);
    out = sim(in);
    L   = out.logsout;
    pos = L.getElement('position_true_rad').Values;   % GERCEK konum
    mv  = L.getElement('motor_voltage_V').Values;

    S = stepinfo(pos.Data, pos.Time, ref);
    v = resample(mv, tt).Data(:);
    e = ref - resample(pos, tt).Data(:);
    M(i,:) = [S.Overshoot, S.SettlingTime, std(v), max(abs(v - mean(v))), ...
        mean(abs(e)), max(abs(e))];
    V{i} = mv;
    fprintf('%d/%d bitti: %s\n', i, n, C{i,1});
end

T = array2table(M, 'VariableNames', {'Asim_pct','Yerlesme_s','Vac_V','Vac_tepe_V', ...
    'Ort_hata_rad','Max_hata_rad'});
T.Durum = C(:,1);  T = movevars(T, 'Durum', 'Before', 1);
T.REQ07 = T.Vac_V < 0.5 & T.Vac_tepe_V < 1.5;
disp(T)
save(fullfile('results', 'encoder_test_sonuclari.mat'), 'T', 'V', 'C');

% Paneller: sol sutun gurultusuz enkoder, sag sutun enkoder + gurultu
figure('Position', [100 100 1100 800]);
tl = tiledlayout(4, 2, 'TileSpacing', 'compact');
for i = 1:n
    if i <= 4, tile = 2*i - 1; else, tile = 2*(i - 4); end
    nexttile(tile);
    plot(V{i}.Time, V{i}.Data, 'LineWidth', 0.8); grid on;
    xlim([18 18.3]); ylim([-8 8]);
    title({C{i,1}, sprintf('Vac = %.2f V', T.Vac_V(i))}, 'FontWeight', 'normal', 'FontSize', 9);
end
title(tl, 'N = 50: enkoder gurultusu ve kompanzasyon e0 etkisi (18.0 - 18.3 s)');
xlabel(tl, 'zaman (s)'); ylabel(tl, 'motor voltaji (V)');
exportgraphics(gcf, fullfile('results', 'enkoder_e0_testi.png'), 'Resolution', 200);
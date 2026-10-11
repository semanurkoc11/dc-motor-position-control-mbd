% run_sensor_test.m - REQ-07: sensor etkisi ve turev filtresi N
model = 'dc_motor_position_control';
load_system(model);
qn  = [model '/Encoder_Quantizer'];
nz  = [model '/Sensor_Noise'];
pid = [model '/PID Controller'];

%        ad                       nicemleme         gurultu cov          N
C = { 'ideal sensor',             '1e-9',           '0',                 '1010'
    'sadece enkoder',           'sensor.q_enc',   '0',                 '1010'
    'enkoder+gurultu N=1010',   'sensor.q_enc',   'sensor.noise_cov',  '1010'
    'enkoder+gurultu N=200',    'sensor.q_enc',   'sensor.noise_cov',  '200'
    'enkoder+gurultu N=50',     'sensor.q_enc',   'sensor.noise_cov',  '50'
    'enkoder+gurultu N=20',     'sensor.q_enc',   'sensor.noise_cov',  '20' };

ref = 50;  tt = (15:0.001:20)';
n = size(C,1);  M = zeros(n, 6);  V = cell(n, 1);

for i = 1:n
    in = Simulink.SimulationInput(model);
    in = setModelParameter(in, 'StopTime', '20');
    in = setBlockParameter(in, qn,  'QuantizationInterval', C{i,2});
    in = setBlockParameter(in, nz,  'Cov', C{i,3});
    in = setBlockParameter(in, pid, 'N',   C{i,4});
    out = sim(in);
    L   = out.logsout;
    pos = L.getElement('position_true_rad').Values;   % GERCEK konum
    mv  = L.getElement('motor_voltage_V').Values;

    S  = stepinfo(pos.Data, pos.Time, ref);
    v  = resample(mv,  tt).Data(:);
    e  = ref - resample(pos, tt).Data(:);
    M(i,:) = [S.Overshoot, S.SettlingTime, ...
        std(v), max(abs(v - mean(v))), sqrt(mean(e.^2)), max(abs(mv.Data))];
    V{i} = mv;                                   % grafik icin sakla
    fprintf('%d/%d bitti: %s\n', i, n, C{i,1});
end

T = array2table(M, 'VariableNames', {'Asim_pct','Yerlesme_s','Vac_V','Vac_tepe_V','Konum_rms_rad','Umax_V'});
T.Durum = C(:,1);
T = movevars(T, 'Durum', 'Before', 1);
disp(T)
save(fullfile('results', 'sensor_test_sonuclari.mat'), 'T', 'V', 'C');   % tekrar kosmadan cizebilmek icin
plot_sensor_results
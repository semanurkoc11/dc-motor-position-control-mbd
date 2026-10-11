% run_all_tests.m - tum dogrulama ve gereksinim testlerini calistirir,
% ozet tablo (test_ozeti.csv) ve tam rapor (test_raporu.txt) uretir.
% Hicbir test modeli degistirmez; en sonda bu da kontrol edilir.
close all; clc; diary off;
if exist(fullfile('results', 'test_raporu.txt'), 'file'), delete(fullfile('results', 'test_raporu.txt')); end
diary(fullfile('results', 'test_raporu.txt'));
fprintf('DC motor konum kontrolu - test raporu (%s)\n\n', char(datetime('now')));
ozet = struct();
mdl_ = 'dc_motor_position_control';

%% 0) Model dogrulama: acik cevrim, teoriye karsi
test_model_validation
ozet.val = dogrulama;  ozet.val_ss = ss_hata;

%% 1) Ic tutarlilik: motor denklemleri, kontrolcu zinciri, enkoder
check_logs
ozet.logs_ok = all(cellfun(@(a, b) max(abs(a - b)) < 1e-6, chk(:,2), chk(:,3))) && enc_ok;
ozet.umax    = max(abs(sat));

%% 2) Adim cevabi ve anti-windup (REQ-01..05)
run_antiwindup_test
r_ = T(strcmp(T.Yontem, 'back-calculation'), :);
ozet.asim = r_.Asim_pct;  ozet.yerlesme = r_.Yerlesme_s;  ozet.ess = r_.ess_rad;

%% 3) Bozucu testi (REQ-06b)
run_disturbance_test
ozet.dist_final = abs(mean(e(t >= 20)));

%% 4) Bozucu taramasi (REQ-06a, 06c), 9 V satiri
run_disturbance_sweep
r_ = T(T{:,1} == 9, :);
ozet.sapma9 = r_{1,2};  ozet.topar9 = r_{1,4};

%% 5) Parametre taramasi (REQ-08)
run_param_sweep
ozet.param_ok      = all(T.REQ02 & T.REQ03 & T.REQ04 & T.REQ05);
ozet.param_max_yer = max(T.Yerlesme_s);

%% 6) Sensor testleri (REQ-07)
run_sensor_test
r_ = T(strcmp(T.Durum, 'enkoder+gurultu N=50'), :);
ozet.vac_gur = r_.Vac_V;  ozet.vtep_gur = r_.Vac_tepe_V;
run_encoder_test
r_ = T(strcmp(T.Durum, 'enkoder, e0=0.01'), :);
ozet.vac_enk = r_.Vac_V;  ozet.vtep_enk = r_.Vac_tepe_V;

%% 7) Testler modeli degistirdi mi?
pid_ = [mdl_ '/PID Controller'];  blk_ = [mdl_ '/dc_motor'];
ozet.model_temiz = ...
    strcmp(get_param(blk_, 'R'), 'motor.R') && strcmp(get_param(blk_, 'J'), 'motor.J') && ...
    strcmp(get_param(pid_, 'AntiWindupMode'), 'back-calculation') && ...
    strcmp(get_param(pid_, 'N'), 'ctrl.N') && ...
    strcmp(get_param([mdl_ '/Voltage_Disturbance'], 'After'), '0') && ...
    strcmp(get_param([mdl_ '/Load_Torque'], 'After'), '0');

%% Ozet tablo
Q = {
    'MODEL-A', 'Surtunmesiz kalici hiz, teoriye bagil hata',  ozet.val(1).hata,   '< 0.005',    ozet.val(1).ok
    'MODEL-A2','Durum-uzayi konum egrisi, bagil hata',        ozet.val_ss,        '< 0.005',    ozet.val_ss < 0.005
    'MODEL-B', 'Coulomb surtunmesi, bagil hata',              ozet.val(2).hata,   '< 0.005',    ozet.val(2).ok
    'MODEL-C', 'Yuk torku girisi ve isareti, bagil hata',     ozet.val(3).hata,   '< 0.005',    ozet.val(3).ok
    'MODEL-D', 'Ic denklemler, kontrolcu zinciri, enkoder',   NaN,                'hepsi PASS', ozet.logs_ok
    'REQ-01',  'Hedefe ulasma (yerlesme gerceklesir)',        ozet.yerlesme,      'sonlu',      ~isnan(ozet.yerlesme)
    'REQ-02',  'Kalici hata (rad)',                           ozet.ess,           '< 0.5',      ozet.ess < 0.5
    'REQ-03',  'Asim (%)',                                    ozet.asim,          '< 5',        ozet.asim < 5
    'REQ-04',  'Yerlesme suresi (s)',                         ozet.yerlesme,      '< 5',        ozet.yerlesme < 5
    'REQ-05',  'Motor voltaji (V)',                           ozet.umax,          '<= 12',      ozet.umax <= 12 + 1e-6
    'REQ-06a', '9 V bozucuda en buyuk sapma (rad)',           ozet.sapma9,        '<= 1',       ozet.sapma9 <= 1
    'REQ-06b', '3 V bozucu sonrasi kalici hata (rad)',        ozet.dist_final,    '< 0.01',     ozet.dist_final < 0.01
    'REQ-06c', '9 V bozucuda 0.05 rad bandina donus (s)',     ozet.topar9,        '< 5',        ozet.topar9 < 5
    'REQ-07a', 'Voltaj titresimi, enkoder+gurultu (V)',       ozet.vac_gur,       '< 0.5',      ozet.vac_gur < 0.5 && ozet.vtep_gur < 1.5
    'REQ-07b', 'Voltaj titresimi, gurultusuz enkoder (V)',    ozet.vac_enk,       '< 0.5',      ozet.vac_enk < 0.5 && ozet.vtep_enk < 1.5
    'REQ-08',  'R, J +-%20: en kotu yerlesme (s)',            ozet.param_max_yer, 'REQ02-05',   ozet.param_ok
    'ALTYAPI', 'Testler sonrasi model ayarlari degismedi',    NaN,                'evet',       ozet.model_temiz };

fprintf('\n======================= GEREKSINIM OZETI =======================\n');
for k_ = 1:size(Q, 1)
    fprintf('%-9s %-43s %10.4g  %-10s %s\n', Q{k_,1}, Q{k_,2}, Q{k_,3}, Q{k_,4}, pf(Q{k_,5}));
end
fprintf('-----------------------------------------------------------------\n');
fprintf('GENEL SONUC: %s\n', pf(all(cell2mat(Q(:,5)))));

Ozet = cell2table(Q(:, 1:4), 'VariableNames', {'ID', 'Aciklama', 'Olculen', 'Sinir'});
Ozet.Sonuc = cellfun(@pf, Q(:,5), 'UniformOutput', false);
writetable(Ozet, fullfile('results', 'test_ozeti.csv'));
diary off
fprintf('\nRapor: results/test_raporu.txt, ozet: results/test_ozeti.csv\n');

function s = pf(c), if c, s = 'PASS'; else, s = 'FAIL'; end, end
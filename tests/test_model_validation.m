% test_model_validation.m - Motor modelinin bagimsiz dogrulamasi (acik cevrim, 1 V)
% A) Surtunmesiz: kalici hiz ve konum egrisi, durum-uzayi modeliyle karsilastirilir.
% B) Coulomb surtunmesi acik: kalici hiz, surtunmeli teorik formulle karsilastirilir.
% C) Yuk torku girisi: kalici hiz, T_load isareti dahil teorik formulle karsilastirilir.
% Teorik kalici hiz: w = (KT*V/R - Tc - TL) / (B + KT*KB/R)
% Asil modele dokunmaz: dc_motor blogu gecici bir modele kopyalanir.
model = 'dc_motor_position_control';
tm    = 'dc_motor_dogrulama';
load_system(model);
init_motor;
if bdIsLoaded(tm), close_system(tm, 0); end
new_system(tm);
add_block('simulink/Sources/Step',       [tm '/V'],  'Time', '0', 'Before', '0', 'After', '1');
add_block('simulink/Sources/Constant',   [tm '/TL'], 'Value', '0');
add_block([model '/dc_motor'],           [tm '/dc_motor']);
add_block('simulink/Sinks/To Workspace', [tm '/pos_out'], ...
    'VariableName', 'pos', 'SaveFormat', 'Timeseries');
add_line(tm, 'V/1',        'dc_motor/1');
add_line(tm, 'TL/1',       'dc_motor/2');
add_line(tm, 'dc_motor/1', 'pos_out/1');
set_param(tm, 'InitFcn', 'init_motor;', 'StopTime', '10');

m = motor;  Vin = 1;  tol = 0.005;                  % %0.5 bagil tolerans
w_teori = @(Tc, TL) (m.KT*Vin/m.R - Tc - TL) / (m.B + m.KT*m.KB/m.R);

%    ad                          Tc (maske)   Tc (sayi)   TL (N.m)
D = {'A: surtunmesiz',           '0',         0,          0
    'B: Coulomb surtunmesi',    'motor.Tc',  m.Tc,       0
    'C: yuk torku 0.02 N.m',    '0',         0,          0.02};

dogrulama = struct('ad', {}, 'w_teori', {}, 'w_sim', {}, 'hata', {}, 'ok', {});
for i = 1:size(D, 1)
    in = Simulink.SimulationInput(tm);
    in = setBlockParameter(in, [tm '/dc_motor'], 'Tc', D{i,2});
    in = setBlockParameter(in, [tm '/TL'], 'Value', num2str(D{i,4}));
    out = sim(in);
    p = out.pos;  tp = p.Time;  xp = p.Data(:);
    i0 = find(tp >= 8, 1);
    w_sim = (xp(end) - xp(i0)) / (tp(end) - tp(i0));    % son 2 s'deki egim
    w_th  = w_teori(D{i,3}, D{i,4});
    h = abs(w_sim - w_th) / abs(w_th);
    dogrulama(i) = struct('ad', D{i,1}, 'w_teori', w_th, 'w_sim', w_sim, 'hata', h, 'ok', h < tol);
    fprintf('%-24s w_teori=%.4f  w_sim=%.4f rad/s  bagil hata=%.2e  %s\n', ...
        D{i,1}, w_th, w_sim, h, pf_(h < tol));
    if i == 1, pA = p; end
end

% A durumunda konum egrisinin tamami: durum-uzayi modeliyle karsilastirma
Ass = [-m.R/m.L, -m.KB/m.L, 0;  m.KT/m.J, -m.B/m.J, 0;  0, 1, 0];
sys = ss(Ass, [1/m.L; 0; 0], [0 0 1], 0);
ts  = linspace(0, 10, 2001)';
x_ss  = step(sys, ts) * Vin;
x_sim = resample(pA, ts).Data(:);
ss_hata = max(abs(x_sim - x_ss)) / max(abs(x_ss));
fprintf('%-24s konum egrisi, bagil hata=%.2e  %s\n', 'A: durum-uzayi', ss_hata, pf_(ss_hata < tol));
dogrulama_ok = all([dogrulama.ok]) && ss_hata < tol;
fprintf('MODEL DOGRULAMA: %s\n', pf_(dogrulama_ok));

figure;
plot(ts, x_sim, 'b', ts, x_ss, 'r--'); grid on;
legend('Simulink', 'Durum-uzayi', 'Location', 'northwest');
xlabel('s'); ylabel('konum (rad)');
title('Acik cevrim 1 V, surtunmesiz: model dogrulama');
exportgraphics(gcf, fullfile('results', 'model_dogrulama.png'), 'Resolution', 200);
close_system(tm, 0);

function s = pf_(c), if c, s = 'PASS'; else, s = 'FAIL'; end, end
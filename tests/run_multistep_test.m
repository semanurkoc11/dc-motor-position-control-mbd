% run_multistep_test.m - REQ-10: art arda adimlar (0 -> 50 -> 20 -> 80 rad)
% Her adim icin asim, yerlesme (%2 bandi), kalici hata olculur. Her adim ayrica ayni
% buyuklukte, dinlenmeden baslayan tek bir adimla karsilastirilir: fark ~0 ise
% kontrolcu onceki hareketten bir iz (integrator birikimi) tasimiyor demektir.
% Modele dokunmaz: referans profili SimulationInput ile yalnizca o kosu icin verilir.
model = 'dc_motor_position_control';
load_system(model);
init_motor;

t_adim = [0 10 20];  hedef = [50 20 80];  t_son = 35;
onceki = [0 hedef(1:end-1)];
t_bit  = [t_adim(2:end) t_son];

% Profil: ayni zaman iki kez yazilinca o anda basamak olur
prof = [0 hedef(1)];
for k = 2:numel(hedef)
    prof = [prof; t_adim(k) hedef(k-1); t_adim(k) hedef(k)]; %#ok<AGROW>
end
prof = [prof; t_son hedef(end)];
disp('Referans profili [zaman konum]:'); disp(prof)

% --- Art arda adimlar ---
out = kos(model, prof, t_son);
pos = out.logsout.getElement('position_true_rad').Values;
t = pos.Time;  x = pos.Data(:);
mv = out.logsout.getElement('motor_voltage_V').Values;
umax = max(abs(mv.Data));

% --- Her adim: olcum + ayni buyuklukte tek adimla karsilastirma ---
n = numel(hedef);  R = nan(n, 8);
for k = 1:n
    m = olc(t, x, t_adim(k), t_bit(k), hedef(k), onceki(k));
    a = hedef(k) - onceki(k);  sure = t_bit(k) - t_adim(k);
    o1 = kos(model, [0 a; 1 a], sure);                       % 0'dan a'ya, dinlenmeden
    p1 = o1.logsout.getElement('position_true_rad').Values;
    m1 = olc(p1.Time, p1.Data(:), 0, sure, a, 0);
    R(k,:) = [onceki(k), hedef(k), a, m.asim, m.yer, m.kalici, m1.yer, m.yer - m1.yer];
    fprintf('%d/%d bitti: %g -> %g rad\n', k, n, onceki(k), hedef(k));
end

% --- Grafik ---
figure('Position', [100 100 900 600]);
tiledlayout(2, 1, 'TileSpacing', 'compact');
nexttile; hold on; grid on;
rf = out.logsout.getElement('reference_rad').Values;
plot(rf.Time, rf.Data(:), 'k--', 'DisplayName', 'referans');
plot(t, x, 'DisplayName', 'konum');
ylabel('Konum (rad)');  legend('Location', 'southeast');
title('REQ-10: art arda adimlar 0 \rightarrow 50 \rightarrow 20 \rightarrow 80 rad', 'FontWeight', 'normal');
nexttile; grid on;
plot(mv.Time, mv.Data(:));  ylim([-13 13]);
ylabel('Motor voltaji (V)');  xlabel('Zaman (s)');
exportgraphics(gcf, fullfile('results', 'cok_adimli_referans.png'), 'Resolution', 200);

T = array2table(R, 'VariableNames', {'Baslangic_rad', 'Hedef_rad', 'Adim_rad', 'Asim_pct', ...
    'Yerlesme_s', 'KaliciHata_rad', 'Yerlesme_tek_adim_s', 'Fark_s'});
disp(T)
fprintf('Umax = %.4f V\n', umax);

function out = kos(model, prof, t_stop)
in = Simulink.SimulationInput(model);
in = setModelParameter(in, 'StopTime', num2str(t_stop));
in = setBlockParameter(in, [model '/Voltage_Disturbance'], 'After', '0');
in = setBlockParameter(in, [model '/Load_Torque'], 'After', '0');
in = setBlockParameter(in, [model '/reference'], 'VariableName', mat2str(prof));
out = sim(in);
end

function m = olc(t, x, t0, t1, r, r0)
% Bir adimin olcumleri: [t0, t1] penceresinde, r0'dan r'ye
w = t >= t0 & t <= t1;  tw = t(w);  xw = x(w);
adim = r - r0;
m.asim = max(0, max(sign(adim) * (xw - r))) / abs(adim) * 100;     % hedefi gecme, %
idx = find(abs(xw - r) > 0.02 * abs(adim), 1, 'last');            % %2 bandindan son cikis
if isempty(idx),         m.yer = 0;
elseif idx == numel(tw), m.yer = NaN;                              % pencerede yerlesmedi
else,                    m.yer = tw(idx) - t0;
end
m.kalici = abs(mean(r - xw(tw >= t1 - 2)));                        % pencerenin son 2 s'si
end
% run_load_worstcase.m - REQ-09: yuk torku, nominal ve en kotu durum kosullarinda
% Kosullar yalnizca MOTORA ve BESLEMEYE uygulanir; kontrolcu nominal degerlerle kalir.
% (Gercekte kontrolcu motorun isindigini ya da beslemenin dustugunu bilmez.)
% Modele dokunmaz: tum degisiklikler SimulationInput ile yalnizca o kosu icin verilir.
model = 'dc_motor_position_control';
load_system(model);
init_motor;
blk = [model '/dc_motor'];
sat = [model '/Saturation'];
lt  = [model '/Load_Torque'];
ref = 50;  t_d = 10;

%     ad                           R_x    Vbesleme  Tc_x
K = { 'Nominal',                   1.0,   12,       1.0
    'Sicak motor',               1.2,   12,       1.0
    'Sicak + 11 V',              1.2,   11,       1.0
    'Sicak + 11 V + Tc x1.5',    1.2,   11,       1.5 };
yukler = [0.3 0.4 0.5 0.6];                      % N.m

nK = size(K, 1);  nY = numel(yukler);
S = nan(nK*nY, 7);  ad = strings(nK*nY, 1);  n = 0;
figure('Position', [100 100 1100 700]);
tl = tiledlayout(2, 2, 'TileSpacing', 'compact');
for k = 1:nK
    kap = motor.KT*K{k,3} / (K{k,2}*motor.R) - K{k,4}*motor.Tc;   % el hesabi kapasite
    nexttile; hold on; grid on;
    for y = yukler
        n = n + 1;
        in = Simulink.SimulationInput(model);
        in = setModelParameter(in, 'StopTime', '30');
        in = setBlockParameter(in, [model '/Voltage_Disturbance'], 'After', '0');
        in = setBlockParameter(in, blk, 'R',  sprintf('%g*motor.R',  K{k,2}));
        in = setBlockParameter(in, blk, 'Tc', sprintf('%g*motor.Tc', K{k,4}));
        in = setBlockParameter(in, sat, 'UpperLimit', num2str(K{k,3}));
        in = setBlockParameter(in, sat, 'LowerLimit', num2str(-K{k,3}));
        in = setBlockParameter(in, lt, 'Time',   num2str(t_d));
        in = setBlockParameter(in, lt, 'Before', '0');
        in = setBlockParameter(in, lt, 'After',  num2str(y));
        out = sim(in);

        pos = out.logsout.getElement('position_true_rad').Values;
        t = pos.Time;  x = pos.Data(:);  e = ref - x;
        St = stepinfo(x(t < t_d), t(t < t_d), ref);              % yuk oncesi adim cevabi

        ad(n) = K{k,1};
        S(n,:) = [y, kap, max(abs(e(t >= t_d))), abs(mean(e(t >= 25))), ...
            recov(t, e, 0.05, t_d), St.SettlingTime, 0];
        S(n,7) = S(n,3) <= 1 && S(n,4) < 0.01 && S(n,5) < 5;    % REQ-09 a+b+c (NaN -> FAIL)
        plot(t, x, 'DisplayName', sprintf('%.1f N.m', y));
        fprintf('%2d/%d bitti: %-24s %.1f N.m\n', n, nK*nY, K{k,1}, y);
    end
    yline(ref, 'k--', 'HandleVisibility', 'off');  xline(t_d, 'r:', 'HandleVisibility', 'off');
    xlim([8 30]);  ylim([ref-3 ref+0.5]);  legend('Location', 'southeast');
    title(sprintf('%s (kapasite = %.3f N.m)', K{k,1}, kap), 'FontWeight', 'normal');
end
title(tl, 'Yuk torku: nominal ve en kotu durum kosullari');
xlabel(tl, 'Zaman (s)');  ylabel(tl, 'Konum (rad)');
exportgraphics(gcf, fullfile('results', 'yuk_en_kotu_durum.png'), 'Resolution', 200);

T = array2table(S, 'VariableNames', {'Yuk_Nm', 'Kapasite_hesap_Nm', 'MaxSapma_rad', ...
    'KaliciHata_rad', 'Toparlanma_0p05_s', 'Yerlesme_s', 'REQ09'});
T.REQ09 = logical(T.REQ09);
T.Kosul = ad;  T = movevars(T, 'Kosul', 'Before', 1);
disp(T)

function tr = recov(t, e, band, t_d)
idx = find(abs(e) > band & t >= t_d, 1, 'last');
if isempty(idx),        tr = 0;      % bandin disina hic cikmadi
elseif idx == numel(t), tr = NaN;    % banda hic donmedi
else,                   tr = t(idx) - t_d;
end
end
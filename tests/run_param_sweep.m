% run_param_sweep.m - REQ-08: R ve J icin +-%20 taramasi
% Modele dokunmaz: R ve J yalnizca o kosu icin SimulationInput ile verilir.
% Not: Surtunme kompanzasyonu init_motor'daki NOMINAL degerleri kullanmaya devam eder.
% Bu bilincli bir tercih: kontrolcu motorun gercek R ve J degerlerini bilmez.
model = 'dc_motor_position_control';
blk   = [model '/dc_motor'];
load_system(model);
init_motor;                                   % her zaman guncel parametreler

scales = [0.8 1.0 1.2];
ref    = 50;
A = zeros(numel(scales)^2, 6);

figure('Name', 'REQ-08 Parametre Taramasi'); hold on; grid on;
n = 0;
for iR = 1:numel(scales)
    for iJ = 1:numel(scales)
        n = n + 1;
        in = Simulink.SimulationInput(model);
        in = setModelParameter(in, 'StopTime', '20');
        in = setBlockParameter(in, [model '/Voltage_Disturbance'], 'After', '0');   % bozucu yok
        in = setBlockParameter(in, [model '/Load_Torque'], 'After', '0');           % yuk yok
        in = setBlockParameter(in, blk, 'R', sprintf('%g*motor.R', scales(iR)));
        in = setBlockParameter(in, blk, 'J', sprintf('%g*motor.J', scales(iJ)));
        out = sim(in);

        L   = out.logsout;
        pos = L.getElement('position_true_rad').Values;
        mv  = L.getElement('motor_voltage_V').Values;
        S   = stepinfo(pos.Data, pos.Time, ref);

        A(n,:) = [scales(iR), scales(iJ), S.Overshoot, S.SettlingTime, ...
            abs(ref - pos.Data(end)), max(abs(mv.Data))];

        if scales(iR) == 1 && scales(iJ) == 1, lw = 2.4; else, lw = 0.8; end   % nominal kalin
        plot(pos.Time, pos.Data, 'LineWidth', lw);
    end
end
yline(ref, 'k--', 'Reference');
xlabel('Zaman (s)'); ylabel('Konum (rad)');
title('R ve J \pm%20 Taramasi (kalin cizgi = nominal)');
exportgraphics(gcf, fullfile('results', 'param_taramasi.png'), 'Resolution', 200);

T = array2table(A, 'VariableNames', {'R_x','J_x','Asim_pct','Yerlesme_s','ess_rad','Umax_V'});
T.REQ03 = T.Asim_pct   < 5;
T.REQ04 = T.Yerlesme_s < 5;
T.REQ02 = T.ess_rad    < 0.5;
T.REQ05 = T.Umax_V     <= 12 + 1e-6;
disp(T)
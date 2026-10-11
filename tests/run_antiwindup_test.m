% run_antiwindup_test.m - REQ-01..05: anti-windup yontemlerinin karsilastirmasi, 50 rad adim
% Modele dokunmaz: tum ayarlar SimulationInput ile yalnizca o kosu icin verilir.
% Cizimler acik figur/eksen tutamaklariyla yapilir: test surerken baska pencereye
% tiklamak grafigi bozmaz.
model   = 'dc_motor_position_control';
blk_pid = [model '/PID Controller'];
load_system(model);

ref   = 50;
modes = {'none', 'clamping', 'back-calculation'};
Asim = zeros(3,1); Yerlesme = zeros(3,1); ess = zeros(3,1); umax = nan(3,1);

f = figure;  ax = axes(f);  hold(ax, 'on');  grid(ax, 'on');
for i = 1:numel(modes)
    in = Simulink.SimulationInput(model);
    in = setModelParameter(in, 'StopTime', '20');
    in = setBlockParameter(in, [model '/Voltage_Disturbance'], 'After', '0');   % bozucu yok
    in = setBlockParameter(in, [model '/Load_Torque'], 'After', '0');           % yuk yok
    in = setBlockParameter(in, blk_pid, 'LimitOutput', 'on');
    in = setBlockParameter(in, blk_pid, 'UpperSaturationLimit', 'ctrl.Vmax');
    in = setBlockParameter(in, blk_pid, 'LowerSaturationLimit', '-ctrl.Vmax');
    in = setBlockParameter(in, blk_pid, 'AntiWindupMode', modes{i});
    out = sim(in);

    pos = out.logsout.getElement('position_true_rad').Values;
    S = stepinfo(pos.Data, pos.Time, ref);
    Asim(i) = S.Overshoot;  Yerlesme(i) = S.SettlingTime;
    ess(i) = abs(ref - pos.Data(end));

    % Motor voltaji: Saturation blogunun cikisi, yoluna gore bulunur
    for kk = 1:out.logsout.numElements
        e = out.logsout{kk};
        if strcmp(e.BlockPath.getBlock(e.BlockPath.getLength()), [model '/Saturation'])
            umax(i) = max(abs(e.Values.Data));
        end
    end

    plot(ax, pos.Time, pos.Data, 'DisplayName', modes{i});
end
yline(ax, ref, 'k--', 'DisplayName', 'referans');
legend(ax); xlabel(ax, 's'); ylabel(ax, 'konum (rad)');
title(ax, 'Anti-windup yöntemlerinin karşılaştırması, 50 rad adım');
exportgraphics(f, fullfile('results', 'antiwindup_karsilastirma.png'), 'Resolution', 200);

T = table(modes', Asim, Yerlesme, ess, umax, ...
    Asim < 5, Yerlesme < 5, ess < 0.5, umax <= 12 + 1e-6, ...
    'VariableNames', {'Yontem','Asim_pct','Yerlesme_s','ess_rad','Umax_V', ...
    'REQ03','REQ04','REQ02','REQ05'});
disp(T)
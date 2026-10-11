model   = 'close_pid_dc_motor_posit_velocity';
blk_pid = [model '/PID Controller'];
load_system(model);
set_param(model, 'StopTime', '20');          % kuyruğun tamamını görmek için
set_param(blk_pid, 'LimitOutput', 'on');
set_param(blk_pid, 'UpperSaturationLimit', '12');
set_param(blk_pid, 'LowerSaturationLimit', '-12');

modes = {'none', 'clamping', 'back-calculation'};
figure; hold on; grid on;
for i = 1:numel(modes)
    set_param(blk_pid, 'AntiWindupMode', modes{i});
    out = sim(model);
    pos = out.logsout.getElement('position_true_rad').Values;
    S   = stepinfo(pos.Data, pos.Time, 50);
    ess = abs(50 - pos.Data(end));
    fprintf('%-18s Asim=%6.2f %%  Yerlesme=%5.2f s  ess=%.4f rad\n', ...
        modes{i}, S.Overshoot, S.SettlingTime, ess);
    plot(pos.Time, pos.Data, 'DisplayName', modes{i});
end
yline(50, 'k--', 'DisplayName', 'referans');
legend; xlabel('s'); ylabel('konum (rad)');
title('Anti-windup yöntemlerinin karşılaştırması, 50 rad adım');
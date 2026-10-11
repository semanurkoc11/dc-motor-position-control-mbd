% add_discrete_controller.m - kontrolcuyu ayrik zamana tasir (ornekleme suresi ctrl.Ts)
% 1) Hata sinyaline Zero-Order Hold (Controller_Sample): PID ve kompanzasyon ornekli hatayi gorur
% 2) PID Controller: Discrete-time, SampleTime = ctrl.Ts
% Bir kez calistirilir: run(fullfile('tools','add_discrete_controller.m'))
model = 'dc_motor_position_control';
load_system(model);
assert(isempty(find_system(model, 'SearchDepth', 1, 'Name', 'Controller_Sample')), ...
    'Controller_Sample zaten eklenmis');

% --- 1) Hata hattini (Sum cikisi) bul, hedeflerini kaydet, kes ---
sp  = get_param([model '/Sum'], 'PortHandles');
ln  = get_param(sp.Outport(1), 'Line');
dst = get_param(ln, 'DstPortHandle');
assert(numel(dst) == 2, 'Hata sinyali iki yere (PID ve Friction_Comp) gitmeliydi');
hedef = cell(numel(dst), 1);
for k = 1:numel(dst)
    hedef{k} = sprintf('%s/%d', get_param(get_param(dst(k), 'Parent'), 'Name'), ...
        get_param(dst(k), 'PortNumber'));
end
disp('Hata sinyalinin gittigi yerler:');  disp(hedef)
delete_line(ln);

% --- 2) Zero-Order Hold ekle ve bagla ---
pp  = get_param([model '/Sum'], 'Position');
zoh = [model '/Controller_Sample'];
add_block('simulink/Discrete/Zero-Order Hold', zoh, 'SampleTime', 'ctrl.Ts', ...
    'Position', [pp(3)+15 pp(2) pp(3)+45 pp(4)]);
add_line(model, 'Sum/1', 'Controller_Sample/1', 'autorouting', 'on');
for k = 1:numel(hedef)
    add_line(model, 'Controller_Sample/1', hedef{k}, 'autorouting', 'on');
end
set_param(get_param(sp.Outport(1), 'Line'), 'Name', 'error');   % hat adini geri ver
zp = get_param(zoh, 'PortHandles');
set_param(zp.Outport(1), 'DataLogging', 'on', 'DataLoggingNameMode', 'Custom', ...
    'DataLoggingName', 'error_sampled_rad');

% --- 3) PID: ayrik zaman ---
pid = [model '/PID Controller'];
set_param(pid, 'TimeDomain', 'Discrete-time', 'SampleTime', 'ctrl.Ts', ...
    'IntegratorMethod', 'Forward Euler', 'FilterMethod', 'Backward Euler');
save_system(model);
fprintf('Bitti: PID %s, Ts = %s\n', get_param(pid, 'TimeDomain'), get_param(pid, 'SampleTime'));
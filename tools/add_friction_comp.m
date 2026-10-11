% add_friction_comp.m - PID cikisina hata tabanli surtunme kompanzasyonu ekler
model = 'dc_motor_position_control';
load_system(model);
assert(isempty(find_system(model, 'SearchDepth', 1, 'Name', 'Friction_Comp')), ...
    'Friction_Comp zaten eklenmis');

% --- 1) PID cikis hattini bul ve kes ---
pid = [model '/PID Controller'];
php = get_param(pid, 'PortHandles');
ln  = get_param(php.Outport(1), 'Line');
dst = get_param(ln, 'DstPortHandle');
assert(numel(dst) == 1, 'PID cikisi tek bir yere gitmeliydi');
dstBlk  = get_param(dst, 'Parent');
dstPort = get_param(dst, 'PortNumber');
fprintf('PID cikisi su an: %s, giris %d\n', dstBlk, dstPort);
delete_line(ln);

% --- 2) Toplama blogu: PID + kompanzasyon ---
pp = get_param(pid, 'Position');
add_block('simulink/Math Operations/Add', [model '/Comp_Sum'], 'Inputs', '++', ...
    'Position', [pp(3)+20 pp(2) pp(3)+50 pp(2)+30]);

% --- 3) Kompanzasyon alt sistemi: Vc * tanh(hata / e0) ---
fc = [model '/Friction_Comp'];
add_block('built-in/Subsystem', fc, 'Position', [pp(1) pp(4)+60 pp(3) pp(4)+110]);
add_block('built-in/Inport',  [fc '/error'],  'Position', [20 40 50 54]);
add_block('built-in/Gain',    [fc '/inv_e0'], 'Gain', '1/ctrl.e0', 'Position', [90 30 140 64]);
add_block('simulink/Math Operations/Trigonometric Function', [fc '/tanh'], ...
    'Operator', 'tanh', 'Position', [180 30 230 64]);
add_block('built-in/Gain',    [fc '/Vc'],     'Gain', 'ctrl.Vc', 'Position', [270 30 320 64]);
add_block('built-in/Outport', [fc '/ff'],     'Position', [360 40 390 54]);
add_line(fc, 'error/1',  'inv_e0/1');
add_line(fc, 'inv_e0/1', 'tanh/1');
add_line(fc, 'tanh/1',   'Vc/1');
add_line(fc, 'Vc/1',     'ff/1');

% --- 4) Ust seviye baglantilar ---
add_line(model, 'PID Controller/1', 'Comp_Sum/1', 'autorouting', 'on');
add_line(model, 'Comp_Sum/1', [get_param(dstBlk, 'Name') '/' num2str(dstPort)], 'autorouting', 'on');
add_line(model, 'Sum/1', 'Friction_Comp/1', 'autorouting', 'on');   % hata sinyali
add_line(model, 'Friction_Comp/1', 'Comp_Sum/2', 'autorouting', 'on');

% --- 5) Loglar ---
set_param(php.Outport(1), 'DataLoggingName', 'pid_fb_V');   % PID'nin kendi cikisi
pc = get_param([model '/Comp_Sum'], 'PortHandles');
set_param(pc.Outport(1), 'DataLogging', 'on', ...
    'DataLoggingNameMode', 'Custom', 'DataLoggingName', 'pid_out_V');   % toplam kontrol
pf = get_param(fc, 'PortHandles');
set_param(pf.Outport(1), 'DataLogging', 'on', ...
    'DataLoggingNameMode', 'Custom', 'DataLoggingName', 'friction_ff_V');
save_system(model);
disp('Bitti.')
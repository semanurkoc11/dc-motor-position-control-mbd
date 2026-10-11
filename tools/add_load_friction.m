% add_load_friction.m - dc_motor icine Coulomb surtunmesi ve yuk torku girisi ekler
% TEK SEFERLIK KURULUM SCRIPT'I: model bu bloklari zaten iceriyor, tekrar calistirmaya
% gerek yok. Tekrar calistirilirsa asagidaki koruma onu modele dokunmadan durdurur.
model = 'dc_motor_position_control';
load_system(model);
blk = [model '/dc_motor'];
assert(isempty(find_system(blk, 'LookUnderMasks', 'all', 'SearchDepth', 1, 'Name', 'Coulomb')), ...
    'Coulomb blogu zaten eklenmis');

% --- 1) Voltaj bozucusuna sabit bir ad ver ---
st = find_system(model, 'SearchDepth', 1, 'BlockType', 'Step');
assert(numel(st) == 1, 'Ust seviyede tek Step blogu bekleniyordu');
set_param(st{1}, 'Name', 'Voltage_Disturbance');

% --- 2) Maskeye yeni parametreler ---
mk = Simulink.Mask.get(blk);
mk.addParameter('Type', 'edit', 'Name', 'Tc', ...
    'Prompt', 'Coulomb surtunme torku (N.m)', 'Value', 'motor.Tc');
mk.addParameter('Type', 'edit', 'Name', 'w_eps', ...
    'Prompt', 'Coulomb yumusatma hizi (rad/s)', 'Value', 'motor.w_eps');

% --- 3) Mekanik toplamaya iki eksi giris (Coulomb, yuk torku) ---
sm = [blk '/Sum1'];
set_param(sm, 'Inputs', [get_param(sm, 'Inputs') '--']);
p = get_param(sm, 'Position');

% --- 4) Coulomb surtunmesi: Tc*tanh(w/w_eps) ---
add_block('simulink/User-Defined Functions/Fcn', [blk '/Coulomb'], ...
    'Expr', 'Tc*tanh(u(1)/w_eps)', 'Orientation', 'left', ...
    'Position', [p(1)+80 p(4)+60 p(1)+220 p(4)+90]);
add_line(blk, 'Integrator1/1', 'Coulomb/1', 'autorouting', 'on');
add_line(blk, 'Coulomb/1', 'Sum1/3', 'autorouting', 'on');

% --- 5) Yuk torku girisi ---
add_block('simulink/Sources/In1', [blk '/T_load'], ...
    'Position', [p(1)-80 p(4)+120 p(1)-50 p(4)+134]);
add_line(blk, 'T_load/1', 'Sum1/4', 'autorouting', 'on');

% --- 6) Ust seviyede yuk torku kaynagi (0) ---
pm = get_param(blk, 'Position');
add_block('simulink/Sources/Step', [model '/Load_Torque'], ...
    'Time', '10', 'Before', '0', 'After', '0', ...
    'Position', [pm(1)-120 pm(4)+40 pm(1)-90 pm(4)+70]);
add_line(model, 'Load_Torque/1', 'dc_motor/2', 'autorouting', 'on');

% --- 7) Yeni sinyalleri logla ---
ph = get_param([model '/Load_Torque'], 'PortHandles');
set_param(ph.Outport(1), 'DataLogging', 'on', ...
    'DataLoggingNameMode', 'Custom', 'DataLoggingName', 'load_torque_Nm');
ph = get_param([blk '/Coulomb'], 'PortHandles');
set_param(ph.Outport(1), 'DataLogging', 'on', ...
    'DataLoggingNameMode', 'Custom', 'DataLoggingName', 'coulomb_Nm');
save_system(model);
disp('Bitti.')
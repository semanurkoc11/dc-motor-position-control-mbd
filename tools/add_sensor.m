% add_sensor.m - geri besleme hattina gurultu + enkoder nicemleme ekler
model = 'dc_motor_position_control';
load_system(model);
assert(isempty(find_system(model,'SearchDepth',1,'Name','Encoder_Quantizer')), ...
    'Sensor zaten eklenmis');

sm = [model '/Sum'];                       % hata toplami (ref - olcum)
ph = get_param(sm, 'PortHandles');
ln = get_param(ph.Inport(2), 'Line');
assert(ln ~= -1, 'Sum girisi 2 bagli degil');
delete_line(ln);                           % eski dogrudan geri besleme

p = get_param(sm, 'Position');  y = p(4) + 120;
add_block('simulink/Sources/Band-Limited White Noise', [model '/Sensor_Noise'], ...
    'Cov', 'sensor.noise_cov', 'Ts', 'sensor.noise_Ts', 'seed', '[sensor.seed]', ...
    'Position', [p(1)-330 y+60 p(1)-270 y+90]);
add_block('simulink/Math Operations/Add', [model '/Sensor_Sum'], ...
    'Inputs', '++', 'Position', [p(1)-230 y p(1)-200 y+40]);
add_block('simulink/Discontinuities/Quantizer', [model '/Encoder_Quantizer'], ...
    'QuantizationInterval', 'sensor.q_enc', 'Position', [p(1)-160 y p(1)-100 y+40]);

add_line(model, 'dc_motor/1',          'Sensor_Sum/1',        'autorouting', 'on');
add_line(model, 'Sensor_Noise/1',      'Sensor_Sum/2',        'autorouting', 'on');
add_line(model, 'Sensor_Sum/1',        'Encoder_Quantizer/1', 'autorouting', 'on');
add_line(model, 'Encoder_Quantizer/1', 'Sum/2',               'autorouting', 'on');

ph2 = get_param([model '/Encoder_Quantizer'], 'PortHandles');
set_param(ph2.Outport(1), 'DataLogging', 'on', ...
    'DataLoggingNameMode', 'Custom', 'DataLoggingName', 'position_meas_rad');
save_system(model);
open_system(model)
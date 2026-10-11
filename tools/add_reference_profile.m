% add_reference_profile.m - sabit referansi (Constant) zaman profiline (From Workspace) cevirir
% Bir kez calistirilir. Varsayilan profil (init_motor: ref_prof) v1.0 ile ayni: t=0'dan sabit 50 rad.
% Testler profili SimulationInput ile yalnizca o kosu icin degistirir; model dosyasi degismez.
model = 'dc_motor_position_control';
load_system(model);
blk = [model '/reference'];
assert(strcmp(get_param(blk, 'BlockType'), 'Constant'), 'reference zaten degistirilmis');
load_system('simulink');   % replace_block kutuphaneyi kendisi acmaz
replace_block(model, 'Name', 'reference', 'simulink/Sources/From Workspace', 'noprompt');
assert(strcmp(get_param(blk, 'BlockType'), 'FromWorkspace'), 'Degistirme basarisiz');

set_param(blk, 'VariableName', 'ref_prof', ...
    'Interpolate', 'on', ...                              % tekrar eden zaman = keskin basamak
    'ZeroCross', 'on', ...                                % cozucu basamak anina tam denk gelir
    'OutputAfterFinalValue', 'Holding final value', ...   % tablo bitince son degerde kal
    'SampleTime', '0');
set_param(blk, 'Description', ...
    'Konum referansi (rad). Veri: ref_prof = [zaman konum], init_motor.m icinde. Varsayilan: sabit 50 rad.');

ph = get_param(blk, 'PortHandles');                       % yeni blogun cikisini logla
set_param(ph.Outport(1), 'DataLogging', 'on', 'DataLoggingNameMode', 'Custom', ...
    'DataLoggingName', 'reference_rad');
save_system(model);
disp('Bitti: reference -> From Workspace (ref_prof)')
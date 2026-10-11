% update_labels_v11.m - v1.1 sonrasi eskimis model aciklamalarini ve Scope etiketlerini gunceller
% Davranisi degistirmez: yalnizca aciklama metinleri ve Scope'taki gosterim adlari.
% Bir kez calistirilir: run(fullfile('tools','update_labels_v11.m'))
model = 'dc_motor_position_control';
load_system(model);

% --- 1) Load_Torque blok aciklamasi ("v1.1 ile eklenecek" artik dogru degil) ---
lt = [model '/Load_Torque'];
set_param(lt, 'Description', ['Yuk torku girisi (N.m). Varsayilan 0. ' ...
    'REQ-09 testleri (run_load_torque_test, run_load_worstcase) basamak yuku ' ...
    'SimulationInput ile yalnizca o kosu icin verir.']);

% --- 2) Diyagramdaki Load_Torque notu ---
an = find_system(model, 'FindAll', 'on', 'SearchDepth', 1, 'Type', 'annotation');
bulundu = 0;
for k = 1:numel(an)
    if contains(get_param(an(k), 'Text'), 'v1.0 icinde')
        set_param(an(k), 'Text', 'Load_Torque: varsayilan 0 N.m (yuk testleri REQ-09, SimulationInput ile)');
        bulundu = bulundu + 1;
    end
end
fprintf('Guncellenen not sayisi: %d (beklenen 1)\n', bulundu);

% --- 3) Scope etiketleri: giris 1 = gercek konum, giris 2 = referans ---
sc = [model '/Scope'];
try
    cfg = get_param(sc, 'ScopeConfiguration');
    cfg.ChannelNames = {'position_true_rad', 'reference_rad'};
    disp('Scope etiketleri guncellendi.');
catch me
    fprintf('Scope etiketleri otomatik guncellenemedi: %s\n', me.message);
end

save_system(model);
disp('Bitti.')

% run_disturbance_sweep.m
% REQ-06: Bozucu genliği taraması

model = 'dc_motor_position_control';

% Model yüklü değilse yükle
if ~bdIsLoaded(model)
    load_system(model);
end

% Üst seviyedeki Step (bozucu) bloğunu bul
st = {[model '/Voltage_Disturbance']};

if numel(st) ~= 1
    fprintf('Üst seviyede bulunan Step blokları (%d adet):\n', numel(st));
    disp(st);
    error(['Üst seviyede tam 1 Step bloğu bekleniyordu. ' ...
           'Bozucu bloğun yolunu elle atayın: ' ...
           'stp = ''dc_motor_position_control/Step'';']);
end

stp = st{1};

% Tarama parametreleri
amps  = [3 6 9 11 13];       % Bozucu genlikleri, V
bands = [0.05 0.01];         % Toparlanma bantları, rad
ref   = 50;                  % Referans konum, rad
t_d   = 10;                  % Bozucunun uygulandığı zaman, s
t_end = 30;                  % Simülasyon süresi, s

nAmp  = numel(amps);
nBand = numel(bands);

% Sonuç matrisi:
% 1: Bozucu genliği
% 2: Maksimum sapma
% 3: Final hata
% 4:(3+nBand): Toparlanma süreleri
% Son sütun: Dış doyma farkı
R = nan(nAmp, 3 + nBand + 1);

figure;
hold on;
grid on;

for i = 1:nAmp
    % Her tarama için yeni SimulationInput nesnesi
    in = Simulink.SimulationInput(model);

    % Model parametresi: simülasyon süresi
    in = setModelParameter(in, 'StopTime', num2str(t_end));

    % Step bloğu parametreleri ayrı ayrı ayarlanıyor
    in = setBlockParameter(in, stp, 'Time', num2str(t_d));
    in = setBlockParameter(in, stp, 'Before', '0');
    in = setBlockParameter(in, stp, 'After', num2str(amps(i)));

    % Simülasyonu çalıştır
    out = sim(in);

    % Log verisini doğrula
    if isempty(out.logsout)
        error(['Simülasyon log verisi boş. Modelde sinyal loglama ' ...
               'etkin olmalı ve log adı "logsout" olmalıdır.']);
    end

    L = out.logsout;

    % Sinyalleri al
    pos = getVals(L, 'position_true_rad');
    pd  = getVals(L, 'pid_minus_dist_V');
    mv  = getVals(L, 'motor_voltage_V');

    % Motor gerilimini PID-doyma öncesi sinyalin zamanına göre eşle
    mv_interp = interp1(mv.Time, mv.Data, pd.Time, 'linear', 'extrap');

    % Konum ve hata sinyali
    t = pos.Time;
    e = ref - pos.Data;

    % Bozucu uygulandıktan sonraki bölge
    k = t >= t_d;

    % 1) Bozucu genliği
    R(i,1) = amps(i);

    % 2) Bozucudan sonraki en büyük mutlak sapma
    R(i,2) = max(abs(e(k)));

    % 3) Simülasyon sonu hata
    R(i,3) = abs(e(end));

    % 4) Toparlanma süreleri
    for b = 1:nBand
        R(i, 3 + b) = recov(t, e, bands(b), t_d);
    end

    % 5) Dış doyma farkı
    % PID çıkışı ile gerçek motor gerilimi arasındaki maksimum fark
    kk = pd.Time >= t_d;
    R(i, end) = max(abs(pd.Data(kk) - mv_interp(kk)));

    % Konum grafikleri
    plot(t, pos.Data, ...
        'DisplayName', sprintf('%g V', amps(i)));
end

% Referans ve bozucu zamanı çizgileri
yline(ref, 'k--', 'HandleVisibility', 'off');
xline(t_d, 'r:', 'HandleVisibility', 'off');

xlim([t_d - 2, t_end]);
legend('Location', 'southwest');
xlabel('Zaman (s)');
ylabel('Konum (rad)');
title('Bozucu genliği taraması');

% Grafiği kaydet
exportgraphics(gcf, fullfile('results', 'bozucu_taramasi.png'), 'Resolution', 200);

% Sonuç tablosu
varNames = [{'Bozucu_V', 'MaxSapma_rad', 'FinalHata_rad'}, ...
            compose('Toparlanma_%g_rad_s', bands), ...
            {'DisDoymaFarki_V'}];

T = array2table(R, 'VariableNames', varNames);
disp(T);

% ---- Yerel fonksiyonlar: scriptin en sonunda olmalı ----

function tr = recov(t, e, band, t_d)
    % Bozucu uygulandıktan sonra hata son kez hangi anda bandın
    % dışına çıkıyorsa, toparlanma süresi o andır.

    % Bozucu sonrası zaman indeksleri
    k = t >= t_d;

    % Bandın dışında kalınan son örnek
    idxOutside = find(abs(e(k)) > band, 1, 'last');

    % Hata hiç bandın dışına çıkmadıysa
    if isempty(idxOutside)
        tr = 0;
        return;
    end

    % k içindeki indeksi, t vektöründeki gerçek indekse çevir
    idxGlobal = find(k, 1, 'first') + idxOutside - 1;

    % Simülasyon sonuna kadar bandın dışında kaldıysa
    if idxGlobal == numel(t)
        tr = NaN;
    else
        tr = t(idxGlobal) - t_d;
    end
end

function v = getVals(L, name)
    % Log içindeki sinyali bulur; bulunamazsa anlaşılır hata verir.
    e = L.getElement(name);

    if isempty(e)
        error('Sinyal bulunamadı: %s', name);
    end

    v = e.Values;
end
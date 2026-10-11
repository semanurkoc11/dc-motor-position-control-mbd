% check_logs.m - loglanan sinyallerin fiziksel ve mantiksal tutarlilik kontrolu
model = 'dc_motor_position_control';
load_system(model); init_motor;

out = sim(model, 'StopTime', '20');
L = out.logsout;

t0 = getTime(L, 'Saturation');                    % ortak zaman ekseni
g  = @(blk) getSig(L, [model '/' blk], t0);

% --- Motor ici ---
sat = g('Saturation');           vin = g('dc_motor/voltage');
i   = g('dc_motor/Integrator');  w   = g('dc_motor/Integrator1');
th  = g('dc_motor/Integrator2'); pos = g('dc_motor');
Ri  = g('dc_motor/Gain');        emf = g('dc_motor/Gain2');
tq  = g('dc_motor/Gain4');       fr  = g('dc_motor/Gain5');
acc = g('dc_motor/Gain6');       dI  = g('dc_motor/Gain1');
sE  = g('dc_motor/Sum');         sM  = g('dc_motor/Sum1');
cl  = g('dc_motor/Coulomb');     tl  = g('Load_Torque');

% --- Kontrolcu tarafi ---
rf  = g('reference');            pm  = g('Encoder_Quantizer');
er  = g('Sum');                  pfb = g('PID Controller');
ff  = g('Friction_Comp');        po  = g('Comp_Sum');
vd  = g('Voltage_Disturbance');  pd  = g('Sum1');

chk = {
    'Saturation cikisi = motor girisi    ', sat, vin
    'R*i                                 ', Ri,  motor.R*i
    'KB*w  (BACK_EMF)                    ', emf, motor.KB*w
    'KT*i  (TORQUE)                      ', tq,  motor.KT*i
    'B*w   (FRICTION)                    ', fr,  motor.B*w
    'Coulomb = Tc*tanh(w/w_eps)          ', cl,  motor.Tc*tanh(w/motor.w_eps)
    'Elektrik toplami V-Ri-emf           ', sE,  vin - Ri - emf
    'di/dt = toplam / L                  ', dI,  sE/motor.L
    'Mekanik toplami T-F-Coulomb-Yuk     ', sM,  tq - fr - cl - tl
    'acc = toplam / J                    ', acc, sM/motor.J
    'Cikis konumu = Integrator2          ', pos, th
    'Hata = referans - olculen konum     ', er,  rf - pm
    'Kompanzasyon = Vc*tanh(e/e0)        ', ff,  ctrl.Vc*tanh(er/ctrl.e0)
    'Kontrol cikisi = PID + kompanzasyon ', po,  pfb + ff
    'Bozucu toplami = kontrol - bozucu   ', pd,  po - vd
    'Saturation = sinirlanmis toplam     ', sat, min(max(pd, -ctrl.Vmax), ctrl.Vmax) };

for k = 1:size(chk,1)
    err = max(abs(chk{k,2} - chk{k,3}));
    fprintf('%s maxhata=%.2e  %s\n', chk{k,1}, err, ternary(err < 1e-6));
end

% Enkoder makullugu: |olculen - gercek| <= q/2 + 6*sigma
enc_sinir = sensor.q_enc/2 + 6*sensor.noise_std;
enc_fark  = max(abs(pm - pos));
enc_ok    = enc_fark <= enc_sinir;
fprintf('Enkoder |olculen-gercek| = %.2e <= %.2e  %s\n', enc_fark, enc_sinir, ternary(enc_ok));
fprintf('Umax (motor voltaji) = %.4f V  %s\n', max(abs(sat)), ternary(max(abs(sat)) <= 12+1e-6));

% Sifir kalan (bagli olmayan) loglari bul
beklenen_sifir = {'voltage_disturbance_V', 'load_torque_Nm'};
for k = 1:L.numElements
    e = L{k};
    if max(abs(e.Values.Data)) == 0 && ~ismember(e.Name, beklenen_sifir)
        fprintf('UYARI: sifir sinyal -> %s | %s\n', e.Name, e.BlockPath.getBlock(1));
    end
end

function t = getTime(L, blk)
for k = 1:L.numElements
    e = L{k}; bp = e.BlockPath;
    if endsWith(bp.getBlock(bp.getLength()), blk), t = e.Values.Time; return; end
end
error('Sinyal bulunamadi: %s', blk);
end
function y = getSig(L, path, t)
for k = 1:L.numElements
    e = L{k}; bp = e.BlockPath;
    if strcmp(bp.getBlock(bp.getLength()), path)
        v = e.Values;
        if numel(v.Time) < 2                          % sabit sinyal (Constant blogu)
            y = repmat(squeeze(v.Data(1)), numel(t), 1);
        else
            y = squeeze(resample(v, t).Data);
        end
        return;
    end
end
error('Sinyal bulunamadi: %s', path);
end
function s = ternary(c), if c, s = 'PASS'; else, s = 'FAIL'; end, end
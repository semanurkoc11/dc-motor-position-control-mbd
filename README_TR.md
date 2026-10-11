# DC Motor Konum Kontrolü: Model Tabanlı Tasarım ve MIL Doğrulaması

*Türkçe özet. Tüm ayrıntılar, denklemler ve grafikler için [README.md](README.md) dosyasına bakın.*

Bir DC motorun konum kontrolü, MATLAB/Simulink'te model tabanlı tasarım (MBD) yaklaşımıyla geliştirildi:
ölçülebilir gereksinimler, teoriye karşı doğrulanmış fiziksel bir motor modeli, ölçümle gerekçelendirilmiş
kontrolcü kararları ve otomatik bir model-in-the-loop (MIL) test takımı.

> **Kapsam.** Proje, V-modelinin sol kolunu ve MIL doğrulamasını kapsar: gereksinimler, motor ve sensör
> modeli, kontrolcü tasarımı ve otomatik test. Kod üretimi ve SIL/HIL doğrulaması kapsam dışıdır ve
> yol haritasında yer alır.

**Durum (v1.0):** 17/17 kontrol PASS — 5 model doğrulama, 11 gereksinim, 1 test altyapısı kontrolü.

---

## Nasıl çalıştırılır

```matlab
openProject('DC_Motor_Position_Control_MBD.prj')   % yolu ayarlar, proje kokune gecer
run_all_tests                                       % ~35 simulasyon, 5-10 dakika
```

Sonuçlar `results/` klasörüne yazılır: `test_ozeti.csv` (gereksinim özeti), `test_raporu.txt`
(tam rapor) ve tüm grafikler. Tüm parametreler tek dosyada: `models/init_motor.m`.

**Test edilen sürüm:** MATLAB/Simulink R2026a, Control System Toolbox.

---

## Sistem

- **Motor modeli:** Kütüphane bloğu yerine fiziksel denklemlerden kuruldu: elektriksel devre, mekanik
  dinamik, viskoz ve Coulomb sürtünmesi, yük torku girişi.
- **Sensör:** 4096 sayım/tur enkoder ve ölçüm gürültüsü.
- **Kontrolcü:** PID (back-calculation anti-windup, ±12 V çıkış sınırı) ve buna paralel, hata tabanlı
  Coulomb sürtünme kompanzasyonu.
- **Model doğrulaması:** Açık çevrimde kalıcı hız, kapalı form teorik çözümle üç durumda yaklaşık
  2·10⁻⁶ bağıl hatayla örtüşüyor. Sürtünmesiz konum eğrisi, durum-uzayı modeliyle %0.03 içinde.

---

## Gereksinimler ve sonuçlar

| ID | Gereksinim | Ölçülen | Sınır | Sonuç |
|---|---|---|---|---|
| REQ-01 | 50 rad hedefe ulaşma | 4.03 s | sonlu | PASS |
| REQ-02 | Kalıcı hata | 4.0·10⁻⁵ rad | < 0.5 rad | PASS |
| REQ-03 | Aşım | %0.28 | < %5 | PASS |
| REQ-04 | Yerleşme süresi (%2 bandı) | 4.03 s | < 5 s | PASS |
| REQ-05 | Motor voltajı | 12.0 V | ≤ 12 V | PASS |
| REQ-06a | 9 V bozucuda en büyük sapma | 0.51 rad | ≤ 1 rad | PASS |
| REQ-06b | 3 V bozucu sonrası kalıcı hata | 1.7·10⁻⁵ rad | < 0.01 rad | PASS |
| REQ-06c | 9 V bozucuda ±0.05 rad bandına dönüş | 4.11 s | < 5 s | PASS |
| REQ-07a | Durağan voltaj titreşimi, enkoder + gürültü (AC rms) | 0.32 V | < 0.5 V | PASS |
| REQ-07b | Durağan voltaj titreşimi, gürültüsüz enkoder (AC rms) | 0.001 V | < 0.5 V | PASS |
| REQ-08 | R ve J ±%20 değişimde REQ-02…05 (9 durum), en kötü yerleşme | 4.37 s | hepsi geçmeli | PASS |

En dar pay yerleşme süresinde. 50 rad'lık hareket, motorun 12 V'taki en yüksek hızıyla sınırlı.

---

## Tasarım kararları

Her kontrolcü seçimi, alternatifler gereksinimlere karşı ölçülerek yapıldı.

1. **Anti-windup: back-calculation.** Anti-windup olmadan aşım %80, yerleşme 13.3 s. Back-calculation
   ile aşım %0.28, yerleşme 4.03 s.
2. **Türev filtresi N = 50 (PID Tuner'ın önerdiği 1010 yerine).** N = 1010 ile enkoder ve gürültü,
   motor dururken voltajı ±12 V arasında çırpındırıyordu (5.4 V rms). Bu, konum grafiğinde görünmüyordu.
   N = 50 ile titreşim 0.32 V'a indi, yerleşme yalnızca 0.03 s uzadı.
3. **Coulomb sürtünme kompanzasyonu.** Sürtünme eklenince konum hedefe çok yavaş sürünüyor ve REQ-06b
   geçmiyordu (0.017 rad). Kontrolcüye `Vc·tanh(e/e0)` biçiminde, Vc = 0.8 V olan bir ön besleme
   eklendi. Kalıcı hata yaklaşık 10 enkoder adımından adımın onda birinin altına indi. Kompanzasyon,
   gerçek bir kontrolcü gibi motorun **nominal** değerlerini kullanıyor.
4. **Kompanzasyon genişliği e0 = 0.01 rad.** e0 = 0.005 ile konum iki enkoder adımı arasında gidip
   geliyordu (limit çevrimi). e0 = 0.02 ile kompanzasyon hedef yakınında fazla zayıf kalıyordu.
   e0 = 0.01 hem en düşük hatayı hem en düşük titreşimi verdi.

---

## Doğrulama yaklaşımı

- Motor modeli kontrolcüden bağımsız olarak, teoriye ve durum-uzayı modeline karşı doğrulandı.
- `check_logs`, referanstan motor konumuna kadar 16 denklemi loglanan sinyallerden yeniden hesaplıyor.
- Yapısal değişiklikler (sürtünme, yük girişi, kompanzasyon) önce sıfır değerle eklendi, sonuçların
  değişmediği kanıtlandıktan sonra açıldı.
- `run_all_tests` her değişiklikten sonra tüm testleri yeniden çalıştırıyor.
- Hiçbir test modeli değiştirmiyor. Bu da her çalıştırmada otomatik olarak kontrol ediliyor.
- Tüm parametreler tek bir dosyadan (`init_motor.m`) geliyor.

---

## Bilinen sınırlar

- Kompanzasyon PID'nin iç sınırından sonra eklendiği için toplam komut 12.8 V'a çıkabiliyor. Dış sınır
  bunu 12 V'a kesiyor, ama PID'nin anti-windup'ı bu kırpmayı görmüyor.
- `tanh` sürtünme modeli gerçek durma sürtünmesini (stiction) temsil etmiyor.
- Tc = 0.05 N·m ve 0.001 rad gürültü varsayımdır, veri sayfasından alınmadı.
- 12 V ve üstü voltaj bozucuları genel olarak dengelenemez.
- Yük torku girişi açık çevrimde doğrulandı. Kapalı çevrim yük torku gereksinimleri v1.1'de eklenecek.
- Kontrolcü sürekli zamanlı.

---

## Yol haritası

- **v1.1:** Kapalı çevrim yük torku testi, ayrık zamanlı PID (Ts = 1 ms), çok adımlı referans dizisi.
- **v1.2:** Frekans analizi (Bode, kararlılık payları), kaskad hız/konum kontrolü karşılaştırması.
- **Sonrası:** Simulink Test / Requirements Toolbox, Model Advisor, kod üretimi ve MIL–SIL karşılaştırmalı test.

## Lisans

MIT (bkz. `LICENSE`).

# DC Motor Konum Kontrolü: Model Tabanlı Tasarım ve MIL Doğrulaması

*Türkçe özet. Tüm ayrıntılar, denklemler ve grafikler için [README.md](README.md) dosyasına bakın.*

Bir DC motorun konum kontrolü, MATLAB/Simulink'te model tabanlı tasarım (MBD) yaklaşımıyla geliştirildi:
ölçülebilir gereksinimler, teoriye karşı doğrulanmış fiziksel bir motor modeli, ölçümle gerekçelendirilmiş
kontrolcü kararları ve otomatik bir model-in-the-loop (MIL) test takımı.

> **Kapsam.** Proje, V-modelinin sol kolunu ve MIL doğrulamasını kapsar: gereksinimler, motor ve sensör
> modeli, kontrolcü tasarımı ve otomatik test. Kod üretimi ve SIL/HIL doğrulaması kapsam dışıdır ve
> yol haritasında yer alır.

**Durum (v1.1):** 28/28 kontrol PASS — 6 model doğrulama, 21 gereksinim, 1 test altyapısı kontrolü.

### v1.1'de yenilikler

- **Ayrık zamanlı kontrolcü** (Ts = 1 ms) ve gereksinimlerin nerede bozulduğunu gösteren örnekleme
  süresi taraması (100 ile 200 ms arasında).
- **Kapalı çevrim yük torku**, en kötü koşullarda (sıcak sargı, besleme düşüşü, artmış sürtünme).
  Elle hesaplanan kapasite, 16 simülasyonun hepsinde geç/kal sınırını doğru tahmin etti.
- **Çok adımlı referans** (0 → 50 → 20 → 80 rad). Tek adım testlerinin gizlediği yavaş bir oturma
  kuyruğunu ortaya çıkardı.
- **Regresyon altında yeniden düzenleme:** referans bloğu değiştirildi, grafikler ve açıklamalar
  temizlendi; her seferinde önceki bütün sonuçlar birebir aynı çıktı.

---

## Nasıl çalıştırılır

```matlab
openProject('DC_Motor_Position_Control_MBD.prj')   % yolu ayarlar, proje kokune gecer
run_all_tests                                       % ~70 simulasyon, 5-10 dakika
```

Sonuçlar `results/` klasörüne yazılır: `test_ozeti.csv` (gereksinim özeti), `test_ozeti_surekli.csv`
(sürekli zamanlı kontrolcüyle aynı özet, karşılaştırma için), `test_raporu.txt` (tam rapor) ve tüm
grafikler. Tüm parametreler tek dosyada: `models/init_motor.m`.

**Test edilen sürüm:** MATLAB/Simulink R2026a, Control System Toolbox.

---

## Sistem

- **Motor modeli:** Kütüphane bloğu yerine fiziksel denklemlerden kuruldu: elektriksel devre, mekanik
  dinamik, viskoz ve Coulomb sürtünmesi, yük torku girişi.
- **Sensör:** 4096 sayım/tur enkoder ve ölçüm gürültüsü.
- **Kontrolcü:** 1 ms'de çalışan ayrık PID (back-calculation anti-windup, ±12 V çıkış sınırı) ve buna
  paralel, hata tabanlı Coulomb sürtünme kompanzasyonu. Hata sinyali, bir örnekle-tut (ZOH) bloğuyla
  örnekleniyor.
- **Referans:** `init_motor.m` içindeki `ref_prof` zaman tablosu. Varsayılan değer v1.0'daki sabit 50 rad.
- **Model doğrulaması:** Açık çevrimde kalıcı hız, kapalı form teorik çözümle üç durumda yaklaşık
  2·10⁻⁶ bağıl hatayla örtüşüyor. Sürtünmesiz konum eğrisi, durum-uzayı modeliyle %0.03 içinde.
  Kapalı çevrimde yükü tutan voltaj, T·R/K_t ile %0.5 içinde örtüşüyor.

---

## Gereksinimler ve sonuçlar

| ID | Gereksinim | Ölçülen | Sınır | Sonuç |
|---|---|---|---|---|
| REQ-01 | 50 rad hedefe ulaşma | 4.03 s | sonlu | PASS |
| REQ-02 | Kalıcı hata | 4.2·10⁻⁵ rad | < 0.5 rad | PASS |
| REQ-03 | Aşım | %0.28 | < %5 | PASS |
| REQ-04 | Yerleşme süresi (%2 bandı) | 4.03 s | < 5 s | PASS |
| REQ-04w | En kötü koşullarda yerleşme süresi | 4.68 s | < 5 s | PASS |
| REQ-05 | Motor voltajı | 12.0 V | ≤ 12 V | PASS |
| REQ-06a | 9 V bozucuda en büyük sapma | 0.51 rad | ≤ 1 rad | PASS |
| REQ-06b | 3 V bozucu sonrası kalıcı hata | 1.7·10⁻⁵ rad | < 0.01 rad | PASS |
| REQ-06c | 9 V bozucuda ±0.05 rad bandına dönüş | 4.11 s | < 5 s | PASS |
| REQ-07a | Durağan voltaj titreşimi, enkoder + gürültü (AC rms) | 0.31 V | < 0.5 V | PASS |
| REQ-07b | Durağan voltaj titreşimi, gürültüsüz enkoder (AC rms) | 0.001 V | < 0.5 V | PASS |
| REQ-08 | R ve J ±%20 değişimde REQ-02…05 (9 durum), en kötü yerleşme | 4.37 s | hepsi geçmeli | PASS |
| REQ-09a | 0.4 N·m yük, en kötü koşul: en büyük sapma | 0.42 rad | ≤ 1 rad | PASS |
| REQ-09b | 0.4 N·m yük, en kötü koşul: kalıcı hata | 0.0037 rad | < 0.01 rad | PASS |
| REQ-09c | 0.4 N·m yük, en kötü koşul: ±0.05 rad bandına dönüş | 4.41 s | < 5 s | PASS |
| REQ-10a | 0 → 50 → 20 → 80 rad dizisi: en büyük aşım | %0.28 | < %5 | PASS |
| REQ-10b | Dizi: en uzun yerleşme | 4.56 s | < 5 s | PASS |
| REQ-10c | Dizi: aynı adımın durgun başlangıçlı haline göre fark | 0.003 s | < 0.05 s | PASS |
| REQ-10d | Dizi: sonraki adımdan önceki hata | 0.032 rad | < 0.05 rad | PASS |
| REQ-11a | Ts ≤ 10 ms'de REQ-02…05 ve 07a sağlanır (10 ms'de aşım) | %0.32 | hepsi geçmeli | PASS |
| REQ-11b | Gereksinimlerin sağlandığı en büyük örnekleme süresi | 100 ms | ≥ 50 ms | PASS |

En kötü koşul (REQ-04w, REQ-09): sargı direnci +%20 (sıcak motor), besleme 11 V, Coulomb sürtünmesi ×1.5.
En dar paylar: REQ-04w (0.32 s), REQ-10d (0.018 rad) ve REQ-09c (0.59 s).

**Sınırlar nereden geliyor:** Projede müşteri gereksinimi olmadığı için her sınır dört kaynaktan birine
dayanıyor: fiziksel limit (12 V besleme; 5 s, çünkü 50 rad tam hızda bile yaklaşık 3 s sürüyor), yaygın
kabul (%5 aşım, ζ ≈ 0.7'ye karşılık geliyor; %2 yerleşme bandı; parametrelerde ±%20), sistemin ölçeği
(1 rad = hareketin %2'si; 0.01 rad ≈ 6.5 enkoder adımı; ±0.05 rad ≈ 33 adım, gürültünün 50 katı) ve
analiz (0.4 N·m, en kötü kapasitenin yaklaşık %20 altı; 10 ms, bulunan örnekleme sınırının 10 kat gerisi).
Her sınırın ayrıntılı gerekçesi README.md'deki "Where the limits come from" tablosunda.

---

## Tasarım kararları

Her kontrolcü seçimi, alternatifler gereksinimlere karşı ölçülerek yapıldı.

1. **Anti-windup: back-calculation.** Anti-windup olmadan aşım %80, yerleşme 13.3 s. Back-calculation
   ile aşım %0.28, yerleşme 4.03 s.
2. **Türev filtresi N = 50 (PID Tuner'ın önerdiği 1010 yerine).** N = 1010 ile enkoder ve gürültü,
   motor dururken voltajı ±12 V arasında çırpındırıyordu. Bu, konum grafiğinde görünmüyordu. N = 50 ile
   titreşim 0.31 V'a indi (ayrık kontrolcüde N = 1010: 2.7 V, sürekli kontrolcüde 5.4 V), yerleşme
   yalnızca 0.03 s uzadı.
3. **Coulomb sürtünme kompanzasyonu.** Sürtünme eklenince konum hedefe çok yavaş sürünüyor ve REQ-06b
   geçmiyordu (0.017 rad). Kontrolcüye `Vc·tanh(e/e0)` biçiminde, Vc = 0.8 V olan bir ön besleme
   eklendi. Kalıcı hata yaklaşık 10 enkoder adımından adımın onda birinin altına indi. Kompanzasyon,
   gerçek bir kontrolcü gibi motorun **nominal** değerlerini kullanıyor.
4. **Kompanzasyon genişliği e0 = 0.01 rad.** e0 = 0.005 ile konum iki enkoder adımı arasında gidip
   geliyordu (limit çevrimi). e0 = 0.02 ile kompanzasyon hedef yakınında fazla zayıf kalıyordu.
   e0 = 0.01 hem en düşük hatayı hem en düşük titreşimi verdi.

---

## v1.1 analizleri

**Yük torku ve en kötü durum.** Motorun hareket halindeyken taşıyabileceği yük K_t·V/R − T_c.
Bu değer nominalde 0.70 N·m, en kötü koşulda 0.498 N·m. Bu basit hesap, 4 koşul × 4 yükten oluşan
16 simülasyonun hepsinde geç/kal sınırını doğru tahmin etti. Gereksinim bu yüzden nominale göre
(0.6 N·m) değil, en kötü koşula göre **0.4 N·m** olarak yazıldı. Tolerans analizi: 0.4 N·m ancak K_t
%17 daha zayıf, R nominalin 1.45 katı ya da sürtünme nominalin 3.5 katı olursa sağlanamıyor. Sonuç
yalnızca MIL'de doğrulandı, donanım testi kapsam dışı.

**Çok adımlı referans ve yavaş kuyruk.** Her adım, aynı büyüklükteki durgun başlangıçlı adımla
neredeyse birebir aynı davranıyor (fark ≤ 0.003 s). Yani kontrolcü bir hareketten ötekine iz taşımıyor.
Ama test bir bulgu ortaya çıkardı: motor %2 bandına 4 s'de giriyor, fakat 0.14 rad'lık küçük aşım
yaklaşık 1.4 s zaman sabitiyle söndüğü için 0.01 rad'a oturması yaklaşık 10 s sürüyor. Nedeni PID'in
integral terimi. İlk akla gelen "motor sürtünmede takılıyor" açıklaması, loglanan kompanzasyon voltajı
(her an ±0.8 V) ile çürütüldü. İyileştirme (2-DOF PID) v1.2'de planlandı.

**Ayrık kontrolcü ve örnekleme payı.** 1 ms'de sürekli zamanlı kontrolcüye göre pratikte fark yok.
Örnekleme süresi 1 ms'den 500 ms'ye taranınca gereksinimler 100 ms'ye kadar sağlanıyor, 200 ms'de sistem
±12 V arasında sürekli salınıma (limit çevrimi) giriyor. 200 ms'de aşım hâlâ %5'in altında; bozulmayı
yakalayan, kalıcı hata ve titreşim ölçümleri oldu. Tarama yalnızca nominal koşulda yapıldığı için
taahhüt 10 ms (sınırın 10 kat gerisi).

---

## Doğrulama yaklaşımı

- Motor modeli kontrolcüden bağımsız olarak, teoriye ve durum-uzayı modeline karşı doğrulandı.
- `check_logs`, referanstan motor konumuna kadar 17 denklemi loglanan sinyallerden yeniden hesaplıyor.
- Yapısal değişiklikler (sürtünme, yük girişi, kompanzasyon) önce sıfır değerle eklendi, sonuçların
  değişmediği kanıtlandıktan sonra açıldı.
- v1.1'de referans bloğu değiştirildi, açıklamalar ve test grafikleri temizlendi; her seferinde önceki
  bütün sonuçlar birebir aynı çıktıktan sonra commit edildi.
- Yük kapasitesi, tutma voltajı ve örnekleme sınırı simülasyondan önce elle tahmin edildi ve
  sonuçlarla karşılaştırıldı.
- `run_all_tests` her değişiklikten sonra tüm testleri yeniden çalıştırıyor.
- Hiçbir test modeli değiştirmiyor. Bu da her çalıştırmada otomatik olarak kontrol ediliyor.
- Tüm parametreler tek bir dosyadan (`init_motor.m`) geliyor.

---

## Bilinen sınırlar

- Motor parametreleri ölçülmedi, örnek değerler. Gerçek bir projede ilk adım parametre tanılama olurdu.
- `tanh` sürtünme modeli gerçek durma sürtünmesini (stiction) temsil etmiyor; sürücünün akım sınırı
  modellenmedi.
- Tc = 0.05 N·m ve 0.001 rad gürültü varsayımdır, veri sayfasından alınmadı.
- 0.4 N·m'yi tutmak sürekli 0.8 A çekiyor (sargıda yaklaşık 6 W). Motorun buna ısıl olarak dayanıp
  dayanamayacağı veri sayfasına bağlı.
- Kompanzasyon PID'nin iç sınırından sonra eklendiği için toplam komut 12.8 V'a çıkabiliyor. Dış sınır
  bunu 12 V'a kesiyor, ama PID'nin anti-windup'ı bu kırpmayı görmüyor.
- Kompanzasyon nominal parametreleri kullanıyor; en kötü koşulda kalıcı hata 3·10⁻⁵'ten 0.0037 rad'a
  çıkıyor (yine de REQ-09b içinde).
- 12 V ve üstü voltaj bozucuları genel olarak dengelenemez.
- Yavaş kuyruk: 0.01 rad'a oturmak yaklaşık 10 s sürüyor (REQ-10d bunu 0.05 rad ile sınırlıyor).
- Gerçek bir kontrolcüdeki hesaplama gecikmesi ve zamanlama sapması (jitter) modellenmedi; örnekleme
  taraması yalnızca nominal adımı kapsıyor.

---

## Yol haritası

- **v1.0:** Motor modeli ve doğrulaması, anti-windup'lı PID, sensör modeli, sürtünme kompanzasyonu,
  otomatik MIL test takımı (17 kontrol).
- **v1.1:** Kapalı çevrim yük torku ve en kötü durum analizi, çok adımlı referans, ayrık zamanlı
  kontrolcü ve örnekleme süresi çalışması (28 kontrol).
- **v1.2:** Yavaş kuyruğu gidermek için 2-DOF PID (set-point ağırlığı); ayrı bir Git dalında, tam
  regresyon altında geliştirilecek. Motora stiction ve sürücü akım sınırı eklenmesi.
- **Sonrası:** Frekans analizi (Bode, kararlılık payları), kaskad hız/konum kontrolü, Monte Carlo
  parametre analizi, Simulink Test / Requirements Toolbox, Model Advisor, kod üretimi ve MIL–SIL
  karşılaştırmalı test.

## Yapay Zeka Kullanımı

Bu proje sürecinde yapay zeka araçları bir eğitmen (tutor) ve gözden geçirici (reviewer) olarak kullanılmıştır. Yapay zeka; sorular ve ipuçlarıyla iş akışına rehberlik etmiş, modelimi ve betiklerimi mantık hatalarına karşı (sinyal akışı, anti-windup, sensör gürültüsü, metrik tanımları) gözden geçirmiş, ayrıca dosyaların düzenlenmesine ve dokümantasyon taslağının yazılmasına yardımcı olmuştur. Sürtünme telafisi ve otomatik test betikleri dahil olmak üzere bazı kısımlarda, yapay zekanın (Claude, Gemini) önerdiği çözümleri inceledim ve bunları kullanmadan önce mantığını adım adım çalıştım.

Tüm simülasyonlar MATLAB/Simulink ortamında çalıştırılmış ve sonuçlar yukarıda açıklandığı gibi bağımsız el hesaplarıyla kontrol edilmiştir. Bu repodaki her bloğun ve betiğin mantığını açıklayabilirim.
## Lisans

MIT (bkz. `LICENSE`).

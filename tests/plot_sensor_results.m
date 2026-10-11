% plot_sensor_results.m - REQ-07 grafikleri (sensor_test_sonuclari.mat dosyasindan)
load(fullfile('results', 'sensor_test_sonuclari.mat'), 'T', 'V', 'C');
n = size(C,1);

% ---- Grafik A: her durum ayri panelde ----
figure('Position', [100 100 1100 700]);
tl = tiledlayout(3, 2, 'TileSpacing', 'compact');
for i = 1:n
    nexttile;
    plot(V{i}.Time, V{i}.Data, 'LineWidth', 0.8); grid on;
    xlim([18 18.3]); ylim([-13 13]);
    title({C{i,1}, sprintf('Vac = %.2f V', T.Vac_V(i))}, ...
        'FontWeight', 'normal', 'FontSize', 9);
end
title(tl, 'Duragan halde motor voltaji (18.0 - 18.3 s)');
xlabel(tl, 'zaman (s)'); ylabel(tl, 'motor voltaji (V)');
exportgraphics(gcf, fullfile('results', 'sensor_testi_paneller.png'), 'Resolution', 200);

% ---- Grafik B: N odunlesimi, her sinir kendi ekseninde ve renginde ----
k  = 3:n;                                   % gurultulu satirlar
Nv = str2double(C(k,4));
mavi = [0 0.447 0.741];  turuncu = [0.85 0.325 0.098];

figure('Position', [100 100 850 500]);
yyaxis left
loglog(Nv, T.Vac_V(k), 'o-', 'LineWidth', 1.5, 'Color', mavi); hold on
yline(0.5, '--', 'REQ-07 siniri (0.5 V)', 'Color', mavi, ...
    'LabelHorizontalAlignment', 'left');
ylim([0.05 10]); ylabel('durağan voltaj titreşimi Vac (V)');

yyaxis right
semilogx(Nv, T.Yerlesme_s(k), 's-', 'LineWidth', 1.5, 'Color', turuncu); hold on
yline(5, '--', 'REQ-04 siniri (5 s)', 'Color', turuncu, ...
    'LabelHorizontalAlignment', 'left');
ylim([3.5 5.5]); ylabel('yerleşme süresi (s)');

xline(50, ':k', 'secilen N = 50', 'LabelVerticalAlignment', 'bottom');
xlim([10 2000]); grid on; xlabel('türev filtresi N');
title('Türev filtresi N: gürültü ve hız ödünleşimi');
exportgraphics(gcf, fullfile('results', 'sensor_N_odunlesimi.png'), 'Resolution', 200);
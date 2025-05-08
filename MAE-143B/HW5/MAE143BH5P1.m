%MAE-143B HW5 P1   

clc; close all; clear;

s = tf('s');

L = {
    ((s + 2)*(s + 6)) / (s*(s + 1)*(s + 5)*(s + 10)),
    1 / (s^2 + 3*s + 10),
    1 / (s^2 * (s + 8)),
    (s + 2) / (s * (s + 10) * (s^2 + 2*s + 2)),
    (s^2 + 1) / (s * (s^2 + 4))
};

for i = 1:length(L)
    figure;
    rlocus(L{i});
    grid on;
    title(['Root Locus of L_', num2str(i)]);
    saveas(gcf, ['rootlocus_' num2str(i) '.png']);
end






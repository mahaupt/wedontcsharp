%% Hinweise

% 1. Bitte in spaceballs.m die Initialisierung komplett auskommentieren
% 2. In spaceballs.m "spiel.zeitraffer_checkbox_anfangswert" auf true setzen

%% 1.) Statistik über beliebige Zufallswerte   &   Voreinstellungen

% A.) Bitte hier die eigene Farbe angeben:
meineFarbe = 'blau';

% B.) Bitte hier die Anzahl der gewünschten Durchgänge angeben:
Durchgaenge =5;

% C.) Zuerst Teil 1 über Strg+Enter (bzw. cmd+Enter) ausführen
%    nach getätigten Änderungen je nach Präferenz Teil 2 oder Teil 3 über Strg+Enter (bzw. cmd+Enter) ausführen

clc
stats(Durchgaenge, meineFarbe, 2);


%% 2.) Statistik über alle in 1. genutzten Zufallswerte
%      [z.b zur Überprüfung nach Veränderung in beschleunigung.m]

clc
stats([], [], 1);


%% 3.) Statistik über alle problematischen Seeds aus 1.)
%      [explizite Betrachtung der Seeds, bei denen verloren oder unent. gespielt wurde]

clc
stats([], [], 0);
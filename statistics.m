% Statistikprogramm 

%ANLEITUNG

% 1. bitte in spaceballs.m die Initialisierung komplett auskommentieren

% 2. bitte hier die eigene Farbe angeben:

meineFarbe = 'blau';

% 3. bitte hier die Anzahl der gewünschten Durchgänge angeben:

Durchgaenge = 3;

% 4. zuerst Teil 1 über Strg+Enter (bzw. cmd+Enter) ausführen
%    nach getätigten Änderungen je nach Präferenz Nr 2 oder Nr 3 über Strg+Enter (bzw. cmd+Enter) ausführen



%%1.) Statistik über beliebige Zufallswerte
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
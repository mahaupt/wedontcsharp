% Statistikprogramm 

mex -setup C++   % C++ compiler aktivieren
%% 1.) Statistik über beliebige Zufallswerte

clc
Durchgaenge = 10;
stats(Durchgaenge, 'blau', 2);


%% 2.) Statistik über alle in 1. genutzten Zufallswerte
%      [z.b zur Überprüfung nach Veränderung in beschleunigung.m]

clc
stats([], [], 1);


%% 3.) Statistik über alle problematischen Seeds aus 1.)
%      [explizite Betrachtung der Seeds, bei denen verloren oder unent. gespielt wurde]

clc
stats([], [], 0);
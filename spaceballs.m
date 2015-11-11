%% Initialisierung


% clear variables
% close all
% clc
% 
% %  rng shuffle
% %  r = round(rand(1)*1000000)
%  rng(200444);

%% Konstanten definieren

spiel.dt = 1/100;
spiel.t_end = 60;
spiel.n_t = round (spiel.t_end/spiel.dt);
spiel.i_t = 0;

spiel.n_mine = 12; % Muss gerade sein! (Default: 12)
spiel.n_tanke = 9; % Muss ungerade sein! (Default: 9)

spiel.kreis_radius = 0.075;
spiel.mine_radius = 0.05;
spiel.tanke_radius = 0.01;
spiel.spaceball_radius = 0.01;

spiel.bes = 0.1;

spiel.rot.getankt = 0;
spiel.blau.getankt = 0;

spiel.rot.punkte = 0;
spiel.blau.punkte = 0;

spiel.rot.ereignis = '';
spiel.blau.ereignis = '';

spiel.farbe.rot = hsv2rgb ([0.95 1 1]);
spiel.farbe.blau = hsv2rgb ([0.6 1 1]);
spiel.farbe.gruen = hsv2rgb ([0.4 1 0.8]);
spiel.farbe.hellrot = hsv2rgb ([0.95 0.4 1]);
spiel.farbe.hellblau = hsv2rgb ([0.6 0.4 1]);
spiel.farbe.grau = [0.4 0.4 0.4];
spiel.farbe.hellgrau = [0.8 0.8 0.8];

spiel.spur_anfangswert = 1*spiel.n_t;
spiel.zeitlupe_anfangswert = 0;
spiel.ges_checkbox_anfangswert = true;
spiel.bes_checkbox_anfangswert = true;
spiel.zeitraffer_checkbox_anfangswert = true;

spiel.rot.spur(spiel.n_t, 2) = 0;
spiel.blau.spur(spiel.n_t, 2) = 0;


%% Video abspeichern?

video.abspeichern = false;

if video.abspeichern
    
    video.writer = VideoWriter ('spaceballs.avi');
    
    video.writer.Quality = 25;
    
    video.writer.FrameRate = 25;
    
    open (video.writer)
    
end


%% Teams einlesen

spiel = teams_einlesen (spiel);


%% Spielfeld erstellen und darstellen

spiel = spielfeld_erstellen (spiel);

spiel = spielfeld_darstellen (spiel);

set (spiel.fenster_handle, 'UserData', true)


%% Spielen

tic;

while get (spiel.fenster_handle, 'UserData')
    
    spiel.i_t = spiel.i_t + 1;
    
    spiel = schritt (spiel, video);
    
end

toc;


%% Aufräumen

if video.abspeichern
    
    close (video.writer);
    
    delete (video.writer);
    
end
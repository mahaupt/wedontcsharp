function spiel = spielfeld_erstellen (spiel)

n_kreis = 2 + spiel.n_mine + spiel.n_tanke;

kreis(n_kreis).pos = [0; 0];

kreis(1).pos(1) = 1.5*spiel.spaceball_radius;
kreis(1).pos(2) = 1.5*spiel.spaceball_radius;
kreis(1).radius = spiel.spaceball_radius;

kreis(2).pos(1) = 1 - 1.5*spiel.spaceball_radius;
kreis(2).pos(2) = 1.5*spiel.spaceball_radius;
kreis(2).radius = spiel.spaceball_radius;

kreis(3).pos = [ ...
    0.5, ...
    spiel.kreis_radius + rand*(1 - 2*spiel.kreis_radius)];

kreis(3).radius = spiel.kreis_radius;

i_kreis = 4;

while i_kreis < n_kreis
    
    kreis(i_kreis).pos = rand (1, 2);
    
    kreis(i_kreis).radius = spiel.kreis_radius;
    
    kreis(i_kreis + 1) = kreis(i_kreis);
    kreis(i_kreis + 1).pos(1) = 1 - kreis(i_kreis).pos(1);
    
    kreise_schneiden = false;
    
    for i_vorhandener_kreis = 1 : i_kreis - 1
        
        if schnitt_kreis_kreis ( ...
                kreis(i_kreis), ...
                kreis(i_vorhandener_kreis))
            
            kreise_schneiden = true;
            
            break
            
        end
        
    end
    
    if ...
            ~ kreise_schneiden && ...
            ~ schnitt_kreis_kreis ( ...
            kreis(i_kreis), ...
            kreis(i_kreis + 1)) && ...
            ~ schnitt_kreis_bande (kreis(i_kreis))
        
        i_kreis = i_kreis + 2;
        
    end
    
end

spiel.rot.radius = kreis(1).radius;
spiel.rot.pos = kreis(1).pos;
spiel.rot.ges = [0 0];
spiel.rot.bes = [0 0];

spiel.blau.radius = kreis(2).radius;
spiel.blau.pos = kreis(2).pos;
spiel.blau.ges = [0 0];
spiel.blau.bes = [0 0];

spiel.tanke(1 : spiel.n_tanke) = kreis(3 : spiel.n_tanke + 2);

for i_tanke = 1 : spiel.n_tanke
    
    spiel.tanke(i_tanke).radius = spiel.tanke_radius;
    
end

spiel.mine(1 : spiel.n_mine) = kreis(spiel.n_tanke + 3 : n_kreis);

for i_mine = 1 : spiel.n_mine
    
    spiel.mine(i_mine).radius = spiel.mine_radius;
    
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% spiel.tanke(1).pos(1) = spiel.kreis_radius;
% spiel.tanke(1).pos(2) = 1 - spiel.kreis_radius;

% spiel.tanke(1).pos(1) = 1.5;
% spiel.tanke(1).pos(2) = 1.5;

% spiel.rot.pos = [spiel.spaceball_radius spiel.kreis_radius];
% spiel.rot.ges = [-0.25 0];

% spiel.rot.pos = [0.5 0.5];
% spiel.rot.pos = spiel.spaceball_radius*(1+eps)*[1 1];
% spiel.rot.ges = [0 0.28];
% spiel.rot.ges = [0 0];

% spiel.blau.pos = [0.45 0.5];
% spiel.blau.ges = [0 0.28];

% spiel.tanke(2).pos = [0.35, 1.5*spiel.spaceball_radius];
% spiel.tanke(3).pos = [0.6, 1.5*spiel.spaceball_radius];

% spiel.mine(1).pos = [spiel.kreis_radius+ 0.005, 0.5];
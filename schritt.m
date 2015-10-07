function spiel = schritt (spiel, video)


%% Zeit

spiel.t = spiel.i_t * spiel.dt;

if spiel.i_t == spiel.n_t
    
    spiel.rot.ereignis = 'Zeit ist abgelaufen.';
    spiel.blau.ereignis = 'Zeit ist abgelaufen.';
    
    set (spiel.fenster_handle, 'UserData', false);
    
end


%% Mine zerstören

if ...
        spiel.n_mine > 0 && ...
        spiel.n_tanke == 0 && ...
        mod (spiel.i_t, 300) == 0

    delete (spiel.mine(spiel.n_mine).graphics_handle);

    spiel.mine(spiel.n_mine) = [];
    
    spiel.n_mine = length (spiel.mine);
    
end


%% Beschleunigung

spiel.rot.bes = spiel.rot.beschleunigung_handle (spiel, 'rot');

rot_bes_norm = norm (spiel.rot.bes);

if rot_bes_norm > 1e-9
    
    spiel.rot.bes = spiel.bes * spiel.rot.bes / rot_bes_norm;
    
end

spiel.blau.bes = spiel.blau.beschleunigung_handle (spiel, 'blau');

blau_bes_norm = norm (spiel.blau.bes);

if blau_bes_norm > 1e-9
    
    spiel.blau.bes = spiel.bes * spiel.blau.bes / blau_bes_norm;
    
end


%% Eulerschritt

spiel.rot.ges = spiel.rot.ges + spiel.rot.bes * spiel.dt;

spiel.rot.pos = spiel.rot.pos + spiel.rot.ges * spiel.dt;

spiel.blau.ges = spiel.blau.ges + spiel.blau.bes * spiel.dt;

spiel.blau.pos = spiel.blau.pos + spiel.blau.ges * spiel.dt;


%% Tanken

zu_loeschende_tanken = [];

for i_tanke = 1 : spiel.n_tanke
    
    if schnitt_kreis_kreis (spiel.tanke(i_tanke), spiel.rot)
        
        spiel.rot.getankt = spiel.rot.getankt + 1;
        
        zu_loeschende_tanken = [zu_loeschende_tanken, i_tanke];
        
    end
    
    if schnitt_kreis_kreis (spiel.tanke(i_tanke), spiel.blau)
        
        spiel.blau.getankt = spiel.blau.getankt + 1;
        
        zu_loeschende_tanken = [zu_loeschende_tanken, i_tanke];
        
    end
    
end

if ~isempty (zu_loeschende_tanken)
    
    % Muss ein Vektor sein, wenn gleichzeitig mehrere Tanken gelöscht
    % werden sollen.
    delete ([spiel.tanke(zu_loeschende_tanken).graphics_handle]);
    
    spiel.tanke(zu_loeschende_tanken) = [];
    
    spiel.n_tanke = length (spiel.tanke);
    
end


%% Minen

rot_trifft_mine = false;

blau_trifft_mine = false;

for i_mine = 1 : spiel.n_mine
    
    if schnitt_kreis_kreis (spiel.mine(i_mine), spiel.rot)
        
        rot_trifft_mine = true;
        
        spiel.rot.ereignis = 'Rot trifft Mine.';
        
    end
    
    if schnitt_kreis_kreis (spiel.mine(i_mine), spiel.blau)
        
        blau_trifft_mine = true;
        
        spiel.blau.ereignis = 'Blau trifft Mine.';
        
    end
    
end


%% Banden

rot_trifft_bande = false;

blau_trifft_bande = false;

if schnitt_kreis_bande (spiel.rot)
    
    rot_trifft_bande = true;
    
    spiel.rot.ereignis = 'Rot trifft Bande.';
    
end

if schnitt_kreis_bande (spiel.blau)
    
    blau_trifft_bande = true;
    
    spiel.blau.ereignis = 'Blau trifft Bande.';
    
end


%% Auswertung

if ...
        rot_trifft_mine || ...
        blau_trifft_mine || ...
        rot_trifft_bande || ...
        blau_trifft_bande
    
    set (spiel.fenster_handle, 'UserData', false);
    
end

if ...
        (rot_trifft_mine || ...
        rot_trifft_bande) && ...
        ~blau_trifft_mine && ...
        ~blau_trifft_bande
    
    spiel.blau.ereignis = 'Blau gewinnt.';
    
    spiel.blau.punkte = 1;
    
elseif ...
        (blau_trifft_mine || ...
        blau_trifft_bande) && ...
        ~rot_trifft_mine && ...
        ~rot_trifft_bande
    
    spiel.rot.ereignis = 'Rot gewinnt.';
    
    spiel.rot.punkte = 1;
    
end


%% Rot und Blau treffen einander

if schnitt_kreis_kreis (spiel.rot, spiel.blau)
    
    if spiel.rot.getankt > spiel.blau.getankt
        
        spiel.rot.ereignis = 'Rot trifft Blau.';
        
        spiel.blau.ereignis = 'Rot trifft Blau.';
        
        spiel.rot.punkte = 1;
        
        set (spiel.fenster_handle, 'UserData', false);
        
    elseif spiel.rot.getankt < spiel.blau.getankt
        
        spiel.rot.ereignis = 'Blau trifft Rot.';
        
        spiel.blau.ereignis = 'Blau trifft Rot.';
        
        spiel.blau.punkte = 1;
        
        set (spiel.fenster_handle, 'UserData', false);
        
    end
    
end


%% Spur

spiel.rot.spur(spiel.i_t, :) = spiel.rot.pos;

spiel.blau.spur(spiel.i_t, :) = spiel.blau.pos;


%% Spielfeld grafisch aktualisieren

spielfeld_aktualisieren (spiel, video);
function spielfeld_aktualisieren (spiel, video)

%% Zeitbalken

set (spiel.zeit_handle, ...
    'Position', [0, 590/600, spiel.i_t/spiel.n_t, 10/690] ...
    );


%% Tankanzeige

set (spiel.rot.getankt_handle, ...
    'String', ['Getankt: ', num2str(spiel.rot.getankt)] ...
    );

set (spiel.blau.getankt_handle, ...
    'String', ['Getankt: ', num2str(spiel.blau.getankt)] ...
    );


%% Punkteanzeige

set (spiel.rot.punkte_handle, ...
    'String', ['Punkte: ', num2str(spiel.rot.punkte)] ...
    );

set (spiel.blau.punkte_handle, ...
    'String', ['Punkte: ', num2str(spiel.blau.punkte)] ...
    );


%% Ereignisanzeige

set (spiel.rot.ereignis_handle, ...
    'String', spiel.rot.ereignis ...
    );

set (spiel.blau.ereignis_handle, ...
    'String', spiel.blau.ereignis ...
    );


%% Spur

n_spur = round (get (spiel.spur_slider_handle, 'Value'));

set (spiel.rot.spur_handle, ...
    'XData', spiel.rot.spur ... 
    (max (spiel.i_t - n_spur, 1) : spiel.i_t, 1), ...
    'YData', spiel.rot.spur ...
    (max (spiel.i_t - n_spur, 1) : spiel.i_t, 2) ...
    );

set (spiel.blau.spur_handle, ...
    'XData', spiel.blau.spur ...
    (max (spiel.i_t - n_spur, 1) : spiel.i_t, 1), ...
    'YData', spiel.blau.spur ...
    (max (spiel.i_t - n_spur, 1) : spiel.i_t, 2) ...
    );


%% Spaceballs

set (spiel.rot.spaceball_handle, ...
    'Position', ...
    [...
    spiel.rot.pos(1) - spiel.rot.radius, ...
    spiel.rot.pos(2) - spiel.rot.radius, ...
    2*spiel.rot.radius, ...
    2*spiel.rot.radius])

set (spiel.blau.spaceball_handle, ...
    'Position', ...
    [...
    spiel.blau.pos(1) - spiel.blau.radius, ...
    spiel.blau.pos(2) - spiel.blau.radius, ...
    2*spiel.blau.radius, ...
    2*spiel.blau.radius])


%% Geschwindigkeit

if get (spiel.ges_checkbox_handle, 'Value')
    
    set (spiel.rot.ges_handle, ...
        'XData', [ ...
        spiel.rot.pos(1), ...
        spiel.rot.pos(1) + 0.5*spiel.rot.ges(1)], ...
        'YData', [ ...
        spiel.rot.pos(2), ...
        spiel.rot.pos(2) + 0.5*spiel.rot.ges(2)] ...
        );
    
    set (spiel.rot.ges_handle, 'Visible', 'on');
    
    set (spiel.blau.ges_handle, ...
        'XData', [ ...
        spiel.blau.pos(1), ...
        spiel.blau.pos(1) + 0.5*spiel.blau.ges(1)], ...
        'YData', [ ...
        spiel.blau.pos(2), ...
        spiel.blau.pos(2) + 0.5*spiel.blau.ges(2)] ...
        );
    
    set (spiel.blau.ges_handle, 'Visible', 'on');
    
else
    
    set (spiel.rot.ges_handle, 'Visible', 'off');
    
    set (spiel.blau.ges_handle, 'Visible', 'off');
    
end

%% Beschleunigung

if get (spiel.bes_checkbox_handle, 'Value')
    
    dreh_trafo = [0 -1; 1 0];
    
    rot_bes_norm = norm (spiel.rot.bes);
    
    if rot_bes_norm > 1e-9
        
        rot_bes_einheit = spiel.rot.bes/norm (spiel.rot.bes);
        
        rot_bes_einheit_gedreht = dreh_trafo*rot_bes_einheit';
        
        rot_linke_ecke = ...
            spiel.rot.pos + ...
            spiel.rot.radius*rot_bes_einheit_gedreht';
        
        rot_rechte_ecke = ...
            spiel.rot.pos - ...
            spiel.rot.radius*rot_bes_einheit_gedreht';
        
        rot_hintere_ecke = ...
            spiel.rot.pos - ...
            4*spiel.rot.radius*rot_bes_einheit;
        
        set (spiel.rot.bes_handle, ...
            'XData', [ ...
            rot_linke_ecke(1), ...
            rot_rechte_ecke(1), ...
            rot_hintere_ecke(1)], ...
            'YData', [ ...
            rot_linke_ecke(2), ...
            rot_rechte_ecke(2), ...
            rot_hintere_ecke(2)] ...
            );
        
        set (spiel.rot.bes_handle, 'Visible', 'on');
        
    else
        
        set (spiel.rot.bes_handle, 'Visible', 'off');
        
    end
    
        blau_bes_norm = norm (spiel.blau.bes);
    
    if blau_bes_norm > 1e-9
        
        blau_bes_einheit = spiel.blau.bes/norm (spiel.blau.bes);
        
        blau_bes_einheit_gedreht = dreh_trafo*blau_bes_einheit';
        
        blau_linke_ecke = ...
            spiel.blau.pos + ...
            spiel.blau.radius*blau_bes_einheit_gedreht';
        
        blau_rechte_ecke = ...
            spiel.blau.pos - ...
            spiel.blau.radius*blau_bes_einheit_gedreht';
        
        blau_hintere_ecke = ...
            spiel.blau.pos - ...
            4*spiel.blau.radius*blau_bes_einheit;
        
        set (spiel.blau.bes_handle, ...
            'XData', [ ...
            blau_linke_ecke(1), ...
            blau_rechte_ecke(1), ...
            blau_hintere_ecke(1)], ...
            'YData', [ ...
            blau_linke_ecke(2), ...
            blau_rechte_ecke(2), ...
            blau_hintere_ecke(2)] ...
            );
        
        set (spiel.blau.bes_handle, 'Visible', 'on');
        
    else
        
        set (spiel.blau.bes_handle, 'Visible', 'off');
        
    end
    
else
    
    set (spiel.rot.bes_handle, 'Visible', 'off');
    
    set (spiel.blau.bes_handle, 'Visible', 'off');
    
end


%% Farbe

if spiel.rot.getankt > spiel.blau.getankt
    
    set (spiel.blau.spaceball_handle, ...
        'Facecolor', spiel.farbe.hellblau)
    
    set (spiel.blau.bes_handle, ...
        'Facecolor', spiel.farbe.hellblau)
    
    set (spiel.blau.ges_handle, ...
        'Color', spiel.farbe.hellgrau)
    
    set (spiel.blau.spur_handle, ...
        'Color', spiel.farbe.hellblau)
    
elseif spiel.rot.getankt < spiel.blau.getankt
    
    set (spiel.rot.spaceball_handle, ...
        'Facecolor', spiel.farbe.hellrot)
    
    set (spiel.rot.bes_handle, ...
        'Facecolor', spiel.farbe.hellrot)
    
    set (spiel.rot.ges_handle, ...
        'Color', spiel.farbe.hellgrau)
    
    set (spiel.rot.spur_handle, ...
        'Color', spiel.farbe.hellrot)
    
else
    
    set (spiel.rot.spaceball_handle, ...
        'Facecolor', spiel.farbe.rot)
    
    set (spiel.blau.spaceball_handle, ...
        'Facecolor', spiel.farbe.blau)
    
    set (spiel.rot.bes_handle, ...
        'Facecolor', spiel.farbe.rot)
    
    set (spiel.blau.bes_handle, ...
        'Facecolor', spiel.farbe.blau)
    
    set (spiel.rot.ges_handle, ...
        'Color', [0 0 0])
    
    set (spiel.blau.ges_handle, ...
        'Color', [0 0 0])
    
    set (spiel.rot.spur_handle, ...
        'Color', spiel.farbe.rot)
    
    set (spiel.blau.spur_handle, ...
        'Color', spiel.farbe.blau)
    
end


%% Video, Zeitraffer und Zeitlupe

if video.abspeichern
    
    if mod (spiel.i_t, 4) == 0
        
        frame = getframe(spiel.fenster_handle);
        
        writeVideo (video.writer, frame)
        
    end
    
elseif get (spiel.zeitraffer_checkbox_handle, 'Value')
    
    drawnow limitrate
    
elseif get (spiel.zeitlupe_slider_handle, 'Value') > 0
    
    pause (get (spiel.zeitlupe_slider_handle, 'Value'))
    
else
    
    drawnow
    
end
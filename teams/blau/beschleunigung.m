function bes = beschleunigung(spiel, farbe)
    bes = [0, 1];
    
    % Teamfarben-Abfrage und Parameter-Zuweisung
    if strcmp(farbe, 'blau')
        ich = spiel.blau;
        gegner = spiel.rot;
    else
        ich = spiel.rot
        gegner = spiel.blau
    end
   
    % Blödsinniger Test xD
    if spiel.i_t<120
        bes=[0,1];
    else
        bes = [-1,-0.5]; 
    end
    
    
end
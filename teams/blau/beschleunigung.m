function bes = beschleunigung(spiel, farbe)
    %statische variablen definieren
    persistent nodeGrid;
    
    
    %Klassen Initialisieren
    if isempty(nodeGrid)
        nodeGrid = minheap();
    end
    
    bes = [0, 1];
    
    
    
end
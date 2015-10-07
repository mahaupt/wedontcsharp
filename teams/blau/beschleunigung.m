function bes = beschleunigung(spiel, farbe)
    %statische variablen definieren
    persistent nodeGrid;
    
    
    %Klassen Initialisieren
    if isempty(nodeGrid)
        nodeGrid = minheap();
    end
    
    bes = [0, 1];
    
    
    node1 = node(true, [1, 1], 5, 6);
    node1.gCost = 10;
    node1.hCost = 4;
    nodeGrid.add(node1);
    nodeGrid.contains(node1);
    
    if (nodeGrid.count > 10)
        nodeGrid.removeFirst();
    end
    
    
end
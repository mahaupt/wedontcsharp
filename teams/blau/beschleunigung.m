function bes = beschleunigung(spiel, farbe)
    %Konstanten
    constSafeBorder = 0.001;
    
    %statische variablen definieren
    persistent nodeGrid;
    %persistent waypointList;
    
    
    %%wird einmal am Anfang ausgeführt
    if (isempty(nodeGrid))
        setupNodeGrid() 
    end
    
    bes = [0, 1];
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function setupNodeGrid()
        gridRadius = 0.01;
        
        gridSizeX = round(1/(gridRadius*2));
        gridSizeY = round(1/(gridRadius*2));
        
        %create grid
        for x = 1 : gridSizeX+1
            for y = 1 : gridSizeY+1
                worldPos = [gridRadius*(x*2-1), gridRadius*(y*2-1)];
                gridPos = [x, y];
                nodeGrid(x,y).worldPos = worldPos;
                nodeGrid(x,y).gridPos = gridPos;
                nodeGrid(x,y).isWalkable = isWalkable(worldPos);            
            end
        end
    end

    %false wenn Wand oder Mine, true wenn frei
    function erg=isWalkable(pos)
        erg = true;
        secureSpaceballRadius = constSafeBorder + spiel.spaceball_radius;
        
        %border check
        if (pos(1) > 1-secureSpaceballRadius || pos(1) < secureSpaceballRadius || pos(2) > 1-secureSpaceballRadius || pos(2) < secureSpaceballRadius)
            erg = false;
            return
        end
        
        %mine check
        for i = 1 : spiel.n_mine
            if (norm(spiel.mine(i).pos-pos) < spiel.mine(i).radius+secureSpaceballRadius)
                erg = false;
                return;
            end
        end
    end
end
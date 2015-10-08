function bes = beschleunigung(spiel, farbe)
    %Konstanten
    constSafeBorder = 0.01;
    constGridRadius = 0.005;
    constNavSecurity = 0.001;
   
    %statische variablen definieren
    persistent nodeGrid;
    persistent waypointList;
    persistent visited_tanke
    
    %%Farbe prüfen und zuweisen
    if strcmp (farbe, 'rot')
        me = spiel.rot;
        enemy = spiel.blau;
    else
        me = spiel.blau;
        enemy = spiel.rot;
    end
    
    %%wird einmal am Anfang ausgeführt
    if spiel.i_t==1
        setupNodeGrid();
        waypointList = findPath(me.pos, spiel.tanke(1).pos);
        visited_tanke = 1;
        debugDRAW();
    end
    
    %%%Nach Erreichen des ersten Wegpunktes: Waypointlist neu aufbauen


    if numel(waypointList) <= 0 && visited_tanke <= spiel.n_tanke
        waypointList = findPath(me.pos, spiel.tanke(visited_tanke).pos);
        visited_tanke=visited_tanke+1;
    end
         
    
    %%Beschleunigung berechnen:
    bes=calculateBES();

    
    %%%%%%%%%%%%%%%%%%%%%%%%PathToBes%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function erg=calculateBES()
    %    if (numel(waypointList) <= 0)
    %        erg = -me.ges;
    %        return;
    %    end
        
        corr = vecNorm(waypointList{1}-me.pos)-vecNorm(me.ges);
        dir = vecNorm(waypointList{1}-me.pos);
        erg = dir + corr;
        distancetowaypoint=norm(waypointList{1}-me.pos);
       
        if (norm(me.ges) / (spiel.bes)) * (norm(me.ges) / 2) > distancetowaypoint
            erg=-me.ges+ corr*0.5;
        end
        
        %%Überprüfen, ob Wegpunkt erreicht wurde, dann 1. Punkt löschen
        if norm(me.pos-waypointList{1}) < 0.01
            waypointList(1) = [];
        end
    end
        

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function setupNodeGrid()
        gridSizeX = round(1/(constGridRadius*2));
        gridSizeY = round(1/(constGridRadius*2));
        
        %create grid
        for x = 1 : gridSizeX+1
            for y = 1 : gridSizeY+1
                worldPos = [constGridRadius*2*x, constGridRadius*2*y];
                gridPos = [x, y];
                nodeGrid(x,y).worldPos = worldPos;
                nodeGrid(x,y).gridPos = gridPos;
                nodeGrid(x,y).isWalkable = isWalkable(worldPos);
                nodeGrid(x,y).hCost = 0;
                nodeGrid(x,y).fCost = 0;
                nodeGrid(x,y).gCost = 0;
            end
        end
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Kollisionscheck
    function erg=isWalkable(pos)
        erg = true;
        secureSpaceballRadius = constSafeBorder + spiel.spaceball_radius;
        
        %border check
        if (pos(1) > 1-secureSpaceballRadius || pos(1) < secureSpaceballRadius || pos(2) > 1-secureSpaceballRadius || pos(2) < secureSpaceballRadius)
            erg = false;
            return;
        end
        
        %mine check
        for i = 1 : spiel.n_mine
            if (norm(spiel.mine(i).pos-pos) < spiel.mine(i).radius+secureSpaceballRadius)
                erg = false;
                return;
            end
        end
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Pathfinder
    function waypoints = findPath(startp, endp)
        pathSuccess = false; % - Pfad gefunden
        
        startPos = worldPosToGridPos(startp);
        endPos = worldPosToGridPos(endp);
        
        openSet = {startPos};
        closedSet = {};
        closedSetIndex = 1;
        
        while(numel(openSet) > 0)
            currentNode = nodeFromGridCoords(openSet{1});
            openSetIndex = 1;
            
            %get node woth lowest fcost (or hcost) and remove it from openlist
            for i=1:numel(openSet)
                cn = nodeFromGridCoords(openSet{i});
                if (cn.fCost < currentNode.fCost || cn.fCost == currentNode.fCost && cn.hCost < currentNode.hCost)
                    currentNode = nodeFromGridCoords(openSet{i});    
                    openSetIndex = i;
                end
            end
            
            %remove node from open set
            openSet(openSetIndex) = [];
            %add node to closed set
            closedSet{closedSetIndex} = currentNode.gridPos;
            closedSetIndex = closedSetIndex + 1;
            
            %if it is target - close - path found!
            if (currentNode.gridPos == endPos)
                pathSuccess = true;
                break;
            end
            
            %calculate neighbour cost indices
            neighbours = getNeighbourNodes(currentNode);
            for i = 1 : numel(neighbours)
                neighbour = neighbours(i);
                if (~neighbour.isWalkable || containsNode(closedSet, neighbour.gridPos))
                    continue;
                end
                
                %update costs for neighbours
                movementCostToNeighbour = currentNode.gCost + norm(currentNode.worldPos - neighbour.worldPos);
                if (movementCostToNeighbour < neighbour.gCost || ~containsNode(openSet, neighbour.gridPos))
                    nodeGrid(neighbour.gridPos(1), neighbour.gridPos(2)).gCost = movementCostToNeighbour;
                    nodeGrid(neighbour.gridPos(1), neighbour.gridPos(2)).hCost = norm(endp - neighbour.worldPos);
                    nodeGrid(neighbour.gridPos(1), neighbour.gridPos(2)).fCost = neighbour.gCost + neighbour.hCost;
                    nodeGrid(neighbour.gridPos(1), neighbour.gridPos(2)).parent = currentNode.gridPos;

                    %add neighbour to openSet
                    if (~containsNode(openSet, neighbour.gridPos))
                        insertIndex = numel(openSet)+1;
                        openSet{insertIndex} = neighbour.gridPos;
                    end
                end
            end
        end
        
        %finished pathfinding
        if (pathSuccess)
            %retrace path
            currentNode = nodeFromGridCoords(endPos);
            waypoints = [];
            waypointIndex = 1;
            
            while (currentNode.gridPos ~= startPos)
               waypoints{waypointIndex} = currentNode.worldPos;
               waypointIndex = waypointIndex + 1;
               currentNode = nodeFromGridCoords(currentNode.parent);
            end
            
            %flip waypoints
            waypoints = simplifyPath(fliplr(waypoints));
        else
            waypoints = [];
        end
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function erg = worldPosToGridPos(pos)
        erg = [round(pos(1)/constGridRadius/2), round(pos(2)/constGridRadius/2)];
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function erg = nodeFromGridCoords(pos)
        erg = nodeGrid(pos(1), pos(2));
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %check if nodes are equal
    function erg = equalsNode(a, b)
        erg = false;
        if (a.gridPos(1) == b.gridPos(1) && a.gridPos(2) == b.gridPos(2))
            erg = true;
        end
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %get node neighbours
    function erg = getNeighbourNodes(node)
        gridSizeX = round(1/(constGridRadius*2));
        gridSizeY = round(1/(constGridRadius*2));
        
        i = 1;
        for x = node.gridPos(1) -1 : node.gridPos(1) +1
            for y = node.gridPos(2) -1 : node.gridPos(2) + 1
                %check if grid coors are valid
                if (x >= 1 && x <= gridSizeX && y >= 1 && y <= gridSizeY)
                
                    if (equalsNode(nodeGrid(x, y), node))
                        continue;
                    end

                    erg(i) = nodeGrid(x, y);
                    i = i + 1;
                end
            end
        end
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % simplify path
    function erg = simplifyPath(path)
        %erg=path;
        %return
        ergIndex = 1;
        lastvec=[0,0];
        for i=2:numel(path)
            thisvec = path{i}-path{i-1};
            if (~(abs(thisvec(1)-lastvec(1)) < constNavSecurity && abs(thisvec(2)-lastvec(2)) < constNavSecurity))
                erg{ergIndex} = path{i-1};
                ergIndex = ergIndex + 1;
            end
            lastvec=thisvec;
        end
        
        erg{ergIndex} = path{numel(path)};
    end


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %check if array contains node
    function erg = containsNode(nodes, pos)
        erg = false;
        
        for i=1:numel(nodes)
            if (nodes{i}(1) == pos(1) && nodes{i}(2) == pos(2))
                erg = true;
                return;
            end
        end
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %normalize 2D vector
    function erg = vecNorm(vec)
        n = norm(vec);
        erg = [vec(1)/n, vec(2)/n];
        
        if (n == 0)
            erg = 0;
        end
    end

%%%%%%%%%DEBUGGING%%%%%%%
    function debugDRAW()
        for i = 1 : numel(waypointList)

            rectangle ( ...
                'Parent', spiel.spielfeld_handle, ...
                'Position', [...
                waypointList{i}, ...
                0.01, ...
                0.01], ...
                'Curvature', [1 1], ...
                'FaceColor', spiel.farbe.rot, ...
                'EdgeColor', 'none' ...
                );
        end
    end
end
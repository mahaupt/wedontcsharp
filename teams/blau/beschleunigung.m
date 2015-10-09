function bes = beschleunigung(spiel, farbe)
    %Konstanten
    constSafeBorder = 0.005; %collision border around mines
    constGridRadius = 0.005; 
    constNavSecurity = 0.03; %simplify path
    constWayPointReachedRadius = 0.02; %0.01
    constMineProxPenality = 0.0006;
   
    %statische variablen definieren
    persistent nodeGrid;
    persistent waypointList;
    
    %%Farbe prüfen und zuweisen
    if strcmp (farbe, 'rot')
        me = spiel.rot;
        %enemy = spiel.blau;
    else
        me = spiel.blau;
        %enemy = spiel.rot;
    end
    
    %%wird einmal am Anfang ausgeführt
    if spiel.i_t==1
        setupNodeGrid()
        createPathToTanken()
        debugDRAW();
    end

    %%Beschleunigung berechnen:
    bes=calculateBES();

    
    %%%%%%%%%%%%%%%%%%%%%%%%PathToBes%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function erg=calculateBES()
        if (numel(waypointList) <= 0)
            erg = -me.ges;
            return;
        end
        
        %acceleration
        corr = vecNorm(waypointList{1}-me.pos)-vecNorm(me.ges);
        dir = vecNorm(waypointList{1}-me.pos);
        erg = dir + corr*5;
        
        %decelerating to stop
        decellerateBias = 0;
        if (numel(waypointList) > 1)
            decellerateBias = 0.01;
            if (decellerateBias > norm(me.ges))
                decellerateBias = norm(me.ges);
            end
        end
        
        %decelleration
        distancetowaypoint=norm(waypointList{1}-me.pos);
        if ((norm(me.ges) - decellerateBias) / (spiel.bes) * (norm(me.ges) - decellerateBias)/2) > distancetowaypoint
            erg=-dir + corr*5;
        end
        
        %%Überprüfen, ob Wegpunkt erreicht wurde, dann 1. Punkt löschen
        if norm(me.pos-waypointList{1}) < constWayPointReachedRadius
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
                mineCost = 0;
                
                %Je dichter an Mine, desto teurer!
                for i=1:spiel.n_mine
                    if (norm(nodeGrid(x,y).worldPos-spiel.mine(i).pos)-spiel.mine_radius < 0.1)
                        mineCost = mineCost + constMineProxPenality/(norm(nodeGrid(x,y).worldPos-spiel.mine(i).pos)-spiel.mine_radius);
                    end
                end
                
                nodeGrid(x,y).mineCost = mineCost;
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
        
        startPos = getValidNodePos(worldPosToGridPos(startp));
        endPos = getValidNodePos(worldPosToGridPos(endp));
        
        openSet = {startPos};
        closedSet = {};
        closedSetIndex = 1;
        
        startNode = nodeFromGridCoords(startPos);
        endNode = nodeFromGridCoords(endPos);
        
        %if start and end node is invalid - abort
        if (~startNode.isWalkable || ~endNode.isWalkable)
            waypoints = [];
            disp('findPath: invalid start or end position');
            return;
        end
        
        %find path...
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
                    nodeGrid(neighbour.gridPos(1), neighbour.gridPos(2)).fCost = movementCostToNeighbour + norm(endp - neighbour.worldPos) + neighbour.mineCost;
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
    % if node is not walkable, check for valid node in neighbours
    function erg = getValidNodePos(gridPos)
        node = nodeFromGridCoords(gridPos);
        erg = gridPos;
        
        %nothing to do
        if (node.isWalkable)
            return;
        end
        
        %check neighbours
        neighbours = getNeighbourNodes(node);
        for i = 1 : numel(neighbours)
            neighbour = neighbours(i);
            
            if (neighbour.isWalkable)
                erg = neighbour.gridPos;
                return;
            end
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
        if (numel(path) == 1)
            erg = path;
            return;
        end
        
        %check collision tube
        for i=fliplr(1:numel(path))
            if (~corridorColliding(path{i}, path{1}, constNavSecurity) || ... 
                    i == 2)
                %no collision detected start to end
                erg = {path{1}, path{i}};
                endIndex = numel(path);
                erg = appendToArray(erg, simplifyPath(path(i:endIndex)));
                return;
            end
        end
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

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %append to existing cell arrray
    function erg = appendToArray(array1, array2)
        array1index = numel(array1)+1;
        erg = array1;
        
        for i=1 : numel(array2)
            erg{array1index} = array2{i};
            array1index = array1index + 1;
        end
    end
    
%%%Search for nearest Tanken and create Path between them
    function createPathToTanken()
        tankdistance=createtankdistance();
        first_tanke = tankdistance(1,1);
        second_tanke = tankdistance(2,1);
        third_tanke = tankdistance(3,1);
        fourth_tanke = tankdistance(4,1);
        fifth_tanke = tankdistance(5,1);
        sixth_tanke = tankdistance(6,1);
        waypointList = findPath(me.pos, spiel.tanke(first_tanke).pos);
        waypointList = appendToArray(waypointList, findPath(spiel.tanke(first_tanke).pos, spiel.tanke(second_tanke).pos));
        waypointList = appendToArray(waypointList, findPath(spiel.tanke(second_tanke).pos, spiel.tanke(third_tanke).pos));
        waypointList = appendToArray(waypointList, findPath(spiel.tanke(third_tanke).pos, spiel.tanke(fourth_tanke).pos));
        waypointList = appendToArray(waypointList, findPath(spiel.tanke(fourth_tanke).pos, spiel.tanke(fifth_tanke).pos));
        waypointList = appendToArray(waypointList, findPath(spiel.tanke(fifth_tanke).pos, spiel.tanke(sixth_tanke).pos));
    end

%%%%%%%%%%%%%
%create Tank Distance Table
    function tankdistance=createtankdistance()

        tankdistance = zeros(spiel.n_tanke,2);
        for i=1:spiel.n_tanke
            tankdistance(i,1) = i;
            tankdistance(i,2) = norm(spiel.tanke(i).pos-me.pos);
        end
        tankdistance=sortrows(tankdistance,[2 1]);
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %corridor colliding
    function erg = corridorColliding(startp, endp, radius)
        length = 0;
        dir = vecNorm(endp-startp);
        erg = false;
        
        while(length < norm(startp - endp))
            pos = startp + dir*length;
            for i = 1:spiel.n_mine
                if (norm(pos-spiel.mine(i).pos)-spiel.mine_radius < radius)
                    erg = true;
                end
            end
            length = length + radius;
        end
    end

%%%%%%%%%DEBUGGING%%%%%%%
    function debugDRAW()
        for i = 1 : numel(waypointList)

            rectangle ( ...
                'Parent', spiel.spielfeld_handle, ...
                'Position', [...
                waypointList{i}-0.0025, ...
                0.005, ...
                0.005], ...
                'Curvature', [1 1], ...
                'FaceColor', spiel.farbe.rot, ...
                'EdgeColor', 'none' ...
                );
        end
    end
end
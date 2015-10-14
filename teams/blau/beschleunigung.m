function bes = beschleunigung(spiel, farbe)

    %Konstanten
    constSafeBorder = 0.001; %collision border around mines
    constGridRadius = 0.003;
    constNavSecurity = 0.03; %simplify path
    constWayPointReachedRadius = 0.02; %0.01
    constMineProxPenality = 0.0006; %Strafpunkte für Nodes - je dichter an Mine, desto höher
    constCornerBreaking = 0.3; %0.03 je größer der Winkel zum nächsten Wegpunkt, desto höheres Bremsen. Faktor.
    constCompetitionModeThreshold = 0.075;
   
    
    %statische variablen definieren
    persistent nodeGrid;
    persistent waypointList;
    persistent drawHandles; %debug drawing
    persistent ArrayOfMines; %Zur Bestimmung des Minenverschwindens benötigt
    persistent StartNumberOfTank; %Zur Entscheidung über Angriff und Tanken benötigt
    persistent NumberOfTank; %Momentane Anzahl der Tankstellen
    persistent ignoreTanke; %number of tanke to be ignored by targetNextTanke
    persistent tankeCompetition;
    persistent waitForEnemy;
    
    
    %%Farbe prüfen und zuweisen
    if strcmp (farbe, 'rot')
        me = spiel.rot;
        enemy = spiel.blau;
    else
        me = spiel.blau;
        enemy = spiel.rot;
    end
    
    
    %%wird einmal am Anfang ausgeführt
    %setup node grid and empty persistent vars
    if spiel.i_t==1
        nodeGrid = [];
        drawHandles = [];
        waypointList = [];
        ignoreTanke = 0;
        ArrayOfMines = spiel.mine;
        StartNumberOfTank = spiel.n_tanke;
        NumberOfTank = spiel.n_tanke;
        tankeCompetition = false;
        waitForEnemy = false;
        setupNodeGrid();
    end
    
    
    %Nodegrid beim Verschwinden einer Mine aktualisieren:
    if numel(spiel.mine) < numel(ArrayOfMines)
        disp('Updating NodeGrid');
        NumberOfMine = customSetdiff(spiel.mine, ArrayOfMines);
        updateNodeGrid(NumberOfMine.pos, spiel.mine_radius);
        waypointList = simplifyPath(waypointList);
        ArrayOfMines = spiel.mine;
    end
    
    %beim Verschwinden einer Tanke:
    if (NumberOfTank ~= spiel.n_tanke)
        NumberOfTank = spiel.n_tanke;
        tankeCompetition = false;
        ignoreTanke = 0;
    end

    
    %Entscheidung über Angriff/Verteidigung/Tanken
    whatToDo();
    
    
    %Beschleunigung berechnen:
    bes=calculateBES();

    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Tanken oder Angreifen oder Verteidigen?
    function whatToDo()
        if StartNumberOfTank*0.5 < me.getankt || (norm(me.pos-enemy.pos)<0.2 && me.getankt>enemy.getankt)
            
            %Wenn wir mehr als die Hälfte der Tanken haben oder nahe des Gegners sind und mehr getankt haben - Angriff!
            attackEnemy();
        elseif enemy.getankt > StartNumberOfTank*0.5 || (norm(me.pos-enemy.pos)<0.2 && me.getankt<enemy.getankt)
            
            %%Erst wenn alle Tanken weg sind und wir weniger haben, als der Gegner - Fliehen!
            fleeEnemy();
        else
            %Erreicht der Gegner die anvisierte Tankstelle vor uns? dann löschen
            checkTankPath()
            %wenn Wegpunktliste leer => Pfad zur besten Tankstelle setzen
            createPathToNextTanke()
        end
    end
    


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Path To Acceleration (bes)
    function erg=calculateBES()
        if (numel(waypointList) <= 0)
            erg = -me.ges;
            return;
        end
        
        %acceleration
        dir = vecNorm(waypointList{1}-me.pos);
        corr = dir-vecNorm(me.ges);
        erg = dir + corr*5;
        
        %calculate safe breaking endvelocity
        breakingEndVel = calcBreakingEndVel();
        
        tooFast = checkIfTooFast();
        emergencyBreak = emergencyBreaking();
        
        %decelleration
        distanceToWaypoint=norm(waypointList{1}-me.pos);
        breakDistance = calcBreakDistance(norm(me.ges), breakingEndVel);
        if (breakDistance > distanceToWaypoint || tooFast)
            erg=-dir + corr*5;
        end
        
        %emergencyBreaking
        if (emergencyBreak)
            erg = -me.ges;
        end
        
        %%Überprüfen, ob Wegpunkt erreicht wurde, dann 1. Punkt löschen
        if norm(me.pos-waypointList{1}) < constWayPointReachedRadius
            waypointList(1) = [];
            debugDRAW();
        end
    end



    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %check if overshooting next waypoint
    function erg = checkIfTooFast()
        erg = false;
        
        %nothing to do
        if (numel(waypointList) <= 0)
            return;
        end
        
        vdir = vecNorm(me.ges);
        towp = waypointList{1} - me.pos;
        velocity = norm(me.ges);

        %distance to overshoot
        minTurnDist = projectVectorNorm(towp, vdir);
        %time to overshoot
        turnTime = minTurnDist/velocity;
        %position to overshoot
        turnPos = me.pos + vecNorm(vdir)*minTurnDist;
        %distace of overshooting
        correctDist = norm(turnPos - waypointList{1});
        
        accCorrection = 0.5*spiel.bes * turnTime^2 * 0.9;
        if (accCorrection < correctDist && norm(towp) > constWayPointReachedRadius*2 && velocity >= 0.01)
            erg = true;
            return;
        end
        
    end



    function erg = emergencyBreaking()
        erg = false;
        velocity = norm(me.ges);
        
        %collision check only on direct-mode
        if (numel(waypointList) == 1)
            %%check if about to collide
            safeSpaceballRadius = constSafeBorder + spiel.spaceball_radius;
            breakDist = calcBreakDistance(norm(velocity), 0)*1.3;
            checkPoint = me.pos + vecNorm(me.ges)*breakDist;
            if (~isWalkable(checkPoint, safeSpaceballRadius) && velocity >= 0.01)
                erg = true;
                return
            end
        end
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %calculate minimum velocity at next waypoint
    function erg=calcBreakingEndVel()
        erg = 0;
        
        %if this is the last waypoint - break to full stop
        if (numel(waypointList) <= 1)
            return;
        end
        
        %break to zero at last waypoint
        erg = 0;
        
        %go backwards through waypoints
        for i=fliplr(1:numel(waypointList)-1)
            
            %get minimal velocity for this waypoint
            if (i==1)
                vec1 = waypointList{i} - me.pos;
            else
                vec1 = waypointList{i} - waypointList{i-1};
            end
            vec2 = waypointList{i+1} - waypointList{i};
            length = norm(vec1);
            minVel = getMaxVelocityToAlignInTime(vec1, vec2, constCornerBreaking);
            
            tway = (sqrt(erg^2+2*spiel.bes*length)-erg)/spiel.bes;
            erg = erg + spiel.bes * tway;
            
            if (minVel < erg)
                erg = minVel;
            end
        end
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %calculate distance for breaking from vel to endvel
    function erg = calcBreakDistance(vel, endvel)
        erg = ((vel)^2 - (endvel)^2)/(2*spiel.bes);
    end
        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %setup node grid for path finding
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
                nodeGrid(x,y).isWalkable = isWalkable(worldPos, constSafeBorder + spiel.spaceball_radius);
                nodeGrid(x,y).hCost = 0;
                nodeGrid(x,y).fCost = 0;
                nodeGrid(x,y).gCost = 0;
                nodeGrid(x,y).heapIndex = 0;
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
    %check if node is a collider (mine, border)
    function erg=isWalkable(pos, radius)
        erg = true;
        secureSpaceballRadius = radius;
        
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


    function updateNodeGrid(PosOfMine, radius)
        gridSizeX = round(1/(constGridRadius*2));
        gridSizeY = round(1/(constGridRadius*2));
        
        radius = radius+0.1+constSafeBorder;
        radius = round(radius / 2 / constGridRadius);
        gridPos = worldPosToGridPos(PosOfMine);
       
        startX = clamp(gridPos(1)-radius,1,gridSizeX);
        startY = clamp(gridPos(2)-radius,1,gridSizeY);
        endX = clamp(gridPos(1)+radius,1,gridSizeX);
        endY = clamp(gridPos(2)+radius,1,gridSizeY);
        
        %create grid
        for x = startX : endX
            for y = startY : endY
                worldPos = [constGridRadius*2*x, constGridRadius*2*y];
                nodeGrid(x,y).isWalkable = isWalkable(worldPos, constSafeBorder + spiel.spaceball_radius);
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

    function erg=customSetdiff(array1, array2)
        erg=null(1);
        for i=1:numel(array1)
            containsElement = false;
            for j=1:numel(array2)
                if equalsVec(array1(i).pos, array2(j).pos)
                    containsElement = true;
                end
            end
            if containsElement == false
                erg=array1(i);
                return;
            end
        end
        if isempty(erg)
            erg=customSetdiff(array2, array1);
        end
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Pathfinder
    function waypoints = findPath(startp, endp)
        pathSuccess = false; % - Pfad gefunden
        
        startPos = getValidNodePos(worldPosToGridPos(startp));
        endPos = getValidNodePos(worldPosToGridPos(endp));
        
        %debug purposes
        if (equalsVec(startPos, endPos))
            disp('Pathfinder: stard equals end, return zero waypoints');
        end
        
        openSet = {};
        openSet = insertHeapNode(openSet, startPos);
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
            %get first heap node and resort heap
            currentNode = nodeFromGridCoords(openSet{1});
            openSet = removeHeapNode(openSet, 1);
            openSet = sortHeapNodeDown(openSet, 1);
            
            %add node to closed set
            closedSet = insertHeapNode(closedSet, currentNode.gridPos);
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
                if (~neighbour.isWalkable || containsHeapNode(closedSet, neighbour.gridPos))
                    continue;
                end
                
                %update costs for neighbours
                movementCostToNeighbour = currentNode.gCost + norm(currentNode.worldPos - neighbour.worldPos);
                if (movementCostToNeighbour < neighbour.gCost || ~containsHeapNode(openSet, neighbour.gridPos))
                    
                    nodeGrid(neighbour.gridPos(1), neighbour.gridPos(2)).gCost = movementCostToNeighbour;
                    nodeGrid(neighbour.gridPos(1), neighbour.gridPos(2)).hCost = norm(endp - neighbour.worldPos);
                    nodeGrid(neighbour.gridPos(1), neighbour.gridPos(2)).fCost = movementCostToNeighbour + norm(endp - neighbour.worldPos) + neighbour.mineCost;
                    nodeGrid(neighbour.gridPos(1), neighbour.gridPos(2)).parent = currentNode.gridPos;
                    heapIndex = neighbour.heapIndex;
                    
                    %add neighbour to openSet
                    if (~containsHeapNode(openSet, neighbour.gridPos))
                        %insert node into heap and resort heap
                        openSet = insertHeapNode(openSet, neighbour.gridPos);
                        heapIndex = numel(openSet);
                        openSet = sortHeapNodeUp(openSet, heapIndex);
                    else
                        openSet = sortHeapNodeUp(openSet, heapIndex);
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
            
            while (~equalsVec(currentNode.gridPos, startPos))
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
    %calculate nodegrid position from world position
    function erg = worldPosToGridPos(pos)
        erg = [round(pos(1)/constGridRadius/2), round(pos(2)/constGridRadius/2)];
        erg = clamp(erg, 1, round(1/(constGridRadius*2)));
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %clamps value between min and max
    function erg = clamp(value, min, max)
       for i=1:numel(value)
           if (value(i) > max)
               value(i) = max;
           end
           if (value(i) < min)
               value(i) = min;
           end
       end
       erg = value;
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
    %check if vectors are equal
    function erg = equalsVec(a, b)
        erg = false;
        if (a(1) == b(1) && a(2) == b(2))
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
        checkIndex = 1;
        pathLength = numel(path);
        
        if (pathLength <= 2)
            erg = path;
            return;
        end
        
        erg = [];
        ergInsertIndex = 1;
        
        %add first waypoint
        erg{ergInsertIndex} = path{1};
        ergInsertIndex = ergInsertIndex+1;
        
        while(checkIndex < pathLength)
            isCollided = false;
            
            %check collision tube
            for i=checkIndex+2:pathLength
                if (corridorColliding(path{checkIndex}, path{i}, constNavSecurity))
                    erg{ergInsertIndex} = path{i-1};
                    ergInsertIndex = ergInsertIndex+1;
                    checkIndex = i;
                    isCollided = true;
                    break;
                end
            end
            
            %no collision on path detected
            if (~isCollided)
                break;
            end
        end
        
        %add last waypoint
        erg{ergInsertIndex} = path{pathLength};
    end



    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %check if heap contains node
    function erg = containsHeapNode(nodes, pos)
        erg = false;
        index = nodeGrid(pos(1), pos(2)).heapIndex;
        if (index < 1 || index > numel(nodes))
            return;
        end
        
        node = nodes{index};
        
        if (node(1) == pos(1) && node(2) == pos(2))
            erg = true;
            return;
        end

    end

    %insert node into heap
    function erg = insertHeapNode(heap, nodePos)
        insertIndex = numel(heap) + 1;
        heap{insertIndex} = nodePos;
        nodeGrid(nodePos(1), nodePos(2)).heapIndex = insertIndex;
        
        erg = heap;
    end

    %remove node from heap
    %replace last node in heap with given node
    function erg = removeHeapNode(heap, index)
        
        nodePos = heap{index};
        nodeGrid(nodePos(1), nodePos(2)).heapIndex = 0;
        
        lastIndex = numel(heap);
        lastNode = heap{lastIndex};
        if (lastIndex ~= index)
            nodeGrid(lastNode(1), lastNode(2)).heapIndex = index;
        end
        
        heap{index} = lastNode;
        heap(lastIndex) = [];
        
        erg = heap;
    end

    function erg = sortHeapNodeDown(heap, index)
        erg = heap;
        %nothing to do
        if (index > numel(heap))
            return;
        end
        
        parentPos = heap{index};
        parentNode = nodeFromGridCoords(parentPos);
        
        child1 = round(index*2);
        child2 = round(index*2+1);
        
        %node has no child nodes
        if (child1 > numel(heap))
            return;
        end
        
        childPos1 = heap{child1};
        childNode1 = nodeFromGridCoords(childPos1);
        
        swapIndex = 0;
        
        %node has two child nodes
        if (child2 <= numel(heap))
            childPos2 = heap{child2};
            childNode2 = nodeFromGridCoords(childPos2);


            if (childNode1.fCost > childNode2.fCost)
                if (childNode2.fCost < parentNode.fCost)
                    swapIndex = child2;
                end
            else
                if (childNode1.fCost < parentNode.fCost)
                    swapIndex = child1;
                end
            end
        else
            %node has one child node
            if (childNode1.fCost < parentNode.fCost)
                    swapIndex = child1;
            end
        end
        
        
        if (swapIndex > 0)
            %swap nodes
            erg = swapHeapNodes(erg, swapIndex, index);
            
            %get new index and continue downsorting
            newNode = nodeFromGridCoords(parentPos);
            erg = sortHeapNodeDown(erg, newNode.heapIndex);
        end
    end

    function erg = sortHeapNodeUp(heap, index)
        erg = heap;
        parentIndex = round(index/2-0.25);
        
        if (parentIndex <= 0)
            return;
        end
        
        parentPos = heap{parentIndex};
        childPos = heap{index};
        
        
        parentNode = nodeFromGridCoords(parentPos);
        childNode = nodeFromGridCoords(childPos);
        
        if (parentNode.fCost > childNode.fCost)
            %swap position
            erg = swapHeapNodes(erg, index, parentIndex);
            
            %get new node index
            newNode = nodeFromGridCoords(childPos);
            newIndex = newNode.heapIndex;
            erg = sortHeapNodeUp(erg, newIndex);
        end
    end

    %Swap two nodes saved in heap;
    function erg = swapHeapNodes(heap, index1, index2)
        nodePos1 = heap{index1};
        nodeGrid(nodePos1(1), nodePos1(2)).heapIndex = index2;
        
        nodePos2 = heap{index2};
        nodeGrid(nodePos2(1), nodePos2(2)).heapIndex = index1;
        
        heap{index1} = nodePos2;
        heap{index2} = nodePos1;
        erg = heap;
    end



    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Normalize 2D vector
    function erg = vecNorm(vec)
        n = norm(vec);
        erg = [vec(1)/n, vec(2)/n];
        
        if (n == 0)
            erg = [0 , 0];
        end
    end



    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Append to existing cell arrray
    function erg = appendToArray(array1, array2)
        array1index = numel(array1)+1;
        erg = array1;
        
        for i=1 : numel(array2)
            erg{array1index} = array2{i};
            array1index = array1index + 1;
        end
    end
    


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Search for nearest Tanken and create Path between them
    function createPathToNextTanke()
        waypointCount = numel(waypointList);
        headedTankenList = getHeadedTanken();
        
        if numel(headedTankenList) <= 1 && spiel.n_tanke > 1 || ...
                numel(headedTankenList) == 0 && spiel.n_tanke == 1 %prevent loop on last tanke
            
            %delete other WPs if spaceball is flying not to a tanke
            if (waypointCount >= 0 && numel(headedTankenList) == 0)
                safeDeleteWaypoints();
                waypointCount = numel(waypointList);
            end
            
            %get last waypoint to append new waypoints
            lastWaypoint = me.pos;
            if (waypointCount >= 1)
                lastWaypoint = waypointList{waypointCount};
            end
            
            %set new waypoints
            tankdistance=createTankEvaluation(lastWaypoint);
            if (numel(tankdistance) <= 0)
                return;
            end
            
            disp('finding Path to next Tanke');
            
            next_tanke = tankdistance(1,1);
            waypointList = appendToArray(waypointList, findPath(lastWaypoint, spiel.tanke(next_tanke).pos));
            debugDRAW();
        end
    end

    %Create Tank Distance Table
    function erg=createTankEvaluation(position)
        erg = zeros(spiel.n_tanke, 5);
        headedTankenList = getHeadedTanken();
        for i=1:spiel.n_tanke
            
            %avoid setting WP to a tanke twice
            ignoreThisTanke = false;
            for j=1:numel(headedTankenList)
                if (headedTankenList(j) == i)
                    ignoreThisTanke = true;
                    break;
                end
            end
            
            %avoid setting tanke as new wp before collecting it
            if (norm(me.pos - spiel.tanke(i).pos) < 2*constWayPointReachedRadius)
                ignoreThisTanke = true;
            end
            
            if (ignoreThisTanke || ignoreTanke == i)
                continue;
            end
            
            
            %Create Tank Evaluation
            erg(i,1) = i;                                       %Spalte 1: Tankstellennummer
            erg(i,2) = 1/norm(spiel.tanke(i).pos-position);     %Spalte 2: Entfernung zu "position"
            erg(i,3) = norm(spiel.tanke(i).pos-enemy.pos);      %Spalte 3: Entfernung zum Gegner
            erg(i,4) = 0;
            
            %iterate through tanken
            dirToTanke = vecNorm(spiel.tanke(i).pos-position);
            for j=1:spiel.n_tanke 
                dirToNextTanke = vecNorm(spiel.tanke(j).pos-spiel.tanke(i).pos);
                if (dot(dirToTanke, dirToNextTanke) > 0.86) 
                    erg(i,4) = erg(i,4) + 1;
                end
            end
            
            
            %end evaluation
            erg(i,5) = erg(i,2) + erg(i,3)*0.3 + erg(i,4)*10;
        end
        
        %remove empty rows
        erg( ~any(erg,2), : ) = [];
        %sort rows
        erg=sortrows(erg,[-5 2 -3 1]);
    end

    %get tanken list from current waypoints
    function tankenList = getHeadedTanken()
        tankenList = zeros(5,1);
        
        %get targeted tanken
        for i=1:numel(waypointList)
            for j=1:spiel.n_tanke
                if norm(spiel.tanke(j).pos-waypointList{i}) <= spiel.tanke_radius+constGridRadius
                    insertIndex = numel(tankenList) + 1;
                    tankenList(insertIndex) = j;
                    break;
                end
            end
        end
        
        %remove duplicates
        tankenList = unique(tankenList);
        %remove zeros
        tankenList = tankenList(tankenList > 0);
    end

    %Check if target tanke is still there
    function checkTankPath()
        %avoid loop when ignoreTanke checked
        if ignoreTanke
            return;
        end
        
        %get Tanken on Waypoints
        tankenList = getHeadedTanken();
        
        %check if enemy reaches targeted tanken before us
        for i = 1:numel(tankenList)
            tankeIndex = tankenList(i);
            enemyPath = spiel.tanke(tankeIndex).pos - enemy.pos;
            ownPath = spiel.tanke(tankeIndex).pos - me.pos;
            
            %estimated time of tanken arrival
            tenemy  = norm(enemyPath)/projectVectorNorm(enemy.ges, enemyPath);
            tvenemy = getTimeToAlignVelocity(enemy.ges, enemyPath);
            
            %time to correct velocity to tanke
            town = norm(ownPath) / projectVectorNorm(me.ges, ownPath);
            tvown = getTimeToAlignVelocity(me.ges, ownPath);
            
            %check mine between enemy and tanke
            enemyColliding = corridorColliding(enemy.pos, spiel.tanke(tankeIndex).pos, spiel.spaceball_radius);
            ownColliding = corridorColliding(me.pos, spiel.tanke(tankeIndex).pos, constNavSecurity);
            
            %only if tanke is about to get taken
            if (tenemy > 0 && tenemy < 0.25 && ~enemyColliding)
                if (i==1 && norm(tenemy- town) < constCompetitionModeThreshold && ~tankeCompetition && ~ownColliding)
                    disp('competition mode activated');
                    tankeCompetition = true;
                    
                    %competition mode activated
                    accpos = getAccPos(spiel.tanke(tankeIndex).pos);
                    
                    waypointList = [];
                    waypointList{1} = spiel.tanke(tankeIndex).pos;
                    waypointList{2} = accpos;
                    debugDRAW();
                    return;
                    
                elseif (tenemy+tvenemy < town+tvown && ~tankeCompetition)
                    disp('enemy reaches tanke before us .. get new target tanke');
                    ignoreTanke = tankeIndex;
                    safeDeleteWaypoints();
                    return;
                end
            end
        end
    end



    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Angriff
    function attackEnemy()
        
        %check if path to enemy is free
        enemypos = calcEnemyHitPosition();
        if (~corridorColliding(me.pos, enemypos, constNavSecurity))
            %delete all other waypoints
            if (numel(waypointList) > 1)
                safeDeleteWaypoints();
            end
            
            %set wp directly to enemy - more precise
            endIndex = numel(waypointList);
            if (endIndex == 0)
                endIndex = 1;
            end
            
            waypointList{endIndex} = getAccPos(enemypos);
            debugDRAW();
        else
            %calculate indirect path to enemy
            recalcPath = false;
            if numel(waypointList) >= 1
                if norm(enemypos-waypointList{numel(waypointList)})>0.15
                    recalcPath = true;
                end
            else
                recalcPath = true;
            end
            
            if recalcPath
                disp('finding Path to Enemy');
                startPos = safeDeleteWaypoints();
                waypointList = appendToArray(waypointList, findPath(startPos,enemypos));

                debugDRAW();
            end
        end
    end

    function erg = calcEnemyHitPosition()
        vel = norm(me.ges-enemy.ges);
        acc = norm(me.bes-enemy.bes);
        dist = norm(me.pos-enemy.pos);
        
        thit = (sqrt(vel^2+2*acc*dist)-vel)/acc;
        
        if (thit > 1)
            erg = enemy.pos;
        else
            erg = enemy.pos + enemy.ges*thit + 0.5*enemy.bes*thit^2;
        end
    end

    %get point behind enemy so it doesn't have to decellerate before this wp
    function erg = getAccPos(pos)
        stepsize = 0.02;
        maxsize = 0.2;
        
        dir = vecNorm(pos - me.pos);
        erg = pos -dir*stepsize;
        
        length = 0;
        while(isWalkable(erg+dir*stepsize, constNavSecurity) && length < maxsize)
            erg = erg+dir*stepsize;
            length = length + stepsize;
        end
    end



    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Verteidigung
    function fleeEnemy()
        if numel(waypointList) == 0
            a=rand();
            if a < 0.5
                cornerTricking();
            elseif waitForEnemy == false
                randomFlee();
            end
        end
    end

    function randomFlee()
        disp('randomFlee');
        startPos = safeDeleteWaypoints();
        RandPoints = rand(4,2);
            for i=1:4
                RandPoints(i,3)=norm([RandPoints(i,1),RandPoints(i,2)]-enemy.pos);
                RandPoints(i,4)=0;
                if spiel.n_mine > 1
                    for j=1:spiel.n_mine
                        RandPoints(i,4)=RandPoints(i,4)+norm(spiel.mine(j).pos-[RandPoints(i,1),RandPoints(i,2)]);
                    end
                end
                RandPoints(i,5)=RandPoints(i,3)*10-RandPoints(i,4)*0.8;
            end
            RandPoints=sortrows(RandPoints,[-5 -3 4 -1 -2]);
            waypointList = appendToArray(waypointList, findPath(startPos, [RandPoints(1,1),RandPoints(1,2)]));
            debugDRAW();
    end

    function cornerTricking()
        if waitForEnemy == false
            disp('cornerTricking Pt1');
            %get nearest corner, go there and wait
            if waitForEnemy == false
                cornerNodes = [0.08,0.92,0;0.92,0.92,0;0.08,0.08,0;0.92,0.08,0];
                for i=1:4
                    cornerNodes(i,3)=norm([cornerNodes(i,1), cornerNodes(i,2)]-me.pos);
                end
                cornerNodes = sortrows(cornerNodes, [3 2 1]);
                waypointList = appendToArray(waypointList, findPath(me.pos,[cornerNodes(1,1), cornerNodes(1,2)]));
                waitForEnemy = true;
            end
        elseif waitForEnemy == true && norm(enemy.pos-me.pos) < 0.2
            disp('cornerTricking Pt2');
            cornerNodes2 = [0.08,0.92,0;0.92,0.92,0;0.08,0.08,0;0.92,0.08,0];
                for i=1:4
                    cornerNodes2(i,3)=norm([cornerNodes2(i,1), cornerNodes2(i,2)]-me.pos-enemy.ges);
                end
            cornerNodes2 = sortrows(cornerNodes2, [3 2 1]);
            pos = [cornerNodes2(2,1), cornerNodes2(2,2)];
            waypointList = appendToArray(waypointList, findPath(me.pos, pos));
            waitForEnemy = false;
        end
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %corridor colliding
    function erg = corridorColliding(startp, endp, radius)
        dir = vecNorm(endp-startp);
        erg = false;
        
            %middle line
            if (lineColliding(startp - dir*radius, endp + dir*radius, radius))
                erg = true;
                return;
            end
    end

    function erg=lineColliding(startp, endp, radius)
        erg = false;

            for i=1:spiel.n_mine
                dist = distanceLinePoint(startp, endp, spiel.mine(i).pos);

                if (dist < spiel.mine_radius+radius)
                    erg = true;
                    return;
                end
            end 
    end



    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function erg = distanceLinePoint(startp, endp, point)
        length = norm(startp - endp);
        dir = vecNorm(endp-startp);
        n = getPerpend(dir);
       
        
        y = (startp(2)*n(1)-point(2)*n(1)-n(2)*startp(1)+point(1)*n(2))/(dir(1)*n(2)-dir(2)*n(1));
        
        linePoint = startp + dir*y;
        
        %point is outside of line
        if (norm(linePoint-startp) > length || norm(linePoint-endp) > length)
            erg=Inf;
            return;
        end
        
        erg = norm(point-linePoint);
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %get perpendicular vector
    function erg = getPerpend(vec)
        erg = [-vec(2), vec(1)];
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %return projected norm of vector 1 projected on vector2
    function erg = projectVectorNorm(vec1, vec2)
        vec1 = vecNorm(vec1);
        vec2 = vecNorm(vec2);
        
        erg = norm(vec1)*dot(vec1, vec2);
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function erg = getTimeToAlignVelocity(vel1, vec)
        length = norm(vel1);
        vec = vecNorm(vec) * length;
        
        deltaV = vec - vel1;
        erg = norm(deltaV)/spiel.bes;
    end

    function erg = getMaxVelocityToAlignInTime(vec1, vec2, time)
        vec1 = vecNorm(vec1);
        vec2 = vecNorm(vec2);
        deltaVec = vec2-vec1;
        erg = time*spiel.bes/norm(deltaVec);
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function endPosition = safeDeleteWaypoints()
        %nothing to do
        endPosition = me.pos;
        if (numel(waypointList) <= 0)
            return;
        end
        
        %break distance and direction to next waypoint
        breakDistance = calcBreakDistance(norm(me.ges), 0)*0.8;
        dir = vecNorm(waypointList{1}-me.pos);
        
        %end position = full break distance
        endPosition = me.pos + dir*breakDistance;
        
        waypointList = [];
        waypointList{1} = endPosition;
    end

    


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Debugging
    function debugDRAW()
        %delete all draw handles
        for i = 1 : numel(drawHandles)
            if (~isempty(drawHandles(i)))
                delete(drawHandles(i))
            end
        end
        drawHandles = [];
        
        for i = 1 : numel(waypointList)
            drawHandles(i) = rectangle ('Parent', spiel.spielfeld_handle, 'Position', [waypointList{i}-0.0025, 0.005, 0.005], 'Curvature', [1 1], 'FaceColor', spiel.farbe.rot, 'EdgeColor', 'none');
        end
    end
end
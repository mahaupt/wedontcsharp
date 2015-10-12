function bes = beschleunigung(spiel, farbe)
    %Konstanten
    constSafeBorder = 0.005; %collision border around mines
    constGridRadius = 0.005; 
    constNavSecurity = 0.03; %simplify path
    constWayPointReachedRadius = 0.02; %0.01
    constMineProxPenality = 0.0006; %Strafpunkte für Nodes - je dichter an Mine, desto höher
    constCornerBreaking = 0.03; %je größer der Winkel zum nächsten Wegpunkt, desto höheres Bremsen. Faktor.
   
    %statische variablen definieren
    persistent nodeGrid;
    persistent waypointList;
    persistent drawHandles; %debug drawing
    persistent NumberOfMines; %Zur Bestimmung des Minenverschwindens benötigt
    persistent NumberOfTank; %Zur Entscheidung über Angriff und Tanken benötigt
    persistent ignoreTanke; %number of tanke to be ignored by targetNextTanke
    
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
        NumberOfMines = spiel.n_mine;
        NumberOfTank = spiel.n_tanke;
        setupNodeGrid();
    end
    
    %Nodegrid beim Verschwinden einer Mine aktualisieren:
    if spiel.n_mine < NumberOfMines
        disp('Updating NodeGrid');
        nodeGrid = [];
        setupNodeGrid();
        NumberOfMines = spiel.n_mine;
        waypointList = simplifyPath(waypointList);
    end

    %Entscheidung über Angriff/Verteidigung/Tanken
    whatToDo();
    
    %Beschleunigung berechnen:
    bes=calculateBES();

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Tanken oder Angreifen oder Verteidigen?
    function whatToDo()
        if NumberOfTank*0.5 < me.getankt || (norm(me.pos-enemy.pos)<0.2 && me.getankt>enemy.getankt)
            
            %Wenn wir mehr als die Hälfte der Tanken haben oder nahe des Gegners sind und mehr getankt haben - Angriff!
            attackEnemy();
        elseif (numel(spiel.tanke) <= 0 && me.getankt < enemy.getankt) || (norm(me.pos-enemy.pos)<0.2 && me.getankt<enemy.getankt)
            
            %%Erst wenn alle Tanken weg sind und wir weniger haben, als der Gegner - Fliehen!
            fleeEnemy();
        else
            %Nächste Tankstelle noch vorhanden?
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
        corr = vecNorm(waypointList{1}-me.pos)-vecNorm(me.ges);
        dir = vecNorm(waypointList{1}-me.pos);
        erg = dir + corr*5;
        
        %calculate safe breaking endvelocity
        breakingEndVel = calcBreakingEndVel();
        
        tooFast = checkIfTooFast();
        
        %decelleration
        distanceToWaypoint=norm(waypointList{1}-me.pos);
        breakDistance = calcBreakDistance(norm(me.ges), breakingEndVel);
        if (breakDistance > distanceToWaypoint)
            erg=-dir + corr*5;
        end
        
        %emergencyBreaking
        if (tooFast)
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
            disp('overshooting, breaking')
            return;
        end
        
        
        %collision check only on direct-mode
        if (numel(waypointList) == 1)
            %%check if about to collide
            safeSpaceballRadius = constSafeBorder + spiel.spaceball_radius;
            breakDist = calcBreakDistance(norm(velocity), 0)*1.1;
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
        
        %calculate vectors to next waypoint
        length = norm(waypointList{1} - me.pos);
        lastDir = vecNorm(waypointList{1} - me.pos);
        waypointIndex = 2;
        
        %return values - try to get the lowest minspeed
        globalMinSpeed = 100;
        globalBreakStartPoint = 100;
        
        %iterate waypoints within fullstop breaking range
        nullBreakDistance = calcBreakDistance(norm(me.ges), 0);
        while(length < nullBreakDistance && waypointIndex <= numel(waypointList))
            nextDist = norm(waypointList{waypointIndex} - waypointList{waypointIndex-1});
            nextDir = vecNorm(waypointList{waypointIndex} - waypointList{waypointIndex-1});
            
            %angle between waypoints 
            angle = acosd(dot(lastDir, nextDir));
            
            %calculate minimal turningspeed 
            minSpeed = 0;
            if (angle < 90 && waypointIndex < numel(waypointList))
                minSpeed = constCornerBreaking/sind(angle);
            end
            
            %get the point where i have to start breaking first
            breakStartPoint = length-calcBreakDistance(norm(me.ges), minSpeed);
            if (breakStartPoint < globalBreakStartPoint)
                globalBreakStartPoint = breakStartPoint;
                globalMinSpeed = minSpeed;
            end
            
            length = length + nextDist;
            lastDir = nextDir;
            waypointIndex = waypointIndex +1;
        end
        
        erg = globalMinSpeed;
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


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %insert node into heap
    function erg = insertHeapNode(heap, nodePos)
        insertIndex = numel(heap) + 1;
        heap{insertIndex} = nodePos;
        nodeGrid(nodePos(1), nodePos(2)).heapIndex = insertIndex;
        
        erg = heap;
    end


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %swap two nodes saved in heap;
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
    %normalize 2D vector
    function erg = vecNorm(vec)
        n = norm(vec);
        erg = [vec(1)/n, vec(2)/n];
        
        if (n == 0)
            erg = [0 , 0];
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
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%Search for nearest Tanken and create Path between them
    function createPathToNextTanke()
        waypointCount = numel(waypointList);
        
        if waypointCount <= 1 && spiel.n_tanke > 0
            if (waypointCount == 1)
                %waypointlist not empty => append new waypoints
                tankdistance=createTankEvaluation(waypointList{waypointCount});
                next_tanke = tankdistance(1,1);
                
                %possible that current tanke waypoint == next_tanke
                if (norm(spiel.tanke(next_tanke).pos-waypointList{waypointCount}) < spiel.tanke_radius+constGridRadius)
                    if (spiel.n_tanke >= 2)
                        next_tanke = tankdistance(2,1);
                    else
                        return;
                    end
                end
                
                waypointList = appendToArray(waypointList, findPath(waypointList{waypointCount}, spiel.tanke(next_tanke).pos));
            else
                %set new waypoints
                tankdistance=createTankEvaluation(me.pos);
                next_tanke = tankdistance(1,1);
                waypointList = findPath(me.pos, spiel.tanke(next_tanke).pos);
            end
            
            disp('finding Path to next Tanke');
            debugDRAW();
        end
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %create Tank Distance Table
    function erg=createTankEvaluation(position)

        erg = zeros(spiel.n_tanke,4);
        for i=1:spiel.n_tanke
            erg(i,1) = i;                                   %Spalte 1: Tankstellennummer
            erg(i,2) = norm(spiel.tanke(i).pos-position);   %Spalte 2: Entfernung zu "position"
            erg(i,3) = norm(spiel.tanke(i).pos-enemy.pos);  %Spalte 3: Entfernung zum Gegner
            a=-1;
            for j=1:spiel.n_tanke
                if i==j
                    continue
                end
                a=a+1/norm(spiel.tanke(i).pos-spiel.tanke(j).pos);
            end
            erg(i,4) = a*0.3+(1/erg(i,2))-0.3*(1/erg(i,3)); %Spalte 4: Anzahl Tankstellen in der Nähe und deren Dichte und deren Dichte zum Gegner
            
            %set evaluation bad if this is the ignore tanken
            if (ignoreTanke == i)
                erg(i,4) = -100;
            end
        end
        erg=sortrows(erg,[-4 2 -3 1]);
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %check if target tanke is still there
    function checkTankPath()
        tankenList = [];
        
        %avoid loop when only 1 tanke exists
        if numel(spiel.n_tanke) == 1 && ignoreTanke == 1
            return;
        end
        
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
        
        %waypoints needs to be recalculated
        recalculateTankenWPs = false;
        reTargetedTanken = [];
        
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
            
            %only if tanke is about to get taken
            if (tenemy > 0 && tenemy < 0.3)
                if (tenemy+tvenemy < town+tvown)
                    disp('enemy reaches tanke before us .. get new target tanke');
                    recalculateTankenWPs = true;
                    ignoreTanke = tankeIndex;
                    break;
                end
            end
            
            %if tanke can still be reached
            insertIndex = numel(reTargetedTanken) + 1;
            reTargetedTanken(insertIndex) = tankeIndex;
        end
        
        
        %recalculate tanken - Wegpunkte neu berechnen, da Tanke nicht mehr
        %angefahren werden soll
        if (recalculateTankenWPs)
            startP = safeDeleteWaypoints();
            
            for i=1:numel(reTargetedTanken)
                if (reTargetedTanken(i) == ignoreTanke)
                    continue;
                end
                
                if (i == 1)
                    startPos = startP;
                else
                    startPos = spiel.tanke(reTargetedTanken(i-1)).pos;
                end
                endPos = spiel.tanke(reTargetedTanken(i)).pos
                waypointList = appendToArray(waypointList, findPath(startPos, endPos));
            end
            
            debugDRAW();
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
            
            waypointList{endIndex} = getEnemyAccPos(enemypos);
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

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function erg = calcEnemyHitPosition()
        thit = norm(me.pos - enemy.pos)/norm(me.ges);
        if (thit > 1)
            erg = enemy.pos;
        else
            erg = enemy.pos + enemy.ges*thit + 0.5*enemy.bes*thit^2;
        end
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %get point behind enemy so it doesn't have to decellerate before this wp
    function erg = getEnemyAccPos(enemypos)
        stepsize = 0.02;
        maxsize = 0.2;
        
        dir = vecNorm(enemypos - me.pos);
        erg = enemypos -dir*stepsize;
        
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
            disp('searching for cover');
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
            RandPoints=sortrows(RandPoints,[-5 -3 4 -1 -2])
            waypointList = appendToArray(waypointList, findPath(startPos, [RandPoints(1,1),RandPoints(1,2)]));
            debugDRAW();
        end
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %corridor colliding
    function erg = corridorColliding(startp, endp, radius)
        dir = vecNorm(endp-startp);
        n = getPerpend(dir);
        erg = false;
        
        %middle line
        if (lineColliding(startp - dir*radius, endp + dir*radius))
            erg = true;
            return;
        end
        
        if (lineColliding(startp + n*radius, endp + n*radius))
            erg = true;
            return;
        end
        
        if (lineColliding(startp - n*radius, endp - n*radius))
            erg = true;
            return;
        end
        
        %start and endpoint
        %if (~isWalkable(startp, radius))
        %    erg = true;
        %    return;
        %end
        %if (~isWalkable(endp, radius))
        %    erg = true;
        %    return;
        %end
    end


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function erg=lineColliding(startp, endp)
        erg = false;
        
        for i=1:spiel.n_mine
            dist = distanceLinePoint(startp, endp, spiel.mine(i).pos);
            
            if (dist < spiel.mine_radius)
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


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function endPosition = safeDeleteWaypoints()
        %nothing to do
        endPosition = me.pos;
        if (numel(waypointList) <= 0)
            return;
        end
        
        fullStopDist = calcBreakDistance(norm(me.ges), 0);
        lastDir = waypointList{1} - me.pos;
        length = norm(lastDir);
        newWPList = [];
        
        for i=1:numel(waypointList)
            if length > fullStopDist || i == numel(waypointList)
                deltaLength = fullStopDist - length;
                dir = vecNorm(lastDir);
                lastWayPoint = waypointList{i} + dir * deltaLength;
                
                if (i == numel(waypointList))
                    lastWayPoint = waypointList{i};
                end
                
                insertIndex = numel(newWPList) + 1;
                newWPList{insertIndex} = lastWayPoint;
                break;
            end
            
            insertIndex = numel(newWPList) + 1;
            newWPList{insertIndex} = waypointList{i};
            lastDir = waypointList{i} - waypointList{i+1};
            length = length + norm(lastDir);
        end
        
        %save data
        waypointList = newWPList;
        debugDRAW();
        
        %get new end point
        endIndex = numel(waypointList);
        if (endIndex > 0)
            endPosition = waypointList{endIndex};
        end
    end
    

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%DEBUGGING%%%%%%%
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
function bes = beschleunigung(spiel, farbe)
    %Konstanten
    constSafeBorder = 0.001;
    constGridRadius = 0.01;
    
    %statische variablen definieren
    persistent nodeGrid;
    %persistent waypointList;
    
    
    %%wird einmal am Anfang ausgeführt
    if (isempty(nodeGrid))
        setupNodeGrid()
        findPath([0.5,0.5], [0.3, 0.3]);
    end
    
    bes = [0, 1];
    
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
        startNode = nodeGrid(startPos(1),startPos(2));
        endNode = nodeGrid(endPos(1),endPos(2));
        
        openSet = [startNode];
        %closedSet = [];
        closedSetIndex = 1;
        
        while(numel(openSet) > 0)
            currentNode = openSet(1);
            openSetIndex = 1;
            
            %get node woth lowest fcost (or hcost) and remove it from openlist
            for i=1:numel(openSet)    
                if (openSet(i).fCost < currentNode.fCost || openSet(i).fCost == currentNode.fCost && openSet(i).hCost > currentNode.hCost)
                    currentNode = openSet(i);    
                    openSetIndex = i;
                end
            end
            
            %remove node from open set
            openSet(openSetIndex) = [];
            %add node to closed set
            closedSet(closedSetIndex) = currentNode;
            closedSetIndex = closedSetIndex + 1;
            
            %if it is target - close - path found!
            if (equalsNode(currentNode, endNode))
                pathSuccess = true;
                break;
            end
            
            %calculate neighbour cost indices
            neighbours = getNeighbourNodes(currentNode);
            for i = 1 : numel(neighbours)
                neighbour = neighbours(i);
                if (~neighbour.isWalkable || containsNode(closedSet, currentNode))
                    continue;
                end
                
                %update costs for neighbours
                movementCostToNeighbour = currentNode.gCost + norm(currentNode.worldPos - neighbour.worldPos);
                if (movementCostToNeighbour < neighbour.gCost && ~containsNode(openSet, neighbour))
                    neighbour.gCost = movementCostToNeighbour;
                    neighbour.hCost = norm(endNode.worldPos - neighbour.worldPos);
                    neighbour.fCost = neighbour.gCost + neighbour.hCost;
                    neighbour.parent = currentNode;

                    %write update
                    nodeGrid(neighbour.gridX, neighbour.gridY) = neighbour;

                    %add neighbour to openSet
                    if (~ismember(openSet, neighbour))
                        append(openSet, neighbour);
                    end
                end
            end
        end
        
        %finished pathfinding
        if (pathSuccess)
            %retrace path
            currentNode = endNode;
            waypoints = [];
            
            while (~equalsNode(currentNode,startNode))
               append(waypoints, currentNode.worldPos);
               currentNode = currentNode.parent;
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
        i = 1;
        for x = node.gridPos(1) -1 : node.gridPos(1) +1
            for y = node.gridPos(2) -1 : node.gridPos(2) + 1
                if (equalsNode(nodeGrid(x, y), node))
                    continue;
                end
                
                erg(i) = nodeGrid(x, y);
                i = i + 1;
            end
        end
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % simplify path
    function erg = simplifyPath(path)
        erg = path;
    end


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %check if array contains node
    function erg = containsNode(nodes, node)
        erg = false;
        for i=1:numel(nodes)
            if (equalsNode(nodes(i), node))
                erg = true;
                return;
            end
        end
    end
end
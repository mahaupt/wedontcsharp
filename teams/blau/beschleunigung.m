function bes = beschleunigung(spiel, farbe)
%% Konstanten & Variablen zu Beginn des Spiels festlegen
    
    %NAVIGATION
    %sicherheitsradius um Minen und Banden
    constSafeBorder = 0.001;
    %Standardradius zum Erreichen eines Wegpunktes
    constWayPointReachedRadius = 0.02;
    %Auflösung des NodeGrids (Radius eines Nodes)
    constGridRadius = 0.003;
    %Korridorbreite für simplifyPath
    constNavSecurity = 0.02;
    %Strafpunkte für Nodes - je dichter an Mine, desto höher
    %wichtig für den Pathfinder
    constMineProxPenality = 0.00001; % 0.0006
    %0.4 je größer der Winkel zum nächsten Wegpunkt, desto höheres Bremsen. Faktor.
    constCornerBreaking = 0.65; 
    %Faktor für Seitwärtsbbeschleunigungen fürs Emergencybreaking
    constEmrBrkAccFac = 0.2; 
    %Faktor für Geschwindigkeit fürs Emergencybreaking
    constEmrBrkVelFac = 1.2; 
    %simplifyPath umgehen
    constSkipSimplifyPath = false;
    %Mine proximity radius
    constMineProxRadius = spiel.mine_radius + spiel.spaceball_radius + 1.5*constNavSecurity;
    
    %TANKEN
    %Zeitdifferenz die der Gegner schneller bei der Tanke sein darf,
    %wir es aber dennoch versuchen
    constCompetitionModeThreshold = 0.1;
    %Anzahl Ebenen für Tankpfadfindung:
    constEbenen = 5;
    
    %ATTACK
    % Gegnerinterpolationsmethode 
    % 0: s= v*t+ 0.5*a*t^2
    % 1: s= v*t
    constEnemyInterpMode = 0; 
    %wenn true, dann wird immer interpoliert (ignoriert Konstante eins weiter unten)
    constEnemyAlwaysInterpolate = true;
    %Ab welcher Nähe (Zeitlich bis Treffer) interpoliert werden darf
    constEnemyInterpolationDistance = 1; 
    %bildet den Mittelwert aus den letzten x Beschleunigungswerten des
    %Gegners - smoothed die Interpolations
    constAccInterpolationSmoothing = 10;
    %TRUE: Beschleunigungsberechnung wird überbrückt,
    %sinnvoll für präzise Manöver ohne Waypoints
    %ACHTUNG: jegliche Kollisionssicherung wird umgangen
    overrideBesCalculation = false;
    %Maximale Anzahl an Minen, bei der auf lockOnAttack geschaltet werden
    %kann wenn der Weg frei ist
    %constMaxLockonMineCount = 12;
    
    %DEBUG MODE
    %true: ermöglicht ausgabe von Text und Zeichnen von gizmos
    constDebugMode = true;
    
    
    %statische Variablen definieren
    persistent nodeGrid;
    persistent waypointList;
    persistent ArrayOfMines; %Zur Bestimmung des Minenverschwindens benötigt
    persistent StartNumberOfTank; %Zur Entscheidung über Angriff und Tanken benötigt
    persistent NumberOfTankEnemy; %Momentane Anzahl der Tanken des Gegners
    persistent currentNumberOfTank; %aktuelle Anzahl an Tanken
    persistent ignoreTanke; %number of tanke to be ignored by targetNextTanke
    persistent tankeCompetition;
    persistent waitForEnemy; %benötigt, um auf den Gegner warten zu können
    persistent dispWhatToDo;
    persistent CompetitionNotbremse;

    
    %%Farbe prüfen und zuweisen
    if strcmp (farbe, 'rot')
        me = spiel.rot;
        enemy = spiel.blau;
    else
        me = spiel.blau;
        enemy = spiel.rot;
    end
    
    
    %wird einmal am Anfang ausgeführt
    if spiel.i_t==1
        initSpaceball();
    end

%% zum debuggen (einfach nen Breakpoint bei "return" setzen)
    %if spiel.i_t==60
    %    return;
    %end
    
%% Veränderungen des Spielfeldes bemerken und dementsprechend handeln
    gameChangeHandler()

    
%% Entscheidungen fällen und Beschleunigung berechnen
    %Entscheidung über Angriff/Verteidigung/Tanken
    whatToDo();
    
    %Beschleunigung berechnen:
    calculateBES();
    
%% Was soll der Spaceball tun?
    %Tanken oder Angreifen oder Verteidigen?
    function whatToDo()
        
        thit = calculateSmoothHitTime(true);
        
        if ((numel(spiel.tanke) == 0 && me.getankt > enemy.getankt) || (thit <= 0.5 && me.getankt>enemy.getankt && ~corridorColliding(me.pos, enemy.pos, constNavSecurity))) && ~tankeCompetition
            if (dispWhatToDo ~= 1)
                dispWhatToDo = 1;
                debugDisp('whatToDo: Angriff');
            end
            
            %Wenn wir mehr als die Hälfte der Tanken haben oder nahe des Gegners sind und mehr getankt haben - Angriff!
            attackEnemy();
            
        elseif (enemy.getankt > StartNumberOfTank*0.5 || (thit <= 0.5 && me.getankt<enemy.getankt && ~corridorColliding(me.pos, enemy.pos, constNavSecurity)) && ~tankeCompetition)
            if (dispWhatToDo ~= 2)
                %vorher: tanken
                if (dispWhatToDo == 3)
                    safeDeleteWaypoints();
                    debugDRAW();
                end
                
                dispWhatToDo = 2;
                debugDisp('whatToDo: Verteidigung');
            end
            
            %%Erst wenn alle Tanken weg sind und wir weniger haben, als der Gegner - Fliehen!
            fleeEnemy();
            
        else
            
            if (dispWhatToDo ~= 3)
                dispWhatToDo = 3;
                debugDisp('whatToDo: Tanken');
            end

            %%Competition Mode aktivieren und überprüfen:
            if numel(waypointList) > 0
                checkTankPath();
            end
            
        end
    end

    %wird einmal am Start aufgerufen
    %initialisiert wichtige Variablen
    function initSpaceball()
        dispWhatToDo = -1;
        nodeGrid = [];
        waypointList = [];
        ignoreTanke = 0;
        ArrayOfMines = spiel.mine;
        StartNumberOfTank = spiel.n_tanke;
        NumberOfTankEnemy = enemy.getankt;
        currentNumberOfTank = numel(spiel.tanke);
        tankeCompetition = false;
        waitForEnemy = false;
        setupNodeGrid();
        CompetitionNotbremse = false;
        CreatePathAllTanken();
    end

    %registriert Änderungen im Spielfeld und Handelt entsprechend
    function gameChangeHandler()
        
        %Nodegrid beim Verschwinden einer Mine aktualisieren:
        if numel(spiel.mine) < numel(ArrayOfMines)
            debugDisp('beschleunigung: Updating NodeGrid');
            NumberOfMine = customSetdiff(spiel.mine, ArrayOfMines);
            updateNodeGrid(NumberOfMine.pos, spiel.mine_radius);
            resimplifyWaypoints();
            ArrayOfMines = spiel.mine;
        end

        %Tanken:
        
        if currentNumberOfTank ~= numel(spiel.tanke)
            CreatePathAllTanken();
            currentNumberOfTank = numel(spiel.tanke);
        end
        
%         %wenn der Gegner eine Tanke einsammelt, die auf unserer Wegpunktliste liegt:
%         if enemy.getankt ~= NumberOfTankEnemy && dispWhatToDo == 3
%             for i = 1:numel(waypointList)
%                 if norm(enemy.pos-waypointList{i}) < 0.05
%                     CreatePathAllTanken();
%                     break;
%                 end
%             end
%             NumberOfTankEnemy = enemy.getankt;
%         end
%         
%         %wenn die Wegpunktliste leer wird
%         if numel(waypointList) <= 2 && numel(spiel.tanke) > 2 && ~tankeCompetition && dispWhatToDo == 3
%             CreatePathAllTanken();
%         end
        
        debugDrawCircle(0, 0, 0, true);
    end


%% Beschleunigung berechnen
    function calculateBES(disableMineMode)
        persistent besCalculationMode; %0: classic - 1: circle mode
        persistent besMineID;
        
        if (overrideBesCalculation)
            return;
        end
        if (isempty(besCalculationMode))
            besCalculationMode = 0;
            besMineID = 0;
        end
        if (nargin > 0 || ((spiel.n_mine <= 0 || spiel.n_mine < besMineID) && besCalculationMode == 1))
            besCalculationMode = 0;
            besMineID = 0;
            debugDisp('calculateBES: Mine Mode disabled!');
            if (nargin > 0)
                return;
            end
        end
        
        
        %Ist kein Wegpunkt vorhanden, schnellstmöglich auf 0 abbremsen und stehen bleiben
        if (numel(waypointList) <= 0)
            bes = -me.ges;
            return;
        end
        
        %check if me and next waypoint is close to mine
        if (spiel.n_mine > 0 && besCalculationMode == 0)
            checkMineID = getNearestMineId(me.pos);
            mineID = getNearestMineId(waypointList{1});
            toMine1 = norm(spiel.mine(mineID).pos - me.pos);
            toMine2 = norm(spiel.mine(mineID).pos - waypointList{1});
            
            if (toMine1 < constMineProxRadius && toMine2 < constMineProxRadius && checkMineID == mineID)
                besCalculationMode = 1;
                besMineID = mineID;
                debugDisp('calculateBES: Mine Mode activated!');
            end
        elseif (spiel.n_mine > 0 && besCalculationMode == 1)
            toMine = norm(spiel.mine(besMineID).pos - me.pos);
            if (toMine > constMineProxRadius*1.1)
                besCalculationMode = 0;
                besMineID = 0;
                debugDisp('calculateBES: Mine Mode disabled!');
            end
        end
        
        if (besCalculationMode == 1)
            calcMineBes();
        else
            calcLineBes();
        end
    end

    %calculate bes around mines
    function calcMineBes()
        minimumMineDist = spiel.mine_radius+spiel.spaceball_radius+constSafeBorder*2;
        secureSpaceballRadius = spiel.spaceball_radius + constSafeBorder;
        
        %Distanzen und Richtungen
        mineID = getNearestMineId(me.pos);
        minePos = spiel.mine(mineID).pos;
        toMine = minePos - me.pos;
        
        %Vektor in Richtung der Kreistangente
        toGes = vecNorm(getPerpend(toMine));
        if (dot(toGes, waypointList{1}-me.pos) < 0)
            toGes = -toGes;
        end
        %Vektor im Rechten Winkel zur Gschwindigkeit, der zur Mine zeigt.
        gesToMine = vecNorm(getPerpend(me.ges));
        gesToMine = gesToMine*distanceLinePoint(me.pos-vecNorm(me.ges), me.pos+vecNorm(me.ges), minePos);
        if (dot(gesToMine, toMine) < 0)
            gesToMine = -gesToMine;
        end
        
        %startwert = mein Abstand zur Mine
        mineDriveRadius = norm(toMine);
        
        %Suche nach kleinstem Radius in Wegpunkten und mache ihn zum 
        %Orbitradius
        for i=1:numel(waypointList)
            if (i > 5)
                break;
            end
            
            dist = norm(minePos-waypointList{i});
            if (dist > constMineProxRadius)
                break;
            end
            if (dist < mineDriveRadius)
                mineDriveRadius = dist;
            end
        end
        
        %Radius darf nicht zu klein werden sonst kommt es zur kollision
        mineDriveRadius = clamp(mineDriveRadius, minimumMineDist, inf);
        
        %ragt Radius über das spielfeld hinaus?
        if (minePos(1) + mineDriveRadius > 1-secureSpaceballRadius)
            mineDriveRadius = (1-minePos(1)+spiel.mine_radius)/2;
        end
        if (minePos(2) + mineDriveRadius > 1-secureSpaceballRadius)
            mineDriveRadius = (1-minePos(2)+spiel.mine_radius)/2;
        end
        if (minePos(1) - mineDriveRadius < secureSpaceballRadius)
            mineDriveRadius = (minePos(1)+spiel.mine_radius)/2;
        end
        if (minePos(2) - mineDriveRadius < secureSpaceballRadius)
            mineDriveRadius = (minePos(2)+spiel.mine_radius)/2;
        end
        
        %maximal radial velocity
        maxVelSq = spiel.bes*mineDriveRadius;

        %velocity correction Geschwindigkeitsvektor muss den Kreis
        %Tangieren
        corr = norm(gesToMine)-mineDriveRadius;
        if (dot(me.ges, toMine) < 0)
            if (norm(toMine) < mineDriveRadius)
                corr = -corr;
            else
                corr = 1;
            end
        end
        if (norm(toMine) < minimumMineDist)
            corr = -1;
        end
        
        %berechne Zentripetalbeschleunigung und addiere darin die
        %Korrektur
        zentp = clamp(norm(me.ges)^2/mineDriveRadius + 20*corr, -spiel.bes, spiel.bes);
        forward = sqrt(spiel.bes^2-zentp^2);
        bes = zentp * vecNorm(toMine) + forward*toGes;
        
        %no velocity
        if (norm(me.ges) < 0.003)
            bes = toGes;
        end
        
        %emergencybreaking
        if (norm(me.ges)^2 > maxVelSq)% || emergencyBreaking())
           bes = -me.ges;
        end
        
        %debug drawing
        debugDrawCircle(1, minePos, mineDriveRadius);
        
        %exit circle mode
        %Springe aus diesem Beschleunigungsmodus, wenn der nächste Wegpunkt
        %außerhalb des Orbitradiusses liegt und unser Beschleunigungsvektor
        %auf den nächsten Wegpunkt zeigt
        towp = vecNorm(waypointList{1}-me.pos);
        wpdist = norm(waypointList{1} - minePos);
        if (wpdist > constMineProxRadius)
            vel1 = vecNorm(me.ges);
            vel2 = vecNorm(vecNorm(me.ges)+vecNorm(toMine)/1000);
            
            if (dot(vel1, towp) > dot(vel2, towp))
                calculateBES(true);
            end
        end
        
        %Wegpunkte einsammeln
        if norm(me.pos-waypointList{1}) < constNavSecurity*1.5
            waypointList(1) = [];
            debugDRAW();
        end
    end

    %calculate line acceleration
    function calcLineBes()
        %acceleration
        dir = vecNorm(waypointList{1}-me.pos);
        corr = dir-vecNorm(me.ges);
        corr = 300*corr * norm(me.ges);
        bes = dir + corr;
        
        %calculate safe breaking endvelocity
        breakingEndVel = calcBreakingEndVel();
        
        %decelleration
        distanceToWaypoint=norm(waypointList{1}-me.pos);
        breakDistance = calcBreakDistance(norm(me.ges), breakingEndVel);
        if (breakDistance > distanceToWaypoint || checkIfTooFast())
            bes=-dir + corr;
        end
        
        %emergencyBreaking
        if (emergencyBreaking())
            bes = -me.ges;
        end
        
        
        wpReachedDist = calcWaypointReachedRadius(breakingEndVel);
        debugDrawCircle(1, waypointList{1}, wpReachedDist);
        
        %%Überprüfen, ob Wegpunkt erreicht wurde, dann 1. Punkt löschen
        if norm(me.pos-waypointList{1}) < wpReachedDist
            waypointList(1) = [];
            if tankeCompetition
                tankeCompetition = false;
                debugDisp('competitionMode deaktivated');
                CreatePathAllTanken();
            end
            debugDRAW();
            
        else
            %%überprüfe, ob 1. Wegpunkt erreichbar ist - wenn nicht, lösche und
            %%berechne neu
            
            %Sonderfall, Spaceball selbst viel zu nah an mine:
            if (spiel.n_mine > 0 && norm(me.ges) < 0.001)
                toMineVec = spiel.mine(getNearestMineId(me.pos)).pos - me.pos;
                closeMineDist = norm(toMineVec);
                if (closeMineDist < spiel.spaceball_radius + spiel.mine_radius + constSafeBorder*2)
                    firstWp = me.pos - vecNorm(toMineVec)*constNavSecurity*1.2;
                    waypointList = appendToArray({firstWp}, waypointList);
                    debugDRAW();
                    bes = -toMineVec;
                    return;
                end
            elseif (corridorColliding(me.pos, waypointList{1}, spiel.spaceball_radius))
                %sonst
                waypointList = appendToArray(findPath(me.pos, waypointList{1}), waypointList(2:end));
                debugDRAW();
                debugDisp('calculateBES: Stuck... recalculating');
            end
            
            %sonderfall - sehr dicht an der Bande
            if (norm(me.ges) < 0.001)
               spaceballRadius = spiel.spaceball_radius + constSafeBorder;
               if (me.pos(1) > 1-spaceballRadius || me.pos(1) < spaceballRadius  || ...
                       me.pos(2) > 1-spaceballRadius || me.pos(2) < spaceballRadius)
                   %beschleunigung zur Mitte
                   bes = ([0.5, 0.5] - me.pos);
               end
            end
        end
    end

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
        
        accCorrection = 0.5*spiel.bes * turnTime^2;
        if (accCorrection < correctDist && norm(towp) > constWayPointReachedRadius*2 && velocity >= 0.01)
            erg = true;
            %debugDisp('TooFast: breaking!');
            return;
        end
        
    end

    %check if enemy is too fast 
    function erg = checkIfTooFastE ()
        
%         enemyPath = me.pos-enemy.pos;
              
        if  (norm(me.pos-enemy.pos) < ((norm(enemy.ges))^2)/(norm(enemy.bes)*2)+0.03)
            erg = true;
        else 
            erg = false;
        end
    end

    %emergency breaking
    function erg = emergencyBreaking(customv, customa)
        erg = false;
        
        %custom settings
        if nargin < 1
            customv = me.ges;
        end
        if nargin < 2
            customa = bes;
        end
        
        velocity = norm(customv);
        
        %%check if about to collide
        safeSpaceballRadius = (spiel.spaceball_radius + constSafeBorder);

        %new emergency breaking - is it better?
        breakTime = velocity / spiel.bes;
        %only get the direction changing acceleration (90° from v)
        gesPerpend = vecNorm(getPerpend(customv)); %vector 90° from v
        besPerpend = gesPerpend*projectVectorNorm(customa, gesPerpend);
        checkPoint1 = me.pos + 0.5*customv*breakTime*constEmrBrkVelFac + 0.5*besPerpend*constEmrBrkAccFac*breakTime^2;
        checkPoint2 = me.pos + 0.5*customv*breakTime*constEmrBrkVelFac; %without acceleration
        
        %check if breaking corridors are free
        %check endpoints are free (includes barriers)
        if ((~isWalkable(checkPoint1, safeSpaceballRadius) || ... 
                ~isWalkable(checkPoint2, safeSpaceballRadius) || ...
            corridorColliding(me.pos, checkPoint1, safeSpaceballRadius) || ...
            corridorColliding(me.pos, checkPoint2, safeSpaceballRadius)))

            %debugDisp('Emergency Breaking: breaking!');
            erg = true;
            return
        end
    end

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

    %calculate distance for breaking from vel to endvel
    function erg = calcBreakDistance(vel, endvel)
        erg = ((vel)^2 - (endvel)^2)/(2*spiel.bes);
    end

    function erg = calcWaypointReachedRadius(endvel)
        erg = constWayPointReachedRadius; %0.01
        if (numel(waypointList) < 2)
            return;
        end
        
        %direction to next wp
        dir1 = vecNorm(waypointList{1} - me.pos);
        dir2 = vecNorm(waypointList{2} - waypointList{1});
        
        %get time to align waypoints
        time = getTimeToAlignVelocity(endvel*dir1, dir2);
        erg = clamp(time/1.6 * norm(me.ges), constWayPointReachedRadius, 0.1);
    end
     


%% Node-Grid erstellen, bzw. updaten
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
                        mineCost = mineCost + constMineProxPenality/(norm(nodeGrid(x,y).worldPos-spiel.mine(i).pos)-spiel.mine_radius)^2;
                    end
                end
                
                nodeGrid(x,y).mineCost = mineCost;
            end
        end
    end

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



%% Pathfinder
    %Wegpunkte finden
    function waypoints = findPath(startp, endp)
        pathSuccess = false; % - Pfad gefunden
        
        startPos = getValidNodePos(worldPosToGridPos(startp));
        endPos = getValidNodePos(worldPosToGridPos(endp));
        
        %debug purposes
        if (equalsVec(startPos, endPos))
            debugDisp('findPath: stard equals end, return zero waypoints');
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
            debugDisp('findPath: invalid start or end position');
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

    %calculate nodegrid position from world position
    function erg = worldPosToGridPos(pos)
        erg = [round(pos(1)/constGridRadius/2), round(pos(2)/constGridRadius/2)];
        erg = clamp(erg, 1, round(1/(constGridRadius*2)));
    end

    %clamps value between min and max
    function erg = clamp(value, min, max)
       for i=1:numel(value)
           if (value(i) > max)
               value(i) = max;
           end
           if (value(i) < min || isnan(value(i)))
               value(i) = min;
           end
       end
       erg = value;
    end
    
    %get node-number from grid coordinates
    function erg = nodeFromGridCoords(pos)
        erg = nodeGrid(pos(1), pos(2));
    end

    %check if nodes are equal
    function erg = equalsNode(a, b)
        erg = false;
        if (a.gridPos(1) == b.gridPos(1) && a.gridPos(2) == b.gridPos(2))
            erg = true;
        end
    end

    %check if vectors are equal
    function erg = equalsVec(a, b)
        erg = false;
        if (a(1) == b(1) && a(2) == b(2))
            erg = true;
        end
    end

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



%% Den Pfad vereinfachen
    % simplify path
    function erg = simplifyPath(path)
        if (constSkipSimplifyPath)
            erg = path;
            return;
        end
        
        checkIndex = 1;
        pathLength = numel(path);
        
        if (pathLength <= 2)
            erg = path;
            return;
        end
        
        erg = [];
        ergInsertIndex = 1;
        
        %add first waypoint
        %erg{ergInsertIndex} = path{1};
        %ergInsertIndex = ergInsertIndex+1;
        
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


    %bestehende Waypoints erneut vereinfachen
    function resimplifyWaypoints()
        waypointList = appendToArray({me.pos}, waypointList);
        waypointList = simplifyPath(waypointList);
    end


%% Heap-System
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

    %sorts the heap system downwards
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

    %sorts the heap system upwards
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



%% Andere Funktionen
    %Normalize 2D vector
    function erg = vecNorm(vec)
        n = norm(vec);
        erg = [vec(1)/n, vec(2)/n];
        
        if (n == 0)
            erg = [0 , 0];
        end
    end

    %Append to existing cell arrray
    function erg = appendToArray(array1, array2)
        array1index = numel(array1)+1;
        erg = array1;
        
        for i=1 : numel(array2)
            erg{array1index} = array2{i};
            array1index = array1index + 1;
        end
    end

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

    function erg = lineColliding(startp, endp, radius)
        erg = false;

            for i=1:spiel.n_mine
                dist = distanceLinePoint(startp, endp, spiel.mine(i).pos);

                if (dist < spiel.mine_radius+radius)
                    erg = true;
                    return;
                end
            end 
    end

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

    %get perpendicular vector
    function erg = getPerpend(vec)
        erg = [-vec(2), vec(1)];
    end

    %return projected norm of vector 1 projected on vector2
    function erg = projectVectorNorm(vec1, vec2)
        vec1 = vecNorm(vec1);
        vec2 = vecNorm(vec2);
        
        erg = norm(vec1)*dot(vec1, vec2);
    end

    function erg = getTimeToAlignVelocity(vel1, vec)
        if(norm(vel1) <= 0.00001)
            erg = 0;
            return;
        end
        
        dotp = dot(vecNorm(vel1), vecNorm(vec));
        angle = acos(dotp);
        if dotp < 0
            angle = angle + pi/2;
        end
        
        deltaV = angle*norm(vel1);
        erg = deltaV/spiel.bes;
    end

    function erg = getMaxVelocityToAlignInTime(vec1, vec2, time)
        dotp = dot(vecNorm(vec1), vecNorm(vec2));
        angle = acos(dotp);
        if dotp < 0
            angle = angle + pi/2;
        end
        
        erg = time*spiel.bes/angle;
    end

    function endPosition = safeDeleteWaypoints()
        %nothing to do
        endPosition = me.pos;
        if (numel(waypointList) <= 0)
            return;
        end
        
        %break distance and direction to next waypoint
        breakDistance = calcBreakDistance(norm(me.ges), 0)*0.8;
        dir = vecNorm(me.ges);
        
        %end position = full break distance
        endPosition = me.pos + dir*breakDistance;
        
        waypointList = [];
        waypointList{1} = endPosition;
    end

    
    function erg = getNearestMineId(pos)
        if (spiel.n_mine <= 0)
            erg = 0;
            return;
        end
        
        erg = 1;
        for i=1:spiel.n_mine
            if (norm(spiel.mine(i).pos-pos) < norm(spiel.mine(erg).pos - pos))
                erg = i;
            end
        end
    end

%% Altes Tankenfindungs-System

    %Check if target tanke is still there
    function checkTankPath()
        
        %check if enemy reaches targeted tanken before us
        for i = 1:numel(spiel.tanke)
            enemyPath = spiel.tanke(i).pos - enemy.pos;
            ownPath = spiel.tanke(i).pos - me.pos;
            
            %estimated time of tanken arrival
            tenemy  = norm(enemyPath)/projectVectorNorm(enemy.ges, enemyPath);
%            tvenemy = getTimeToAlignVelocity(enemy.ges, enemyPath);
            
            %time to correct velocity to tanke
            town = norm(ownPath) / projectVectorNorm(me.ges, ownPath);
            tvown = getTimeToAlignVelocity(me.ges, ownPath);
            
            %check mine between enemy and tanke
            enemyColliding = corridorColliding(enemy.pos, spiel.tanke(i).pos, spiel.spaceball_radius);
            ownColliding = corridorColliding(me.pos, spiel.tanke(i).pos, constNavSecurity);
  
            %less then zero time - never arrive
            if (tenemy < 0)
                tenemy = inf;
            end
            if (town < 0)
                town = inf;
            end
            
            %check if ignoreTanke is still valid
%             if ignoreTanke
%                 if i == ignoreTanke
%                     if ~(tenemy < 0.25 && ~enemyColliding  && (tvenemy < 0.5 || norm(enemyPath) < 0.03))
%                         %uncheck ignoreTanke if above is false
%                         ignoreTanke = 0;
%                         debugDisp('checkTankPath: disabled ignoretanke');
%                     end
%                 end
%                 continue;
%             end
            
            
            %only if tanke is about to get taken
            if (tenemy < 0.3 && ~enemyColliding)
                if (norm(spiel.tanke(i).pos - waypointList{1}) < 0.05 && norm(tenemy- town) < constCompetitionModeThreshold && ~tankeCompetition && ~ownColliding ...
                        && tvown < 0.5)
                    debugDisp('checkTankPath: competition mode activated');
                    tankeCompetition = true;
                    
                    %competition mode activated
                    accpos = getAccPos(spiel.tanke(i).pos);
                    
                    waypointList = [];
                    waypointList{1} = spiel.tanke(i).pos;
                    waypointList{2} = accpos;
                    debugDRAW();
                    return;
                    
%                 elseif (tenemy+tvenemy < town+tvown && ~tankeCompetition && (tvenemy < 0.5 || norm(enemyPath) < 0.03))
%                     debugDisp('checkTankPath: enemy reaches tanke before us .. get new target tanke');
%                     ignoreTanke = i;
%                     CreatePathAllTanken();
%                     return;
                 end
            end
          
            %Notbremse bei zu spätem Erreichen einer Tanke
            if (tankeCompetition && i==1 && tenemy < town && ~CompetitionNotbremse && enemy.getankt > me.getankt)
                vel = norm(me.ges);
                acc = spiel.bes;
                dist = norm(ownPath);
                
                tacc = (sqrt(vel^2+2*acc*dist)-vel)/acc; %t beim beschleunigen
                tbrk = (-sqrt(vel^2-2*acc*dist)+vel)/acc; %t beim bremsen
                deltat = tbrk-tacc; %differenz
                
                %distanz die Gegner in deltat zurück legen kann
                Distance = deltat*norm(enemy.ges);
                
                if (Distance <= spiel.spaceball_radius * 2)
                    debugDisp('checkTankPath: Notbremse, Tanke wird nicht vor Gegner erreicht');
                    safeDeleteWaypoints();
                    %%WEGFINDUNG VERBESSERN!
                    if isWalkable(waypointList{1} - 0.3 * enemy.ges, spiel.spaceball_radius)
                        waypointList{1} = waypointList{1} - 0.3 * enemy.ges;
                    end
                    ignoreTanke = i;
                    CompetitionNotbremse = true;
                end
            end
        end
    end



%% NEUE Tankenfindung

    function [penalty, tList] = createTankList(pathPenalty, tankList, prevPos, prevPath, Ebenen)
        if numel(tankList) <= 0 || Ebenen == 0
            penalty = pathPenalty;
            tList = {prevPos};
            return
        end
        
        penalty = inf;
        
        for i = 1:numel(tankList)
            pen = calcTankPen(tankList(i).pos, prevPos, prevPath);
            j = setdiff(1:numel(tankList), i);
            [erg1, erg2] = createTankList(pen + pathPenalty, tankList(j), tankList(i).pos, tankList(i).pos-prevPos, Ebenen-1);
            if (erg1 < penalty)
                penalty = erg1;
                tList = erg2;
            end
        end
        
        insertIndex = numel(tList)+1;
        tList{insertIndex} = prevPos;
    end

    function penalty = calcTankPen(tankPos, prevPos, prevPath)
        distPen = norm(tankPos - prevPos);
        dirPen  = getTimeToAlignVelocity(vecNorm(tankPos-prevPos), vecNorm(prevPath));
        collPen = 0;
        enemyPen = 0;
        if corridorColliding(tankPos, prevPos, constNavSecurity);
            collPen = 1.5;
        end
        enemyPen = - (norm(enemy.pos-tankPos) + getTimeToAlignVelocity(vecNorm(enemy.ges), vecNorm(tankPos - enemy.pos)));
        penalty = distPen + dirPen / 75 + collPen + enemyPen;
    end

    function CreatePathAllTanken()
        if ~tankeCompetition
            TankList=[];
            Liste = spiel.tanke;
                if ignoreTanke ~= 0
                    Liste(ignoreTanke) = [];
                end
            CompetitionNotbremse = false;
            disp('finding our Tank-path');
            [e1, TankList] = createTankList(0, Liste, me.pos, me.ges, constEbenen);
            TankList = fliplr(TankList);
            disp('calculating Path between Tanken');
            waypointList = [];
            for i = 1:numel(TankList)-1
                waypointList = appendToArray(waypointList, findPath(TankList{i},TankList{i+1}));
            end
            debugDRAW();
        end
    end

%% Angriff
    %Angriff
    function attackEnemy()
        %lockon attack ist nur sicher, wenn sich zwischen Gegner und Mir
        %keine Mine befindet
        
        %bitte stehen lassen! ich weiß noch nicht ob die untere einfache
        %Methode, sicher ist!
%         useLockonAttack = false;
%         if (spiel.n_mine < constMaxLockonMineCount)
%             useLockonAttack = true;
%             if (spiel.n_mine > 0)
%                 % 0 - constMaxLockonMineCount mines, calculate
%                 dangerRadius = spiel.spaceball_radius + spiel.mine_radius + constSafeBorder;
%                 
%                 dirToEnemy = vecNorm(enemy.pos-me.pos);
%                 dirToAxis = [dirToEnemy(1)/norm(dirToEnemy(1)), 0]; % [-1, 0 ] or [1, 0] in enemy direction
%                 angle = acos(dot(dirToEnemy, dirToAxis));
% 
%                 %negative angle if rotated clockwise
%                 if (dirToAxis(1) < 0)
%                     if (dirToEnemy(2) < 0)
%                         angle = -angle;
%                     end
%                 else
%                      if (dirToEnemy(2) > 0)
%                         angle = -angle;
%                      end
%                 end
% 
%                 %calculate rotation matrices
%                 rotMat1 = [cos(angle), -sin(angle); sin(angle), cos(angle)]; %rotate to enemy direction
%                 ownPos = (rotMat1*me.pos')';
%                 
%                 %check every mine
%                 for i=1:spiel.n_mine
%                     minePos = (rotMat1*spiel.mine(i).pos')';
%                     % größer null - gefahr!
%                     %kleiner null - mine hinter mir!
%                     checkPos = (minePos(1)-ownPos(1))*dirToAxis(1) + dangerRadius;  
%                     if (checkPos > 0)
%                         useLockonAttack = false;
%                         break;
%                     end
%                 end
%             end
%         end

        useLockonAttack = false;
        if (~corridorColliding(me.pos, enemy.pos, spiel.mine_radius*3))
            useLockonAttack = true;
        end
        
        %select attack mode
        if (~useLockonAttack)
            directAttack();
        else
            lockonAttack();
        end
    end

    
    function directAttack()
        
        %check if path to enemy is free
        enemypos = calcEnemyHitPosition(constEnemyInterpMode, constEnemyAlwaysInterpolate);
        if (~corridorColliding(me.pos, enemypos, constNavSecurity) || norm(me.pos-enemypos) < constWayPointReachedRadius+2*constGridRadius)
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
            debugDrawCircle(2, enemypos, spiel.spaceball_radius);
        else
            pathResolution = clamp(norm(enemypos-me.pos)/2, 0.1, 0.5);
            
            %Prüfe, ob Pfad neu berechnet werden soll (Gegner liegt
            %außerhalb von pathResolution vom letzten Wegpunkt)
            recalcPath = false;
            if numel(waypointList) >= 1
                if norm(enemypos-waypointList{numel(waypointList)}) > pathResolution
                    recalcPath = true;
                end
            else
                recalcPath = true;
            end
            
            %calculate indirect path to enemy
            if recalcPath
                debugDisp('directAttack: finding Path to Enemy');
                startPos = safeDeleteWaypoints();
                waypointList = appendToArray(waypointList, findPath(startPos,enemypos));

                debugDRAW();
                debugDrawCircle(2, enemypos, spiel.spaceball_radius);
            end
        end
    end


    function lockonAttack()
        persistent lockAnnouncement;
        persistent transformationAngle;
        if (isempty(lockAnnouncement))
            lockAnnouncement = 0;
        end
        
        %delete waypointlist
        if (numel(waypointList) > 1)
            waypointList = {};
            debugDRAW();
        end
        
        %rotate coordinates
        dirToEnemy = vecNorm(enemy.pos-me.pos);
        dirToAxis = [dirToEnemy(1)/norm(dirToEnemy(1)), 0]; % [-1, 0 ] or [1, 0] in enemy direction
        angle = acos(dot(dirToEnemy, dirToAxis));
        
        %negative angle if rotated clockwise
        if (dirToAxis(1) < 0)
            if (dirToEnemy(2) < 0)
                angle = -angle;
            end
        else
             if (dirToEnemy(2) > 0)
                angle = -angle;
             end
        end
        
        %keep angle when locked
        if (lockAnnouncement == 1)
            angle = transformationAngle;
        end
        
        
        
        %calculate rotation matrices
        rotMat1 = [cos(angle), -sin(angle); sin(angle), cos(angle)]; %rotate to enemy direction
        rotMat2 = [cos(angle), sin(angle); -sin(angle), cos(angle)]; %rotate back
        
        %calculate rotated positions
        rotMePos = (rotMat1*me.pos')';
        rotMeGes = (rotMat1*me.ges')';
        
        rotEnemyPos = (rotMat1*enemy.pos')';
        rotEnemyGes = (rotMat1*enemy.ges')';
        rotEnemyBes = (rotMat1*enemy.bes')';

        
        %set waypoint pos to enemy
        toEnemy = vecNorm(rotEnemyPos - rotMePos);
        
        %position aligned - finetune position -> lock onto target
        if (norm(rotMeGes(2)-rotEnemyGes(2)) < 0.0005 && norm(rotMePos(2)-rotEnemyPos(2)) < spiel.spaceball_radius*0.8)
            if (lockAnnouncement ~= 1)
                debugDisp('LockOnAttack: Target Locked!');
                lockAnnouncement = 1;
                transformationAngle = angle;
            end
        elseif (lockAnnouncement ~= 0)
            if (norm(rotMeGes(2)-rotEnemyGes(2)) > 0.002 && norm(rotMePos(2)-rotEnemyPos(2)) > spiel.spaceball_radius*1.2)
                debugDisp('LockOnAttack: WARNING! Lock failed! Realigning...');
                lockAnnouncement = 0;
                transformationAngle = 0;
            end
        end

        %copy enemy acceleration
        ax2comp = rotEnemyBes(2);

        %y velocity and position correction
        deltaA = ((rotEnemyGes(2)-rotMeGes(2)) + 0.1*(rotEnemyPos(2)-rotMePos(2)))/spiel.dt;
        ax2comp = clamp(ax2comp+deltaA, -spiel.bes, spiel.bes);

        %debug
        %str1 = sprintf('LockOnAttack: deltaV: %d   deltaS: %d', norm(enemy.ges(ax2)-me.ges(ax2)), norm(enemy.pos(ax2)-me.pos(ax2)));
        %debugDisp(str1);

        %valid x component to enemy
        ax1comp = sqrt(spiel.bes^2 - ax2comp^2) * toEnemy(1)/norm(toEnemy(1));

        %set to override acceleration calculation and set acceleration
        %manually
        overrideBesCalculation = true;
        bes = (rotMat2*[ax1comp, ax2comp]')';
        
        %debug
        drawradius = norm(rotEnemyGes(2)-rotMeGes(2))*20 + norm(rotEnemyPos(2)-rotMePos(2));
        debugDrawCircle(2, enemy.pos, clamp(drawradius, 0.02 - 0.01*lockAnnouncement, 0.2));
        
        
         %minimal to enemy velocity
        if (lockAnnouncement == 0 && norm(toEnemy(1)/rotMeGes(1)) > 10)
            bes = dirToEnemy;
        end
     
        % emergencybreaking
        if (lockAnnouncement == 0)
            if (emergencyBreaking())
                bes = -me.ges;
                
                %stuck at the wall
                if (norm(me.ges) < 0.001)
                    bes = dirToEnemy;
                end
            end
        elseif (lockAnnouncement == 1)
            %Geschwindigkeitskomponente die vom Gegner weg zeigt
            cvx = vecNorm(toEnemy) * clamp(projectVectorNorm(rotMeGes, -toEnemy), 0, Inf);
            
            customv = rotMat2*[cvx(1); rotMeGes(2)];
            customa = rotMat2*[0; ax2comp];
            if (emergencyBreaking(customv', customa'))
                bes = -me.ges;
            end
        end
    end

    function erg = calcEnemyHitPosition(interpolationMode, alwaysInterpolate)
        % SMOOTH ACCELERATION VALUES
        persistent lastInterpEnemyPos;
        
        if (isempty(lastInterpEnemyPos))
            lastInterpEnemyPos = [0, 0];
        end
        
        
        %always interpolate
        if (nargin <= 2)
            alwaysInterpolate = false;
        end
        if (nargin <= 1)
            interpolationMode = 0;
        end
        
        %calculate hit time
        thit = calculateSmoothHitTime(interpolationMode==0);
        ergs = getSmoothedAccelerationValues();
        enemyacc = ergs(2);
        
        if (interpolationMode == 1)
            enemyacc = 0;
        end
        
        %vorher : (thit > 1) neu : (dist > 0.2)
        if (thit > constEnemyInterpolationDistance && ~alwaysInterpolate)
            erg = enemy.pos;
            lastInterpEnemyPos = erg;
        else
            %interpolate
            erg = enemy.pos + enemy.ges*thit + 0.5*enemyacc*thit^2;
            
            %clamping erg
            safeSpaceballRadius = spiel.spaceball_radius + constSafeBorder;
            erg = clamp(erg, safeSpaceballRadius, 1-safeSpaceballRadius);
            
            %point is not walkable -> set own point
            if (~isWalkable(erg, spiel.spaceball_radius + constSafeBorder))
                erg = lastInterpEnemyPos;
            else
                lastInterpEnemyPos = erg;
            end
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

    function [meacc, enemyacc] = getSmoothedAccelerationValues()
        % SMOOTH ACCELERATION VALUES
        persistent enemyAccSmooth;
        persistent meAccSmooth;
        persistent lastTimeCalculated;
        
        %set vars on startup
        if (isempty(enemyAccSmooth))
            enemyAccSmooth = [0, 0];
            meAccSmooth = [0, 0];
            lastTimeCalculated = -1;
        end
        
        %if values are not created yet, calculate
        if (lastTimeCalculated ~= spiel.i_t)
            enemyAccSmooth = enemyAccSmooth*(constAccInterpolationSmoothing-1) + enemy.bes;
            enemyAccSmooth = enemyAccSmooth/constAccInterpolationSmoothing;
            meAccSmooth = meAccSmooth*(constAccInterpolationSmoothing-1) + me.bes;
            meAccSmooth = meAccSmooth/constAccInterpolationSmoothing;
            lastTimeCalculated = spiel.i_t;
        end
        
        %output values
        meacc = meAccSmooth;
        enemyacc = enemyAccSmooth;
    end

    function time=calculateSmoothHitTime(includeAcceleration)
        persistent lastTimeCalculated;
        persistent lastCalculatedValue;
        
        %set vars on startup
        if (isempty(lastTimeCalculated))
            lastTimeCalculated = -1;
            lastCalculatedValue = 0;
        end
        
        %cet smoothed acceleration values
        enemyAccSmooth = 0;
        meAccSmooth = 0;
        if (includeAcceleration)
            [enemyAccSmooth, meAccSmooth] = getSmoothedAccelerationValues();
        end
        
        %calculate time only if necessary
        if (lastTimeCalculated ~= spiel.i_t)
            a = norm(enemyAccSmooth-meAccSmooth);
            v = norm(me.ges - enemy.ges);
            s = norm(me.pos - enemy.pos);

            if (a > 0.0001)
                lastCalculatedValue = (sqrt(v^2+2*a*s)-v)/a;
            else
                lastCalculatedValue = s/v;
            end
            lastTimeCalculated = spiel.i_t;
        end
        
        time = lastCalculatedValue;
    end



%% Verteidigung
    %Verteidigung
    function fleeEnemy()
        if numel(waypointList) == 0
            cornerTricking();
        end
    end

    function cornerTricking()
        
        %define a Matrix that contains all corner positions
        cornerNodes = [0.01,0.99,0;0.99,0.99,0;0.01,0.01,0;0.99,0.01,0];
        if waitForEnemy == false
            debugDisp('cornerTricking: Pt1');
%             get nearest corner, go there and wait
            if waitForEnemy == false
                for i=1:4
                    cornerNodes(i,3)=norm(cornerNodes(i,1:2)-me.pos);
                end
                nearestCorner = sortrows(cornerNodes, [3 2 1]);
                waypointList = appendToArray(waypointList, findPath(me.pos,nearestCorner(1,1:2)));
                waitForEnemy = true;
            end
        %waiting for the enemy
        elseif waitForEnemy == true
            %calculate vector between us and enemy
            %enemyPath = me.pos-enemy.pos;
            %the time, enemy needs to get to our position
            %tenemy  = norm(enemyPath)/projectVectorNorm(enemy.ges, enemyPath);
            %check if there is a mine on the enemy's path towards us
            %enemyColliding = corridorColliding(enemy.pos, me.pos, spiel.spaceball_radius);

            
            if checkIfTooFastE () == true %|| tenemy < 0.0001
                debugDisp('cornerTricking: Pt2');
                    %sort all corners based on the direction the enemy is coming from and their distance to us
                    for i=1:4
                        cornerNodes(i,3)=norm(cornerNodes(i,1:2)-me.pos-enemy.ges);
                    end
                nextCorner = sortrows(cornerNodes, [3 2 1]);
                %go to the second corner, since the first one is on our current position
                waypointList = appendToArray(waypointList, findPath(me.pos, nextCorner(2,1:2)));
                waitForEnemy = false;
            end
        end
    end



%% Debugging
    %Wegpunkte einzeichnen
    function debugDRAW()
        persistent drawHandles;
        
        if (~constDebugMode)
            return;
        end
        
        %delete all draw handles
        for i = 1 : numel(drawHandles)
            if (ishandle(drawHandles(i)))
                delete(drawHandles(i))
            end
        end
        drawHandles = [];
        
        %get color
        dcolor = spiel.farbe.blau;
        if strcmp (farbe, 'rot')
            dcolor = spiel.farbe.rot;
        end
        
        for i = 1 : numel(waypointList)
            drawHandles(i) = rectangle ('Parent', spiel.spielfeld_handle, 'Position', [waypointList{i}-0.0025, 0.005, 0.005], 'Curvature', [1 1], 'FaceColor', dcolor, 'EdgeColor', [0, 0, 0]);
        end
    end


    function debugDrawCircle(index, pos, rad, clearall)
        persistent mineDraw;
        
        %skip
        if (~constDebugMode)
            return;
        end
        
        %delete all
        if (nargin > 3)
            for i=1:numel(mineDraw)
                if ishandle(mineDraw(i))
                    delete(mineDraw(i));
                end
            end
            return;
        end
        
        %get color
        dcolor = spiel.farbe.blau;
        if strcmp (farbe, 'rot')
            dcolor = spiel.farbe.rot;
        end
        
        %empty prev drawing
        if (index <= numel(mineDraw))
            if ishandle(mineDraw(index))
                delete(mineDraw(index));
            end
        end
        
        if (rad > 0)
            mineDraw(index) = rectangle ('Parent', spiel.spielfeld_handle, 'Position', [pos-rad, rad*2, rad*2], 'Curvature', [1 1], 'FaceColor', 'none', 'EdgeColor', dcolor);
        end
    end

    function debugDisp(str)
        if (~constDebugMode)
            return;
        end
        
        disp(str);
    end
end

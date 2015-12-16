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
    %0.4 je größer der Winkel zum nächsten Wegpunkt, desto höheres Bremsen. Faktor.
    constCornerBreaking = 0.50;
    %Faktor für Seitwärtsbbeschleunigungen fürs Emergencybreaking
    constEmrBrkAccFac = 0.2; 
    %Faktor für Geschwindigkeit fürs Emergencybreaking
    constEmrBrkVelFac = 1.2;
    %simplifyPath umgehen
    constSkipSimplifyPath = false;
    %Mine proximity radius
    constMineProxRadius = spiel.mine_radius + spiel.spaceball_radius + 1.7*constNavSecurity;
    
    %WHATTODO
    constDecisionOfAttackAndDefense = 0.5;
    
    %ATTACK
    % Gegnerinterpolationsmethode 
    % 0: s= v*t+ 0.5*a*t^2
    % 1: s= v*t
    constEnemyInterpMode = 0; 
    %bildet den Mittelwert aus den letzten x Beschleunigungswerten des
    %Gegners - smoothed die Interpolations
    constAccInterpolationSmoothing = 10;
    %TRUE: Beschleunigungsberechnung wird überbrückt,
    %sinnvoll für präzise Manöver ohne Waypoints
    %ACHTUNG: jegliche Kollisionssicherung wird umgangen
    overrideBesCalculation = false;
    
    %REFUEL
    constIgnoreTankeTime = 1.5; %wie lange darf der Gegner zu seiner nächsten Tanke benötigen, damit wir sie ignorieren
    
    %DEBUG MODE
    %true: ermöglicht ausgabe von Text und Zeichnen von gizmos
    constDebugMode = true;
    
    %COMPILING
    %true = force compiling, false = not compiling
    constCompiling = true;
    
    %statische Variablen definieren
    persistent waypointList; %Liste mit Wegpunkten. Werden automatisch nacheinander abgefahren
    persistent ArrayOfMines; %Zur Bestimmung des Minenverschwindens benötigt
    persistent StartNumberOfTank; %Anzahl der Tanstellen zu Spielbeginn
    persistent currentNumberOfTank; %aktuelle Anzahl an Tanken
    persistent lastNumberOfMeTanke;
    persistent TankList; %Liste mit Index-Nummern der Tanken, die wir anfahren.
    persistent tankeCompetition; %ist CompetitionMode aktiviert?
    persistent cancelCompetition; %CompetitionMode wird gerade abgebrochen
    persistent ignoreTanke; %diese Tanke ignorieren!
    persistent waitForEnemy; %benötigt, um auf den Gegner warten zu können
    persistent dispWhatToDo; %Was tun wir gerade? 1=Angriff, 2=Verteidigung, 3=Tanken
    persistent mexHandle; %handle of mex functions
    persistent Verteidigung;
    persistent firstTankePositionPersistent;
    
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
    
    
%% Veränderungen des Spielfeldes bemerken und dementsprechend handeln
    gameChangeHandler()


%% Entscheidungen fÃ¤llen und Beschleunigung berechnen
    %Entscheidung Ã¼ber Angriff/Verteidigung/Tanken
    whatToDo();
    
    %Beschleunigung berechnen:
    calculateBES();
    
%% Letzte Sachen machen
    lastNumberOfMeTanke = me.getankt;
    
    
%% Was soll der Spaceball tun?
    %Tanken oder Angreifen oder Verteidigen?
    function whatToDo()
        
        thit = calculateSmoothHitTime(true);
        
        if ((spiel.n_tanke == 0 && me.getankt > enemy.getankt) || (thit <= constDecisionOfAttackAndDefense && me.getankt>enemy.getankt && ~corridorColliding(me.pos, enemy.pos, constNavSecurity))) && ~tankeCompetition
            if (dispWhatToDo ~= 1)
                dispWhatToDo = 1;
                debugDRAW();
                debugDisp('whatToDo: ATTACK');
            end
            
            %Wenn wir mehr als die Hälfte der Tanken haben oder nahe des Gegners sind und mehr getankt haben - Angriff!
            attackEnemy();
            
        elseif (enemy.getankt > StartNumberOfTank*0.5 ...
                || (thit <= constDecisionOfAttackAndDefense - constDecisionOfAttackAndDefense/4 && me.getankt<enemy.getankt && ~corridorColliding(me.pos, enemy.pos, constNavSecurity)) && ~tankeCompetition) ...
                || (spiel.n_tanke == 1 && me.getankt <= enemy.getankt && ignoreTanke ~= 0)
            if (dispWhatToDo ~= 2)
                %vorher: tanken
                if (dispWhatToDo == 3)
                    safeDeleteWaypoints();
                    debugDRAW();
                end
                
                dispWhatToDo = 2;
                debugDisp('whatToDo: DEFENCE');
            end
            
            %%Erst wenn alle Tanken weg sind und wir weniger haben, als der Gegner - Fliehen!
            
            fleeEnemy();
           
        else
            
            if (dispWhatToDo ~= 3)
                dispWhatToDo = 3;
                debugDisp('whatToDo: REFUEL');
                CreatePathAllTanken();
            end
            doesEnemyGetTanke();
        end
    end

    %wird einmal am Start aufgerufen
    %initialisiert wichtige Variablen
    function initSpaceball()
        dispWhatToDo = -1;
        waypointList = [];
        ArrayOfMines = spiel.mine;
        StartNumberOfTank = spiel.n_tanke;
        currentNumberOfTank = spiel.n_tanke;
        tankeCompetition = false;
        cancelCompetition = false;
        ignoreTanke = 0;
        waitForEnemy = false;
        Verteidigung = false;
        lastNumberOfMeTanke = 0;
        firstTankePositionPersistent = [0, 0];
        
        %compile mex files
        if strcmp (farbe, 'rot')
            cd teams/rot
        else
            cd teams/blau
        end
        
        if (constCompiling)
            mex source/esc_find_path.cpp
            mex source/esc_find_tanke.cpp
        end
        
        %%clear static variables
        clear esc_find_path
        clear esc_find_tanke
        
        %set handles
        mexHandle.esc_find_path = @esc_find_path;
        mexHandle.esc_find_tanke = @esc_find_tanke;

        cd ../../
    end

    %registriert Änderungen im Spielfeld und handelt entsprechend
    function gameChangeHandler()
        
        %Nodegrid beim Verschwinden einer Mine aktualisieren:
        if spiel.n_mine < numel(ArrayOfMines)
            resimplifyWaypoints();
            ArrayOfMines = spiel.mine;
        end

        %TankListe beim Verschwinden einer Tanke aktualisieren:
        
        if currentNumberOfTank ~= spiel.n_tanke && dispWhatToDo == 3
            ignoreTanke = 0;
            CreatePathAllTanken();
            currentNumberOfTank = spiel.n_tanke;
        end
        
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
            
            if (toMine1 < constMineProxRadius && toMine2 < constMineProxRadius && checkMineID == mineID && ~Verteidigung ) 
                besCalculationMode = 1;
                besMineID = mineID;
            end
        elseif (spiel.n_mine > 0 && besCalculationMode == 1)
            toMine = norm(spiel.mine(besMineID).pos - me.pos);
            if (toMine > constMineProxRadius*1.1)
                besCalculationMode = 0;
                besMineID = 0;
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
        
        %%enlarge maximal velocity if angle is small
        %get output wp
        outwaypt = [0, 0];
        for i=1:numel(waypointList)
            if (norm(minePos-waypointList{i}) > constMineProxRadius)
                outwaypt = waypointList{i};
                break;
            end
        end
        if (~isequal(outwaypt, [0,0]))
            if (dot(vecNorm(waypointList{i}-minePos), vecNorm(me.ges)) > 0.9 && i < 3)
                maxVelSq = maxVelSq * 1.5;
            end
        end

        %velocity correction Geschwindigkeitsvektor muss den Kreis
        %Tangieren
        corr = (norm(gesToMine)-mineDriveRadius)/constSafeBorder;
        if (dot(me.ges, toMine) < 0)
            if (norm(toMine) < mineDriveRadius)
                corr = -corr;
            else
                corr = 1;
            end
        end
        if (norm(toMine) < minimumMineDist)
            corr = (norm(toMine)-mineDriveRadius)/constSafeBorder;
        end
        
        
        %kreisgeschwindigkeit
        circvel = norm(projectVectorNorm(me.ges, toGes));
        
        %berechne Zentripetalbeschleunigung und addiere darin die
        %Korrektur
        zentp = clamp(circvel^2/norm(mineDriveRadius)*(1+corr) + corr*0.1, -spiel.bes, spiel.bes);
        
        %Vorwärtsbeschleunigung
        forward = sqrt(spiel.bes^2-zentp^2);
        
        
        %break before last waypoint or if no wps exist
        if (numel(waypointList) == 1 && norm(waypointList{1}-minePos) <= constMineProxRadius)
            pathToWpRad = real(acos(clamp(dot(vecNorm(-toMine), vecNorm(waypointList{1}-minePos)), -1, 1))) * mineDriveRadius;
            breakDist = calcBreakDistance(norm(me.ges), 0);
            if (breakDist > pathToWpRad && dot(me.ges, toGes) > 0)
                forward = -(forward); % + norm(zentp)/2
            end
        end
        
        %setuo beschleunigung
        bes = zentp * vecNorm(toMine) + forward*toGes;
        
        %no velocity
        if (norm(me.ges) < 0.01)
            bes = toGes;
        end
        
        %emergencybreaking
        if (norm(me.ges)^2 > maxVelSq)% || emergencyBreaking())
           bes = vecNorm(bes)-vecNorm(me.ges)*1.5;
        elseif (dot(me.ges, toGes) < 0)
           bes = vecNorm(bes)+vecNorm(toGes);
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
        if (breakDistance > distanceToWaypoint && breakingEndVel < norm(me.ges) || checkIfTooFast())
            bes=-dir + corr;
        end
        
        %emergencyBreaking
        if (emergencyBreaking()) % && ~Verteidigung
            bes = -me.ges;
        end
        
        
        wpReachedDist = calcWaypointReachedRadius(breakingEndVel);
        debugDrawCircle(1, waypointList{1}, wpReachedDist);
        
        %%Überprüfen, ob Wegpunkt erreicht wurde, dann 1. Punkt löschen
        if norm(me.pos-waypointList{1}) < wpReachedDist
            waypointList(1) = [];
            if tankeCompetition
                tankeCompetition = false;
                debugDisp('Tanken: compMode deactivated');
                waypointList = [];
                CreatePathAllTanken();
                debugDRAW();
            end
            debugDRAW();
            
        else
            %%überprüfe, ob 1. Wegpunkt erreichbar ist - wenn nicht, lösche und
            %%berechne neu
            
            %Sonderfall, Spaceball selbst viel zu nah an mine:
            if (spiel.n_mine > 0 && norm(me.ges) < 0.002)
                toMineVec = spiel.mine(getNearestMineId(me.pos)).pos - me.pos;
                closeMineDist = norm(toMineVec);
                if (closeMineDist < spiel.spaceball_radius + spiel.mine_radius + constSafeBorder*2)
                    firstWp = me.pos - vecNorm(toMineVec)*constNavSecurity*1.2;
                    waypointList = appendToArray({firstWp}, waypointList);
                    debugDRAW();
                    bes = -toMineVec;
                    return;
                end
            elseif (corridorColliding(me.pos, waypointList{1}, spiel.spaceball_radius) && dispWhatToDo ~= 3 && ~Verteidigung)
                %sonst
                debugDisp('calculateBES: Stuck - recalculating');
                waypointList = appendToArray(findPath(me.pos, waypointList{1}), waypointList(2:end));
                debugDRAW();
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
        
        timetoarrive = norm(waypointList{1}-me.pos)/norm(me.ges);
        timetoalign = getTimeToAlignVelocity(me.ges, waypointList{1}-me.pos);
        
        if (timetoalign > timetoarrive && norm(waypointList{1}-me.pos) > constWayPointReachedRadius*2 && norm(me.ges) >= 0.01)
            erg = true;
            return;
        end
        
    end

    %check if enemy is too fast 
    function erg = checkIfTooFastE ()
        enemyPath = me.pos-enemy.pos;
        if  (norm(enemyPath)) < (((norm(enemy.ges))^2)/(norm(enemy.bes)*2)+0.03);
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
            customa = vecNorm(bes)*spiel.bes;
        end
        
        velocity = norm(customv);
        
        %%check if about to collide
        safeSpaceballRadius = (spiel.spaceball_radius + constSafeBorder);

        %new emergency breaking - is it better?
        breakTime = velocity / spiel.bes;
        %only get the direction changing acceleration (90Â° from v)
        gesPerpend = vecNorm(getPerpend(customv)); %vector 90Â° from v
        besPerpend = gesPerpend*projectVectorNorm(customa, gesPerpend);
        checkPoint1 = me.pos + 0.5*customv*breakTime*constEmrBrkVelFac + 0.5*besPerpend*constEmrBrkAccFac*breakTime^2;
        checkPoint2 = me.pos + 0.5*customv*breakTime*constEmrBrkVelFac; %without acceleration
        
        %check if breaking corridors are free
        %check endpoints are free (includes barriers)
        if ((~isWalkable(checkPoint1, safeSpaceballRadius) || ... 
                ~isWalkable(checkPoint2, safeSpaceballRadius) || ...
            corridorColliding(me.pos, checkPoint1, safeSpaceballRadius) || ...
            corridorColliding(me.pos, checkPoint2, safeSpaceballRadius)))

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
            
            tway = sqrt(erg^2+2*spiel.bes*length)-erg;
            erg = erg + tway;
            
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
     

%% Pathfinder
    %Wegpunkte finden
    function waypoints = findPath(startp, endp)
        waypoints = simplifyPath(mexHandle.esc_find_path(spiel.mine, startp, endp));
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
        erg = norm(vec1)*dot(vecNorm(vec1), vecNorm(vec2));
    end

    function erg = getTimeToAlignVelocity(vel1, vec)
        if(norm(vel1) <= 0.00001)
            erg = 0;
            return;
        end
        if(norm(vec) <= 0.00001)
            erg = 0;
            return;
        end
        
        dotp = real(clamp(dot(vecNorm(vel1), vecNorm(vec)), -1, 1));
        angle = real(acos(dotp));
        if dotp < 0
            angle = angle + pi/2;
        end
        
        deltaV = angle*norm(vel1);
        erg = deltaV/spiel.bes;
    end

    function erg = getMaxVelocityToAlignInTime(vec1, vec2, time)
        dotp = dot(vecNorm(vec1), vecNorm(vec2));
        angle = real(acos(dotp));
        if dotp < 0
            angle = angle + pi/2;
        end
        
        erg = time*spiel.bes/angle;
        
        if (angle == 0)
            erg = inf;
        end
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
        for i=1:spiel.n_mine
            mineList(i).dist = norm(pos - spiel.mine(i).pos);
        end
        [minValue,index] = min([mineList.dist]);
        erg=index;
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


%% Tankenfindung

    function CreatePathAllTanken()
        waitForEnemy = false;
        if ~tankeCompetition && ~cancelCompetition
            ebenen = round(StartNumberOfTank/2)-me.getankt;
            if me.getankt >= round(StartNumberOfTank/2)
                ebenen = spiel.n_tanke;
            end
            TankList = mexHandle.esc_find_tanke(spiel.mine, spiel.tanke, me.pos, me.ges, enemy.pos, enemy.ges, ignoreTanke, ebenen);
            
            TankList = fliplr(TankList);
            if (numel(TankList) > 0)
                
                %check if keep first tanke
                keepFirstTanke = false;
                if (~isequal(firstTankePositionPersistent, [0, 0]) && me.getankt == lastNumberOfMeTanke)
                    if (~isequal(firstTankePositionPersistent, spiel.tanke(TankList{1}).pos))
                        keepFirstTanke = true;
                        if (ignoreTanke ~= 0)
                            if (isequal(firstTankePositionPersistent, spiel.tanke(ignoreTanke).pos))
                                keepFirstTanke = false;
                            end
                        end
                    end
                end
                
                
                if (keepFirstTanke)
                    debugDisp('Tanken: calculating Path - keeping first Tanke');
                    waypointList=findPath(me.pos,firstTankePositionPersistent);
                    waypointList = appendToArray(waypointList, findPath(firstTankePositionPersistent,spiel.tanke(TankList{1}).pos));
                else
                    debugDisp('Tanken: calculating Path');
                    waypointList = findPath(me.pos,spiel.tanke(TankList{1}).pos);
                    firstTankePositionPersistent = spiel.tanke(TankList{1}).pos;
                end
                for i = 1:numel(TankList)-1
                    waypointList = appendToArray(waypointList, findPath(spiel.tanke(TankList{i}).pos,spiel.tanke(TankList{i+1}).pos));
                end
                debugDRAW();
            end
        end
    end

    function doesEnemyGetTanke()
        %ignoreTanke setzen:
        if spiel.n_tanke >= 1
            
            %Zeit berechnen, die der Gegner zu allen Tanken benötigt
            for i=1:spiel.n_tanke
                enemyPath = spiel.tanke(i).pos - enemy.pos;
                EnemyTimeToTankList(i) = norm(enemyPath) / projectVectorNorm(enemy.ges, enemyPath);
                %liegt die Tanke hinter dem Gegner (Zeit < 0) - auf inf setzen
                if EnemyTimeToTankList(i) < 0
                    EnemyTimeToTankList(i) = inf;
                end
            end
            %Wie lange braucht der Gegner zu seiner dichtesten Tanke und welche ist das?
            [EnemyTimeToClosestTanke,ClosestEnemyTanke] = min(EnemyTimeToTankList);
            
            %liegt noch eine Mine zwischen Gegner / uns und Tanke?
            enemyColliding = corridorColliding(enemy.pos, spiel.tanke(ClosestEnemyTanke).pos, spiel.spaceball_radius);
            ownColliding = corridorColliding(me.pos, spiel.tanke(ClosestEnemyTanke).pos, spiel.spaceball_radius);
            if spiel.i_t == 150
                return;
            end
            
            %Gegner schnell bei Tanke, Tanke noch nicht ignoriert, keine Mine dazwischen UND noch Tanken in der TankList
            if (EnemyTimeToClosestTanke < constIgnoreTankeTime && ClosestEnemyTanke ~= ignoreTanke && ~enemyColliding) && numel(TankList) > 0
                %wie lange brauchen wir zu der am Gegner dichtesten Tanke?
                myPath = spiel.tanke(ClosestEnemyTanke).pos - me.pos;
                timeMeToTanke = norm(myPath) / projectVectorNorm(me.ges, myPath);
                if timeMeToTanke < 0
                    timeMeToTanke = inf;
                end
                %Ist diese Tanke unsere nächste Tanke und brauchen wir nicht mehr lange dorthin?
                %oder ist nur noch eine Tanke da und keine Mine im Weg
                %UND wir sind noch nicht im compMode
                if ((ClosestEnemyTanke == TankList{1} && timeMeToTanke < constIgnoreTankeTime + 0.5) || spiel.n_tanke == 1 && ~ownColliding) && ~tankeCompetition && ~cancelCompetition && timeMeToTanke < EnemyTimeToClosestTanke + 0.2
                    debugDisp('Tanken: compMode activated!');
                    tankeCompetition = true;
                    accpos = getAccPos(spiel.tanke(ClosestEnemyTanke).pos);
                    waypointList = [];
                    waypointList{1} = spiel.tanke(ClosestEnemyTanke).pos;
                    waypointList{2} = accpos;
                    debugDRAW();
                %sonst, ist noch mehr als 1 Tanke vorhanden - ignorieren,
                elseif ~tankeCompetition
                    debugDisp('Tanken: ignore Tanke');
                    ignoreTanke = ClosestEnemyTanke;
                    CreatePathAllTanken;
                end
                
                %comMode mit Vollbremsung abbrechen:
                if tankeCompetition
                    DistanceToStop = calcBreakDistance(norm(me.ges), 0);
                    DistanceToTanke = norm(me.pos-spiel.tanke(ClosestEnemyTanke).pos)-spiel.tanke_radius-spiel.spaceball_radius;
                    if DistanceToStop >= DistanceToTanke && timeMeToTanke - EnemyTimeToClosestTanke > 0.0001
                        debugDisp('Tanken: compMode canceled!');
                        waypointList = [];
                        cancelCompetition = true;
                        tankeCompetition = false;
                        debugDRAW();
                    end
                end
                
                %CancelComp beenden
                if cancelCompetition && EnemyTimeToClosestTanke > constIgnoreTankeTime
                    cancelCompetition = false;
                end
            end
        end
        
        %ignoreTanke entfernen:
        
        %existiert die ignorierte Tanke?
        if (ignoreTanke > 0 && ignoreTanke <= spiel.n_tanke)
            %wie lange braucht der Gegner zu dieser Tanke?
            enemyPath = spiel.tanke(ignoreTanke).pos - enemy.pos;
            timeEnemyToTanke = norm(enemyPath) / projectVectorNorm(enemy.ges, enemyPath);
            if timeEnemyToTanke < 0
                timeEnemyToTanke = inf;
            end
            %braucht er lange oder ist nur noch eine Tanke da - entfernen
            if timeEnemyToTanke > constIgnoreTankeTime
                debugDisp('Tanken: ignored Tanke deleted');
                ignoreTanke = 0;
                CreatePathAllTanken();
            end
        end
    end


%% Angriff
    %Angriff
    function attackEnemy()
        persistent lastAttackMode;
        waitForEnemy = false;
        
        %lockon attack ist nur sicher, wenn sich zwischen Gegner und Mir
        %keine Mine befindet
        useLockonAttack = false;
        if (~corridorColliding(me.pos, enemy.pos, spiel.mine_radius*2.5))
            useLockonAttack = true;
        elseif (lastAttackMode == 2)    % real dangerous KAMIKAZE     
            %%check if about to collide
            safeSpaceballRadius = (spiel.spaceball_radius);

            %new emergency breaking - is it better?
            breakTime = norm(me.ges) / spiel.bes;
        
            checkPoint = me.pos + 0.5*me.ges*breakTime; %without acceleration
        
            if (~isWalkable(checkPoint, safeSpaceballRadius))
                %debugDisp('keeepAttack');
                useLockonAttack = true;
            else
                %debugDisp('switch attack');
            end
        end
        
        %select attack mode
        if (~useLockonAttack)
            directAttack();
            lastAttackMode = 1;
        else
            lockonAttack();
            lastAttackMode = 2;
        end
    end
    
    function directAttack()
        
        %check if path to enemy is free
        enemypos = calcEnemyHitPosition(constEnemyInterpMode);
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
            pathResolution = clamp(norm(enemypos-me.pos)/3, 0.05, 0.4);
            
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
        if (norm(rotMeGes(2)-rotEnemyGes(2)) < 0.001 && norm(rotMePos(2)-rotEnemyPos(2)) < spiel.spaceball_radius*0.8 && ... 
                (rotEnemyGes(1)-rotMeGes(1))/(rotEnemyPos(1)-rotEnemyGes(1)) > -0.2)
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

    function erg = calcEnemyHitPosition(interpolationMode)
        % SMOOTH ACCELERATION VALUES
        persistent lastInterpEnemyPos;
        
        if (isempty(lastInterpEnemyPos))
            lastInterpEnemyPos = [0, 0];
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
            vvec = enemy.ges - me.ges;
            svec = enemy.pos - me.pos;
            v = norm(vvec);
            s = norm(svec);

            if (a > 0.001)
                lastCalculatedValue = (sqrt(v^2+2*a*s)-v)/a;
            else
                lastCalculatedValue = s/v;
            end
            
            lastCalculatedValue = clamp(lastCalculatedValue, 0, 1.3);
            
            lastTimeCalculated = spiel.i_t;
        end
        
        time = lastCalculatedValue;
    end


%% Verteidigung

    function time = defCornerTime(pos) %Berechnet die Zeit von unserem/vom gegnerischen Standort in Ecke X 
        edgepos = pos;
        meToEdge = edgepos - me.pos;
        enemyToEdge = edgepos - enemy.pos;

        metime = getTimeToAlignVelocity(me.ges, vecNorm(meToEdge)) + norm(meToEdge)/(norm(me.ges) + spiel.bes); %Zeit um Geschwindigkeitsvektor auszurichten + s/v + spiel.bes als const. damit nicht = 0 
        enemytime = getTimeToAlignVelocity(enemy.ges, vecNorm(meToEdge)) + norm(enemyToEdge)/(norm(enemy.ges) + spiel.bes);
      
        time = enemytime - metime; %Differenz berechnen, je größer der Wert desto besser 

        
        if (dot(vecNorm(meToEdge), vecNorm(enemy.pos-me.pos)) > 0.6 && norm(meToEdge) > 0.06)  
            %Wenn in Richtung der Ecke + ca 25° zu jeder Seite der Gegner ist und wir uns im Radius von 0.06 vom WP befinden -> 100 Strafsekunden
            time = time - 100;
            
            if (dot(vecNorm(meToEdge), vecNorm(enemy.pos-me.pos)) > 0.9)
                time = time - 100;
            end
        end
        
        
    end

    function tEcke = bestDefCorner() 
        cornerNodes = {[0.02,0.98], [0.98,0.98], [0.02,0.02], [0.98,0.02]};
        tEcke = [0, 0];
        savetime = -Inf;
        
        for i=1:4
            checktime = defCornerTime(cornerNodes{i}); %Zeitdiff. für alle Ecken berechnen 
            
            if (savetime < checktime) %Zeit für Ecke 1 überschreibt savetime und wir als neue savetime gespeichert. Die neue Savetime wird nur von größeren Zeitdiffs überschrieben. 
                tEcke = cornerNodes{i}; %Ecke von Zeit X wird als beste Ecke festgelegt und ggf. wieder überschrieben 
                savetime = checktime;
            end
        end
    end

    function fleeEnemy()
        if  spiel.n_mine > 4
            mineTricking;
            changePathForDefence();
        else
            cornerTricking();
        end
    end

    function cornerTricking()
        cornerNodes = {[0.02,0.98], [0.98,0.98], [0.02,0.02], [0.98,0.02]};
        if waitForEnemy == false
            debugDisp('Defence: cornerTricking 1');
            safeDeleteWaypoints();
            waypointList = appendToArray(waypointList, findPath(me.pos, bestDefCorner())); %Ecke Anfahren 
            waitForEnemy = true;
            debugDRAW();
            %waiting for the enemy
        else
            firstWaypoint = me.pos;
            if (numel(waypointList) >= 1)
                firstWaypoint = waypointList{1};
            end
            
            if (checkIfTooFastE () == true || norm(me.pos-enemy.pos) <= 0.15) && norm(firstWaypoint-me.pos) < 0.1
                
                    Verteidigung = true;  
                    debugDisp('Defence: cornerTricking 2');
                        
            
                    edges = zeros(4,2);
                    for i=1:4
                        edges(i, 1) = defCornerTime(cornerNodes{i}); %Berechnet Zeit für Ecke 1-4 -> Erste Spalte edges 
                        edges(i, 2) = i; %Nr. der Ecke -> 2. Spalte edges
                    end

                    nextCorner = sortrows(edges, [1 2]); %Sortiert Ecken nach den Werten der ersten Zeile
                    
                    waypointList = [];
                    waypointList{1} = cornerNodes{nextCorner(3, 2)}; %Setzt das 3. Element von nextCorner als WP 
                    
                    debugDRAW();
                %end
            end
        end
        constEmrBrkVelFac = 1.1;
        constEmrBrkAccFac = 0;
    end

    function mineTricking()
        ClosestMine = getNearestMineId(me.pos);
        enemyDist = spiel.mine(ClosestMine).pos - enemy.pos;
        if numel(waypointList) <= 1
             waypointList{1} =  spiel.mine(ClosestMine).pos + vecNorm(enemyDist)*spiel.mine_radius + constSafeBorder*vecNorm(enemyDist) + spiel.spaceball_radius*vecNorm(enemyDist)*1.2;
% +          waypointList{1} =  spiel.mine(ClosestMine).pos + vecNorm(enemyDist)*(spiel.mine_radius + constSafeBorder + spiel.spaceball_radius)*1.2;
        end
        
        debugDRAW();
    end

    function changePathForDefence()
        %Diese Funktion soll überprüfen, ob wir durch Abfahren unseres
        %Pfades den Gegner treffen würden und den Pfad entsprechend ändern.
        if norm(enemy.pos-me.pos) <= 0.1 && ~corridorColliding(enemy.pos, me.pos, spiel.spaceball_radius) && numel(waypointList) > 0
            %hier die wegpunktListe interpretieren:
            %Alle Wegpunkte von hinten durchgehen und prüfen, ob der Gegner
            %dazwischen liegt. Dann den letzten Wegpunkt dorthin setzen
            if enemyLineColliding(waypointList{1}, me.pos, spiel.spaceball_radius)
                debugDisp('DEFENCE: enemy is in our way!');
                waypointList = [];
            end
        end
    end

    function erg = enemyLineColliding(startp, endp, radius)
        erg = false;
        interpolatedEnemyPos = calcEnemyHitPosition(constEnemyInterpMode);
        dist = distanceLinePoint(startp, endp, interpolatedEnemyPos);
        if (dist < spiel.spaceball_radius+radius)
            erg = true;
            return;
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
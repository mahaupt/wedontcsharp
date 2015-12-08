function bes = beschleunigung(spiel, farbe)
%% Variablen und Konstanten

    %Wegpunkte
    global X;
    global Y;
    global G;

    %Legt den Sicherheitsabstand fest
    margin = 0.0025;
    
    %Umrechnungskonstante von Grad auf Radiant
    deg2rad = pi/180;
    %Umrechnungskonstante von Radiant auf Grad
    rad2deg = 180/pi;
    
    %Gradaufteilung
    grad = 10:10:360;
    
    %Zeitschritt
    dt = spiel.dt;
    
    %Beschleunigungsbetrag
    nBes = spiel.bes;
    
    %Graphische Darstellung?
    graphics = false;%spiel.graphics;


%% Hilfsfunktionen

    %Gibt den Index der am nächsten an pos gelegenen Sache zurück
    function index = naechsteSache (sachen, pos)
        l = length(sachen);
        if l ~= 0
            tSachen(l) = 0;
            for j = 1:l
                tSachen(j) = norm(sachen(j).pos - pos);
            end
            [~, index] = min(tSachen);
        else
            index = -1;
        end
    end

    %Eine Rotationsmatrix
    function rot = rotMat(alpha)
        rot = [cos(alpha), -sin(alpha) ; sin(alpha), cos(alpha)];
    end

    %Notbremsung
    function emergBrake()
        %Vektorisierter Bremsweg
        if norm(wir.ges) > 10^-3
            vecBrems = dot(wir.ges + margin, wir.ges + margin)/(2 * nBes) * wir.ges/norm(wir.ges);
            vBx = abs(vecBrems(1));
            vBy = abs(vecBrems(2));
        else
            vecBrems = margin;
            vBx = margin;
            vBy = margin;
        end
        
        bes = [0,0];
        %Mine ausweichen
        if ~isempty(minen)
            m = minen(naechsteSache(minen, wir.pos));
            
            dir = wir.pos - m.pos;
            
            if norm(dir) - wir.radius - m.radius < norm(vecBrems)
                bes = dir;
            end
        end
        
        %Banden ausweichen
        if wir.pos(1) - wir.radius < vBx
            bes = bes + [1, 0];
        end
        if wir.pos(2) - wir.radius < vBy
            bes = bes + [0, 1];
        end
        if wir.pos(1) + wir.radius > 1 - vBx
            bes = bes + [-1, 0];
        end
        if wir.pos(2) + wir.radius > 1 - vBy
            bes = bes + [0, -1];
        end
        
        %Ansonsten einfach zur Mitte beschleunigen
        if norm(bes) < eps
            bes = [0.5, 0.5] - wir.pos;
        end
    end

    %Gibt den Weg mit dem kleinsten Abstand zurück
    function G_INDEX = minDistance(pos)
        %Weg des Wegevektors
        le = length(G);
        
        %Vorfüllen
        abs(le) = 0;

        %Alle Entfernungen berechnen
        for i = 1:le
            abs(i) = norm(pos - [X(i), Y(i)]);
        end

        %Geringsten Abstand auswählen
        [~, G_INDEX] = min(abs);
    end

    %Gibt den Weg mit dem größten Abstand zurück
    function G_INDEX = maxDistance(pos)
        %Weg des Wegevektors
        le = length(G);
        
        %Vorfüllen
        abs(le) = 0;

        %Alle Entfernungen berechnen
        for i = 1:le
            abs(i) = norm(pos - [X(i), Y(i)]);
        end

        %Geringsten Abstand auswählen
        [~, G_INDEX] = max(abs);
    end

    %Beschleunigung zuweisen
    function assignBes(deg)
        %Beschleunigung zuweisen
        if norm(wir.ges) > 10^-3
            bes = wir.ges * rotMat(deg * deg2rad);
        end
    end

    
%% Vorbereitungen und Zuweisungen

    %Tanken
    tanken = spiel.tanke;
    %Minen
    minen = spiel.mine;
    
    %Gegner und uns zuweisen
    if strcmp(farbe, 'rot')
        wir = spiel.rot;
        gegner = spiel.blau;
    else
        wir = spiel.blau;
        gegner = spiel.rot;
    end

    %Standardbeschleunigung zuweisen
    if isempty(tanken)
        bes = [0.5, 0.5] - wir.pos;
    else
        bes = tanken(naechsteSache(tanken, wir.pos)).pos - wir.pos;
    end
    
    if graphics
        %Grafik-Objekte aufräumen
        delete(findobj('Tag', 'PLoc'));
    end
    

%% Algorithmus

	%Bremszeit berechnen
	t_brems = norm(wir.ges)/nBes;
    
    %Testen, ob wir mehr als die Hälfte der Tankstellen haben oder Spontanangriff
    if (wir.getankt > gegner.getankt + spiel.n_tanke || (wir.getankt == gegner.getankt && spiel.n_tanke == 0)) || (gegner.getankt < wir.getankt && norm(wir.pos - gegner.pos) < 0.1 && 0.9397 < dot(wir.ges, gegner.ges)/(norm(wir.ges)*norm(gegner.ges)))
        %Maß für die Zeit, die zwischen uns und dem Gegner liegt
        t_rel = 1 * norm(gegner.pos - wir.pos)/norm(gegner.ges - wir.ges);
        %Projizierte Position
        gPos = gegner.pos + gegner.ges * t_rel;

        %Wenn projizierter Punkt außerhalb des Spielfeldes oder nicht innehalb eines gewissen Radius liegt, dann
        %direkt auf Gegner zuhalten
        if gPos(1) <= 0 || gPos(1) >= 1 || gPos(2) <= 0 || gPos(2) >= 1 || norm(gPos - gegner.pos) >= 0.5 || norm(gPos - wir.pos) >= 0.5
            gPos = gegner.pos;
        end

        %Fadenkreuz zeichnen
        if graphics
            hold on
            plot(gPos(1), gPos(2), 'r+', 'Tag', 'PLoc');
            hold off
        end

        %Alle möglichen Wegpunkte ermitteln
        [X,Y,G] = wegPunkte(wir.pos, wir.ges, t_brems + margin, grad, nBes, dt, wir.radius + margin, minen);

        %Länge des Wegpunkt-Vektors
        le = length(G);
        
        %Keine Wege
        if le == 0
            %Notbremsung
            emergBrake();
        else
            %Bester Weg
            g_index = minDistance(gPos);

            %Beschleunigung zuweisen
            assignBes(G(g_index));
        end

    %Nein, Gegner hat vielleicht schon mehr als die Hälfte
    elseif (gegner.getankt > wir.getankt + spiel.n_tanke) || (gegner.getankt > wir.getankt && norm(wir.pos - gegner.pos)/norm(wir.ges - gegner.ges) < t_brems / 2 && (0.9397 < dot(wir.ges, gegner.ges)/(norm(wir.ges) * norm(gegner.ges)) || 0.9397 < dot(wir.pos - gegner.pos, gegner.ges)/(norm(wir.pos - gegner.pos) * norm(gegner.ges))))
        %Maß für die Zeit, die zwischen uns und dem Gegner liegt
        t_rel = norm(gegner.pos - wir.pos)/norm(gegner.ges - wir.ges);
        %Projizierte Position
        gPos = (gegner.pos + gegner.ges * t_rel + 0.5 * gegner.bes * t_rel^2);

        %Wenn projizierter Punkt außerhalb des Spielfeldes liegt, dann
        %direkt von Gegner weghalten
        if gPos(1) <= 0 || gPos(1) >= 1 || gPos(2) <= 0 || gPos(2) >= 1
            gPos = gegner.pos;
        end
        
        %Eckenradius
        cRad = 0.2;

        %Sind wir in der Nähe einer Ecke
        corner = (wir.pos(1) < cRad && wir.pos(2) < cRad) || (wir.pos(1) > 1 - cRad && wir.pos(2) > 1 - cRad) || (wir.pos(1) > 1 - cRad && wir.pos(2) < cRad) || (wir.pos(1) < cRad && wir.pos(2) > 1 - cRad);
        
        if corner
            %Zum Mittelpunkt zielen
            gPos = [0.5, 0.5];
        end
        
        if graphics
            hold on
            plot(gPos(1), gPos(2), 'md', 'Tag', 'PLoc');
            hold off
        end

        %Alle möglichen Wegpunkte ermitteln
        [X,Y,G] = wegPunkte(wir.pos, wir.ges, t_brems + margin, grad, nBes, dt, wir.radius + margin, minen);
        
        %Länge des Wegpunkt-Vektors
        le = length(G);
        
        %Keine Wege
        if le == 0
            %Notbremsung
            emergBrake();
        else
            %Beste Verteidigung checken
            g_indexMax = maxDistance(gPos);
            g_indexMin = minDistance([1, 1] - gPos);
            
            %Bester Weg
            if corner
                g_index = minDistance(gPos);
            else
                if norm([X(g_indexMax), Y(g_indexMax)] - gPos) > norm([X(g_indexMin), Y(g_indexMin)] - gPos)
                    g_index = g_indexMax;
                else
                    g_index = g_indexMin;
                end
            end
            
            %Beschleunigung zuweisen
            assignBes(G(g_index));
        end

    %Alles noch offen
    else
        %Gegnerische Tanke ausschließen
        t_gegner = naechsteSache(tanken,gegner.pos);
        if spiel.n_tanke > 1

            %Winkel zwischen der Richtung des Gegners und dem Weg zur Tanke vom Gegners aus
            tankWinkel = dot(tanken(t_gegner).pos - gegner.pos, gegner.ges) / (norm(tanken(t_gegner).pos - gegner.pos) * norm(gegner.ges));
            
            %Zeit, die der Gegner braucht, um dahin zu kommen
            gegnerTankeZeit = norm(tanken(t_gegner).pos - gegner.pos)/norm(gegner.ges);
            %Zeit, die wir brauchen, um dahin zu kommen
            wirTankeZeit = norm(tanken(t_gegner).pos - wir.pos)/norm(wir.ges);

            %Testen
            if (tankWinkel > 0.8660) && (gegnerTankeZeit < wirTankeZeit)
                if graphics
                    %Graphische Darstellung
                    hold on
                    plot(tanken(t_gegner).pos(1),tanken(t_gegner).pos(2),'mo','Tag','PLoc');
                    hold off
                end
                %Löschen
                tanken(t_gegner) = [];
            end
        end
        
        %Anzahl der Tanken
        le_t = length(tanken);

        %Tanken bewerten
        tankGuete(le_t) = 0;
        
        for t_i = 1:le_t
            tankRichtung = tanken(t_i).pos - wir.pos;
            tankRichtungG = tanken(t_i).pos - gegner.pos;
            tankGuete(t_i) = 4 * (1 - norm(tankRichtung)) + 2 * dot(wir.ges, tankRichtung) + norm(tankRichtungG) + (1 - dot(gegner.ges, tankRichtungG));
        end
        
        %Beste Tanke auswählen
        [~, t_tanke] = max(tankGuete);
        
        if graphics
            hold on
            plot(tanken(t_tanke).pos(1),tanken(t_tanke).pos(2),'m+','Tag','PLoc');
            hold off
        end
        
        %Alle möglichen Wegpunkte ermitteln
        [X,Y,G] = wegPunkte(wir.pos, wir.ges, t_brems + margin, grad, nBes, dt, wir.radius + margin, minen);

        %Keine Wege
        if isempty(G);
            %Notbremsung
            emergBrake();
        else
            %Bester Weg
            g_index = minDistance(tanken(t_tanke).pos);
            
            %Beschleunigung zuweisen
            assignBes(G(g_index));
        end
    end
    
    
    if graphics
        %Graphische Darstellung der Wegpunkte
        hold on
        %plot(X, Y, 'r*', 'MarkerSize', 1, 'Tag', 'PLoc');
        hold off
    end
    

    if graphics
        %Faktor berechnen
        sTime = spiel.i_t * dt;
        rTime = toc;
        factor = round(rTime/sTime * 100) / 100;
        
        %Zeitdarstellung
        text(-0.29, 0.33, ['Spielzeit: ', num2str(sTime), ' Sekunden'], 'Color', [0.1, 0.1, 1], 'EdgeColor', 'White', 'Tag', 'PLoc');
        text(-0.29, 0.29, ['Echtzeit: ', num2str(round(rTime * 100) / 100), ' Sekunden'], 'Color', [0.1, 0.1, 1], 'EdgeColor', 'White', 'Tag', 'PLoc');
        text(-0.29, 0.24, ['Faktor: ' , num2str(factor)], 'Color', [0.1, 0.1, 1], 'EdgeColor', 'White', 'Tag', 'PLoc');
    end
    
    
end
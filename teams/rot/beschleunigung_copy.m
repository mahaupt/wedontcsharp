function bes = beschleunigung( spiel, farbe )
    persistent middleTankPos;
    
    if strcmp(farbe, 'rot')
        me = spiel.rot;
        enemy = spiel.blau;
    else
        me = spiel.blau;
        enemy = spiel.rot;
    end
    
    %get middle tanke
    if spiel.i_t == 1
        for i=1:spiel.n_tanke;
            if (spiel.tanke(i).pos(1) == 0.5)
                middleTankPos = spiel.tanke(i).pos;
            end
        end
    end
    
    if (distanceLinePoint(me.pos, me.pos+me.ges*2, middleTankPos) < spiel.tanke_radius)
        bes = me.ges;
        return;
    end
    
    %copy beschleunigung
    bes = [-enemy.bes(1), enemy.bes(2)];
    
    
    
    
    
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
    %Normalize 2D vector
    function erg = vecNorm(vec)
        n = norm(vec);
        erg = [vec(1)/n, vec(2)/n];
        
        if (n == 0)
            erg = [0 , 0];
        end
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %get perpendicular vector
    function erg = getPerpend(vec)
        erg = [-vec(2), vec(1)];
    end
end


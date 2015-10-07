function true_false = schnitt_kreis_bande (kreis)

if ...
        kreis.pos(1) - kreis.radius <= 0 || ... % Linke Bande
        kreis.pos(2) - kreis.radius <= 0 || ... % Obere Bande
        kreis.pos(1) + kreis.radius >= 1 || ... % Rechte Bande
        kreis.pos(2) + kreis.radius >= 1        % Untere Bande
    
    true_false = true;
    
else
    
    true_false = false;
    
end
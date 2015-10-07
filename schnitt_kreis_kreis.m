function true_false = schnitt_kreis_kreis (kreis_1, kreis_2)

if ...
        (kreis_1.radius + kreis_2.radius)^2 >= ...
        (kreis_1.pos(1) - kreis_2.pos(1))^2 + ...
        (kreis_1.pos(2) - kreis_2.pos(2))^2 
    
    true_false = true;
    
else
    
    true_false = false;
    
end

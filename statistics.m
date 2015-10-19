% Wichtig:
% clear variables in spaceballs.m muss auskommentiert sein!

clear all
range = 100
data = cell(range,5);


    for stat_i = 1 : 1 : range
        % run spaceballs, zeitraffer auto-on
        spaceballs
        zeitraffer_checkbox_anfangswert = true;
       
        % aktueller Schritt in data Zeile i, Spalte 2 eintragen
        data{stat_i,1} = horzcat('Lauf Nr.:  ', num2str(stat_i));      
        
        % data füllen: wenn verloren, dann den Seed und den Grund ausgeben
        if spiel.blau.punkte == 1
           data{stat_i,2} = 1;
           data{stat_i,3} = 'Gewonnnen';
           data{stat_i,4} = ' ';
           data{stat_i,5} = ' ';
        else
           data{stat_i,2} = 0;
           data{stat_i,3} = 'Verloren';
           data{stat_i,4} = spiel.blau.ereignis;
           data{stat_i,5} = horzcat('Seed: ', num2str(r));
        end
        
        % clear variables vgl. Spaceballs.m ausführen, Ausnahme: data und range
        clearvars -except data range

        
    end

% und hübsch darstellen ;) 
    
sumWins = sum(cellfun(@double,data(:,2)))

Quote = horzcat('Von ', num2str(range), ' Spielen wurde(n) ', num2str(sumWins), ' gewonnen! (', num2str(sumWins/range*100),'%)')
Statistische_Erhebung = data
 



 

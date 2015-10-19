% Wichtig:
% clear variables in spaceballs.m muss auskommentiert sein!
% Zeitraffer-Funktion in spaceballs.m sollte aktiviert sein!

clear all
close all
clc

range = 1; % Anzahl Durchgänge
color = 1; % 1 für Blau, 2 für Rot


data = cell(range,7);

    for stat_i = 1 : 1 : range
        % run spaceballs
        spaceballs
       
        if color == 2
            me = spiel.rot;
            enemy = spiel.blau;
        else
            me = spiel.blau;
            enemy = spiel.rot;
        end
        
        % aktueller Schritt in data Zeile i, Spalte 2 eintragen
        data{stat_i,2} = horzcat(num2str(stat_i));
        
        % data füllen: wenn verloren, dann den Seed und den Grund ausgeben
        if spiel.blau.punkte == 1
           data{stat_i,1} = 1;
           data{stat_i,3} = 'Gewonnnen';
           data{stat_i,4} = enemy.ereignis;
           data{stat_i,5} = spiel.i_t/100;
           data{stat_i,6} = horzcat('# ', num2str(r));
           if strcmp(me.ereignis,'Rot trifft Blau.') || strcmp(me.ereignis,'Blau trifft Rot.')
                data{stat_i,7} = 1;
           else
               data{stat_i,7} = 0;
           end
        elseif spiel.rot.punkte == 1
           data{stat_i,1} = 0;
           data{stat_i,3} = 'Verloren';
           data{stat_i,4} = me.ereignis;
           data{stat_i,5} = spiel.i_t/100;
           data{stat_i,6} = horzcat('# ', num2str(r));
        elseif spiel.blau.getankt > spiel.rot.getankt
           data{stat_i,1} = 2;
           data{stat_i,3} = 'Unent. Angr.';
           data{stat_i,4} = me.ereignis;
           data{stat_i,5} = spiel.i_t/100;
           data{stat_i,6} = horzcat('# ', num2str(r));
        elseif spiel.blau.getankt < spiel.rot.getankt
           data{stat_i,1} = 3;
           data{stat_i,3} = 'Unent. Vert.';
           data{stat_i,4} = me.ereignis;
           data{stat_i,5} = spiel.i_t/100;
           data{stat_i,6} = horzcat('# ', num2str(r));
        else
           data{stat_i,1} = 99;
           data{stat_i,3} = 'ERROR';
           data{stat_i,4} = 'ERROR';
           data{stat_i,5} = spiel.i_t/100;
           data{stat_i,6} = horzcat('# ', num2str(r));
        end
        
        % clear variables vgl. Spaceballs.m ausführen, Ausnahme: data und range
        clearvars -except data range

        
    end

% Prozente berechnen und hübsch darstellen
sumWins = 0;
sumLose = 0;
sumAttack = 0;
sumDefense = 0;
gegErwischt = 0;
for i=1:range
    if cellfun(@double,data(i,1)) == 1
        sumWins = sumWins + 1;
    elseif cellfun(@double,data(i,1)) == 0
        sumLose = sumLose + 1;
        if cellfun(@double,data(i,7)) == 1
            gegErwischt = gegErwischt + 1;
        end
    elseif cellfun(@double,data(i,1)) == 2
        sumAttack = sumAttack + 1;
    elseif cellfun(@double,data(i,1)) == 3
        sumDefense = sumDefense + 1;
    end
end

medianTime = 0;
for i=1:range
    if cellfun(@double,data(i,1)) == 1
       medianTime = medianTime + cellfun(@double,data(i,5)); 
    end
end
medianTime = medianTime/sumWins;

Satz = horzcat('Von ', num2str(range), ' Spielen wurden ', num2str(sumWins), ' in durchschnittlich ', num2str(medianTime), ' Sekunden gewonnen.');
Quote1 = horzcat('Gewonnen: ', num2str(sumWins/range*100),' %');
Quote2 = horzcat('   davon den Gegner erwischt: ', num2str(gegErwischt/sumWins),' %');
Quote3 = horzcat('Verloren: ', num2str(sumLose/range*100),' %');
Quote4 = horzcat('   davon in Mine gefahren: ');
Quote5 = horzcat('   davon in Bande gefahren: ');
Quote6 = horzcat('Unentschieden im Angriff: ', num2str(sumAttack/range*100),' %');
Quote7 = horzcat('Unentschieden in der Verteidigung: ', num2str(sumDefense/range*100),' %');

Statistische_Erhebung = data(:,2:6);
clc;
disp(Satz);
disp(' ');
disp(Quote1);
disp(Quote2);
disp(Quote3);
disp(Quote4);
disp(Quote5);
disp(Quote6);
disp(Quote7);
disp(' ');
disp(Statistische_Erhebung);
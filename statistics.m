clear all
close all
clc

% Wichtig:
% In spaceballs.m "clear variables" auskommentieren!
% In spaceballs.m "spiel.zeitraffer_checkbox_anfangswert" auf true setzen!
% In spaceballs.m "rng shuffle" durch folgendes ersetzen:
    %rng shuffle
    %r = round(rand(1)*1000000);
    %rng(r);

Durchgaenge = 100; % Anzahl Durchgänge
Farbe = 'blau'; % Farbe des eigenen SpaceBalls eintragen: 'rot' oder 'blau'



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

data = cell(Durchgaenge,9);

    for stat_i = 1 : 1 : Durchgaenge
        % run spaceballs
        spaceballs
       
        if strcmp(Farbe,'rot')
            me = spiel.rot;
            enemy = spiel.blau;
            myName = spiel.rot.name;
            enemyName = spiel.blau.name;
        else
            me = spiel.blau;
            enemy = spiel.rot;
            myName = spiel.blau.name;
            enemyName = spiel.rot.name;
        end
                    
        % aktueller Schritt in data Zeile i, Spalte 2 eintragen
        data{stat_i,2} = horzcat(num2str(stat_i));
        data{stat_i,7} = 0;
        data{stat_i,8} = 0;
        data{stat_i,9} = 0;
        
        
        % data füllen: wenn verloren, dann den Seed und den Grund ausgeben
        if me.punkte == 1
           data{stat_i,1} = 1;
           data{stat_i,3} = 'Gewonnnen';
           data{stat_i,4} = enemy.ereignis;
           data{stat_i,5} = spiel.i_t/100;
           data{stat_i,6} = horzcat('# ', num2str(r));
           if strcmp(me.ereignis,'Rot trifft Blau.') || strcmp(me.ereignis,'Blau trifft Rot.')
                data{stat_i,7} = 1;
           end
        elseif enemy.punkte == 1
           data{stat_i,1} = 0;
           data{stat_i,3} = 'Verloren';
           data{stat_i,4} = me.ereignis;
           data{stat_i,5} = spiel.i_t/100;
           data{stat_i,6} = horzcat('# ', num2str(r));
           if strcmp(me.ereignis,'Rot trifft Mine.') || strcmp(me.ereignis,'Blau trifft Mine.')
               data{stat_i,8} = 1;
           elseif strcmp(me.ereignis,'Rot trifft Bande.') || strcmp(me.ereignis,'Blau trifft Bande.')
               data{stat_i,9} = 1;
           end
        elseif me.getankt > enemy.getankt
           data{stat_i,1} = 2;
           data{stat_i,3} = 'Unent. Angriff';
           data{stat_i,4} = me.ereignis;
           data{stat_i,5} = spiel.i_t/100;
           data{stat_i,6} = horzcat('# ', num2str(r));
        elseif me.getankt < enemy.getankt
           data{stat_i,1} = 3;
           data{stat_i,3} = 'Unent. Verteidigung';
           data{stat_i,4} = me.ereignis;
           data{stat_i,5} = spiel.i_t/100;
           data{stat_i,6} = horzcat('# ', num2str(r));
        else
           data{stat_i,1} = 99;
           data{stat_i,3} = 'ERROR';
           data{stat_i,4} = me.ereignis;
           data{stat_i,5} = spiel.i_t/100;
           data{stat_i,6} = horzcat('# ', num2str(r));
        end
        
        % clear variables vgl. Spaceballs.m ausführen, Ausnahme: data und range
        clearvars -except data Durchgaenge Farbe myName enemyName

        
    end

% Prozente berechnen und hübsch darstellen
sumWins = 0;
sumLose = 0;
sumAttack = 0;
sumDefense = 0;
sumERROR = 0;
gegErwischt = 0;
MineGetr = 0;
BandeGetr = 0;
for i=1:Durchgaenge
    if cellfun(@double,data(i,1)) == 1
        sumWins = sumWins + 1;
        if cellfun(@double,data(i,7)) == 1
            gegErwischt = gegErwischt + 1;
        end
    elseif cellfun(@double,data(i,1)) == 0
        sumLose = sumLose + 1;
        if cellfun(@double,data(i,8)) == 1
            MineGetr = MineGetr + 1;
        elseif cellfun(@double,data(i,9)) == 1
            BandeGetr = BandeGetr + 1;
        end
    elseif cellfun(@double,data(i,1)) == 2
        sumAttack = sumAttack + 1;
    elseif cellfun(@double,data(i,1)) == 3
        sumDefense = sumDefense + 1;
    elseif cellfun(@double,data(i,1)) == 99
        sumERROR = sumERROR + 1;
    end
end

medianTime = 0;
for i=1:Durchgaenge
    if cellfun(@double,data(i,1)) == 1
       medianTime = medianTime + cellfun(@double,data(i,5)); 
    end
end
medianTime = medianTime/sumWins;



Gegner = horzcat(myName, ' VS  ', enemyName);
Satz = horzcat('Von ', num2str(Durchgaenge), ' Spielen wurden ', num2str(sumWins), ' in durchschnittlich ', num2str(medianTime), ' Sekunden gewonnen.');
Quote1 = horzcat('Gewonnen: ', num2str(sumWins/Durchgaenge*100),' %');
Quote2 = horzcat('   davon den Gegner erwischt: ', num2str(gegErwischt/sumWins*100),' %');
Quote3 = horzcat('Verloren: ', num2str(sumLose/Durchgaenge*100),' %');
Quote4 = horzcat('   davon in Mine gefahren: ', num2str(MineGetr/sumLose*100),' %');
Quote5 = horzcat('   davon in Bande gefahren: ', num2str(BandeGetr/sumLose*100),' %');
Quote6 = horzcat('Unentschieden im Angriff: ', num2str(sumAttack/Durchgaenge*100),' %');
Quote7 = horzcat('Unentschieden in der Verteidigung: ', num2str(sumDefense/Durchgaenge*100),' %');
Quote8 = horzcat('Error: ', num2str(sumERROR/Durchgaenge*100),' %');

Statistische_Erhebung = data(:,2:6);
clc;
disp(Gegner);
disp(' ');
disp(Satz);
disp(' ');
disp(' ');
disp(Quote1);
disp(Quote2);
disp(' ');
disp(Quote3);
disp(Quote4);
disp(Quote5);
disp(' ');
disp(Quote6);
disp(' ');
disp(Quote7);
disp(' ');
disp(Quote8);
disp(' ');
disp(' ');
disp(Statistische_Erhebung);

   

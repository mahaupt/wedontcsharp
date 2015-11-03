% Wichtig:
% In spaceballs.m "clear variables" auskommentieren!
% In spaceballs.m " folgendes auskommentieren:
    %rng shuffle
    %r = round(rand(1)*1000000);
    %rng(r);
% In spaceballs.m "spiel.zeitraffer_checkbox_anfangswert" auf true setzen!
% Part 1 über Strg+Enter ausführen, erst danach Part 2 ab Zeile 218 wieder über Strg+Enter 

%%
%%%%%%%%%%%%%%%%%%% Part 1: Run Statistics %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
close all
clc

% Anzahl Durchgänge
Durchgaenge = 15; 

% Farbe des eigenen SpaceBalls eintragen: 'rot' oder 'blau'
Farbe = 'blau'; 

% Tabellenkopfzeile einfügen
rows = Durchgaenge+1;
                   
data = cell(rows,9);
data{1,2} = 'Durchlauf:';
data{1,3} = 'Ereignis:';
data{1,4} = 'Grund f. Ergeinis:';
data{1,5} = 'Zeit:';
data{1,6} = 'Seed:';

for i = 2 : rows
            spaceballs
 
            rng shuffle
            r = round(rand(1)*1000000);
            rng(r);
                   
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
        data{i,2} = num2str(i-1);
        data{i,7} = 0;
        data{i,8} = 0;
        data{i,9} = 0;
        
        
        % data füllen: wenn verloren, dann den Seed und den Grund ausgeben
        if me.punkte == 1
           data{i,1} = 1;
           data{i,3} = 'Gewonnnen';
           data{i,4} = enemy.ereignis;
           data{i,5} = spiel.i_t/100;
           data{i,6} = num2str(r);
           if strcmp(me.ereignis,'Rot trifft Blau.') || strcmp(me.ereignis,'Blau trifft Rot.')
                data{i,7} = 1;
           end
        elseif enemy.punkte == 1
           data{i,1} = 0;
           data{i,3} = 'Verloren';
           data{i,4} = me.ereignis;
           data{i,5} = spiel.i_t/100;
           data{i,6} = num2str(r);
           if strcmp(me.ereignis,'Rot trifft Mine.') || strcmp(me.ereignis,'Blau trifft Mine.')
               data{i,8} = 1;
           elseif strcmp(me.ereignis,'Rot trifft Bande.') || strcmp(me.ereignis,'Blau trifft Bande.')
               data{i,9} = 1;
           end
        elseif me.getankt > enemy.getankt
           data{i,1} = 2;
           data{i,3} = 'Unent. Angriff';
           data{i,4} = me.ereignis;
           data{i,5} = spiel.i_t/100;
           data{i,6} = num2str(r);
        elseif me.getankt < enemy.getankt
           data{i,1} = 3;
           data{i,3} = 'Unent. Verteidigung';
           data{i,4} = me.ereignis;
           data{i,5} = spiel.i_t/100;
           data{i,6} = num2str(r);
        else
           data{i,1} = 99;
           data{i,3} = 'ERROR';
           data{i,4} = me.ereignis;
           data{i,5} = spiel.i_t/100;
           data{i,6} = num2str(r);
        end
        
        % clear variables mit Ausnahmen vgl. Spaceballs.m ausführen
        clearvars -except Farbe seedData seedNum rows data Durchgaenge myName enemyName

        
    end

%%% Prozente berechnen und hübsch darstellen
sumWins = 0;
sumLose = 0;
sumAttack = 0;
sumDefense = 0;
sumERROR = 0;
gegErwischt = 0;
MineGetr = 0;
BandeGetr = 0;
for i=2:rows
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

%%% Durchschnittsspielzeit berechnen 

medianTime = 0;
for i=2:rows
    if cellfun(@double,data(i,1)) == 1
       medianTime = medianTime + cellfun(@double,data(i,5)); 
    end
end
medianTime = medianTime/sumWins;

%%% Seeds auslagern

loseSeeds = cell(i,1);
unentDefence = cell(i,1);
unentAttack = cell(i,1);

for i=1:Durchgaenge
        
    if cellfun(@double,data(i+1,1)) == 0
       loseSeeds{i+1,1} = data{i+1,6};
       
    elseif cellfun(@double,data(i+1,1)) == 2
       unentAttack{i+1,1} = data{i+1,6};
       
    elseif cellfun(@double,data(i+1,1)) == 3
       unentDefence{i+1,1} = data{i+1,6};
    end   
end

% leere cells löschen, reihenfolge wird beibehalten
loseSeeds = loseSeeds(~cellfun('isempty',loseSeeds));
unentDefence = unentDefence(~cellfun('isempty',unentDefence));
unentAttack = unentAttack(~cellfun('isempty',unentAttack));
 
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
    if sumLose/Durchgaenge*100 == 0
        disp(' ')
    else
        disp(Quote4);
        disp(Quote5);
        disp(' ');
    end    
disp(Quote6);
disp(' ');
disp(Quote7);
disp(' ');
disp(Quote8);
disp(' ');
disp(' ');
disp(Statistische_Erhebung);


%%% relevante Seeds speichern

save seeds.mat loseSeeds unentDefence unentAttack

seedData = [str2double(unentDefence); str2double(unentAttack); str2double(loseSeeds)];
seedNum = (length(unentDefence)+length(unentAttack)+length(loseSeeds));


%%
%%%%%%%%%%%%%%%%%%% Part 2: Seed Analysis incl. statistics %%%%%%%%%%%%%%%%

% Wichtig:
% In spaceballs.m "clear variables" auskommentieren!
% In spaceballs.m " folgendes auskommentieren:
    %rng shuffle
    %r = round(rand(1)*1000000);
    %rng(r);
% In spaceballs.m "spiel.zeitraffer_checkbox_anfangswert" auf true setzen!

clear all
close all
clc

load('seeds.mat')             
seedData = [str2double(unentDefence); str2double(unentAttack); str2double(loseSeeds)];
seedNum = (length(unentDefence)+length(unentAttack)+length(loseSeeds));

% Anzahl Durchgänge
Durchgaenge = 1; 

% Farbe des eigenen SpaceBalls eintragen: 'rot' oder 'blau'
Farbe = 'blau'; 

% Tabellenkopfzeile einfügen
rows = seedNum+1;
                   
data = cell(rows,9);
data{1,2} = 'Durchlauf:';
data{1,3} = 'Ereignis:';
data{1,4} = 'Grund f. Ergeinis:';
data{1,5} = 'Zeit:';
data{1,6} = 'Seed:';
             
for i = 2 : rows
              
              spaceballs
              r = seedData(i-1,:);
              rng(r);
                                           
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
        data{i,2} = num2str(i-1);
        data{i,7} = 0;
        data{i,8} = 0;
        data{i,9} = 0;
        
        
        % data füllen: wenn verloren, dann den Seed und den Grund ausgeben
        if me.punkte == 1
           data{i,1} = 1;
           data{i,3} = 'Gewonnnen';
           data{i,4} = enemy.ereignis;
           data{i,5} = spiel.i_t/100;
           data{i,6} = num2str(r);
           if strcmp(me.ereignis,'Rot trifft Blau.') || strcmp(me.ereignis,'Blau trifft Rot.')
                data{i,7} = 1;
           end
        elseif enemy.punkte == 1
           data{i,1} = 0;
           data{i,3} = 'Verloren';
           data{i,4} = me.ereignis;
           data{i,5} = spiel.i_t/100;
           data{i,6} = num2str(r);
           if strcmp(me.ereignis,'Rot trifft Mine.') || strcmp(me.ereignis,'Blau trifft Mine.')
               data{i,8} = 1;
           elseif strcmp(me.ereignis,'Rot trifft Bande.') || strcmp(me.ereignis,'Blau trifft Bande.')
               data{i,9} = 1;
           end
        elseif me.getankt > enemy.getankt
           data{i,1} = 2;
           data{i,3} = 'Unent. Angriff';
           data{i,4} = me.ereignis;
           data{i,5} = spiel.i_t/100;
           data{i,6} = num2str(r);
        elseif me.getankt < enemy.getankt
           data{i,1} = 3;
           data{i,3} = 'Unent. Verteidigung';
           data{i,4} = me.ereignis;
           data{i,5} = spiel.i_t/100;
           data{i,6} = num2str(r);
        else
           data{i,1} = 99;
           data{i,3} = 'ERROR';
           data{i,4} = me.ereignis;
           data{i,5} = spiel.i_t/100;
           data{i,6} = num2str(r);
        end
        
        % clear variables mit Ausnahmen vgl. Spaceballs.m ausführen
        clearvars -except Farbe seedData seedNum rows data Durchgaenge myName enemyName

        
    end

%%% Prozente berechnen und hübsch darstellen
sumWins = 0;
sumLose = 0;
sumAttack = 0;
sumDefense = 0;
sumERROR = 0;
gegErwischt = 0;
MineGetr = 0;
BandeGetr = 0;
for i=2:rows
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

%%% Durchschnittsspielzeit berechnen 

medianTime = 0;
for i=2:rows
    if cellfun(@double,data(i,1)) == 1
       medianTime = medianTime + cellfun(@double,data(i,5)); 
    end
end
medianTime = medianTime/sumWins;

%%% Seeds auslagern

loseSeeds = cell(i,1);
unentDefence = cell(i,1);
unentAttack = cell(i,1);

for i=1:Durchgaenge
        
    if cellfun(@double,data(i+1,1)) == 0
       loseSeeds{i+1,1} = data{i+1,6};
       
    elseif cellfun(@double,data(i+1,1)) == 2
       unentAttack{i+1,1} = data{i+1,6};
       
    elseif cellfun(@double,data(i+1,1)) == 3
       unentDefence{i+1,1} = data{i+1,6};
    end   
end

% leere cells löschen, reihenfolge wird beibehalten
loseSeeds = loseSeeds(~cellfun('isempty',loseSeeds));
unentDefence = unentDefence(~cellfun('isempty',unentDefence));
unentAttack = unentAttack(~cellfun('isempty',unentAttack));
 
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
    if sumLose/Durchgaenge*100 == 0
        disp(' ')
    else
        disp(Quote4);
        disp(Quote5);
        disp(' ');
    end    
disp(Quote6);
disp(' ');
disp(Quote7);
disp(' ');
disp(Quote8);
disp(' ');
disp(' ');
disp(Statistische_Erhebung);


%%% relevante Seeds speichern

save seeds.mat loseSeeds unentDefence unentAttack

seedData = [str2double(unentDefence); str2double(unentAttack); str2double(loseSeeds)];
seedNum = (length(unentDefence)+length(unentAttack)+length(loseSeeds));

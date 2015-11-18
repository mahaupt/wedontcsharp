% Wichtig:
% In spaceballs.m INITIALISIERUNG komplett auskommentieren!
% In spaceballs.m "spiel.zeitraffer_checkbox_anfangswert" auf true setzen!
% Part 1 über Strg+Enter ausführen, erst danach Part 2 ab Zeile 253 wieder über Strg+Enter
% In Part 2 steht zur Auswahl, ob alle oder nur Problem-Seeds erneut verwendet werden sollen


%%
%%%%%%%%%%%%%%%%%%%% Part 1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all
close all
clc
mex -setup C++

% Anzahl Durchgänge
Durchgaenge = 10; 
rows = Durchgaenge+1;


% Farbe des eigenen SpaceBalls eintragen: 'rot' oder 'blau'
Farbe = 'blau'; 


% Datentabelle erzeugen
data = cell(rows,6);
data{1,1} = 'Durchlauf:';
data{1,2} = 'Ereignis:';
data{1,3} = 'Grund f. Ergeinis:';
data{1,4} = 'Zeit:';
data{1,5} = 'Seed:';
% data{1,6}: 0=Verloren, 1=Gewonnen, 1.1=Gewonnen durch Gegner getroffen, 2=Crash in Mine, 3=Crash in Bande,
%            4=me.getankt>enemy.getankt, 5=me.getankt<enemy.getankt


% rng auf "zw" setzen, damit über r in der For-Schleife ein immer gleicher, reproduzierbarer
% Zufallswert erzeugt werden kann
zw = round(rand(1)*1000000);
zwVar = rng(zw);
totalTime = 0;


for i = 1 : Durchgaenge
    
           r = round(rand(1)*1000000);
           rng(r)
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
        data{i+1,1} = num2str(i);
 
        
% data füllen: wenn verloren, dann den Seed und den Grund ausgeben
        if me.punkte == 1
           data{i+1,2} = 'Gewonnnen';
           data{i+1,3} = enemy.ereignis;
           data{i+1,4} = spiel.i_t/100;
           data{i+1,5} = r;
           data{i+1,6} = 1;
           if strcmp(me.ereignis,'Rot trifft Blau.') || strcmp(me.ereignis,'Blau trifft Rot.')
                data{i+1,6} = 1.1;
           end
           
        elseif enemy.punkte == 1
           data{i+1,2} = 'Verloren';
           data{i+1,3} = me.ereignis;
           data{i+1,4} = spiel.i_t/100;
           data{i+1,5} = r;
           data{i+1,6} = 0;
           if strcmp(me.ereignis,'Rot trifft Mine.') || strcmp(me.ereignis,'Blau trifft Mine.')
               data{i+1,6} = 2;
           elseif strcmp(me.ereignis,'Rot trifft Bande.') || strcmp(me.ereignis,'Blau trifft Bande.')
               data{i+1,6} = 3;
           end
           
        elseif me.getankt > enemy.getankt
           data{i+1,2} = 'Unent. Angriff';
           data{i+1,3} = me.ereignis;
           data{i+1,4} = spiel.i_t/100;
           data{i+1,5} = r;
           data{i+1,6} = 4;
           
        elseif me.getankt < enemy.getankt
           data{i+1,2} = 'Unent. Verteidigung';
           data{i+1,3} = me.ereignis;
           data{i+1,4} = spiel.i_t/100;
           data{i+1,5} = r;
           data{i+1,6} = 5;
           
        else
           data{i+1,2} = 'Error';
           data{i+1,3} = me.ereignis;
           data{i+1,4} = spiel.i_t/100;
           data{i+1,5} = r;
           data{i+1,6} = 99;
        end
        
        totalTime = totalTime + toc
        i
% clear variables mit Ausnahmen
        clearvars -except data Durchgaenge Farbe myName enemyName zw zwVar totalTime
        close all
end


% weitere Variablen/Cells deklarieren
sumWins = 0;
sumLose = 0;
sumAttack = 0;
sumDefense = 0;
sumERROR = 0;
gegErwischt = 0;
MineGetr = 0;
BandeGetr = 0;
time = 0;

loseSeeds = cell(sumLose,1);
unentDefense = cell(sumDefense,1);
unentAttack = cell(sumAttack,1);


% Ereignissummen Berechnen 
for i = 1 : Durchgaenge
        if data{i+1,6} == 1
           sumWins = sumWins + 1;
        
        elseif data{i+1,6} == 1.1
           sumWins = sumWins + 1;
           gegErwischt = gegErwischt + 1;
            
        elseif data{i+1,6} == 0
           sumLose = sumLose + 1;
        
        elseif data{i+1,6} == 2
           sumLose = sumLose + 1;
           MineGetr = MineGetr + 1;
        
        elseif data{i+1,6} == 3
           sumLose = sumLose + 1;
           BandeGetr = BandeGetr + 1;
            
        elseif data{i+1,6} == 4
           sumAttack = sumAttack + 1;
            
        elseif data{i+1,6} == 5
           sumDefense = sumDefense + 1;
            
        elseif data{i+1,6} == 99
           sumERROR = sumERROR + 1;
        end
end               

% Durchschnittszeit  
for i = 1 : Durchgaenge
        if data{i+1,6} == 1 || 1.1
           time = time + data{i+1,4}; 
        end

        medianTime = time/sumWins;
        
        
% relevante Seeds speichern
        if data{i+1,6} == 0
           loseSeeds{i,1} = data{i+1,5};
           
        elseif data{i+1,6} == 2
           loseSeeds{i,1} = data{i+1,5};
           
        elseif data{i+1,6} == 3
           loseSeeds{i,1} = data{i+1,5};

        elseif data{i+1,6} == 4
           unentAttack{i,1} = data{i+1,5};

        elseif data{i+1,6} == 5
           unentDefense{i,1} = data{i+1,5};
        end   
end


% leere Cells löschen
loseSeeds = loseSeeds(~cellfun('isempty',loseSeeds));
unentDefense = unentDefense(~cellfun('isempty',unentDefense));
unentAttack = unentAttack(~cellfun('isempty',unentAttack));


% Seeddaten sammeln und speichern
seedData = [cell2mat(unentDefense); cell2mat(unentAttack); cell2mat(loseSeeds)];
seedNum = (length(unentDefense)+length(unentAttack)+length(loseSeeds));
seedsAll = cell2mat(data(2:end,5))
save seedsFaulty.mat seedData Farbe zw 
save seedsAll.mat seedsAll Farbe zw

% Auswertung num2str(sum(cell2mat(data(2:end,4))*1/60))
time = horzcat('Gesamtrechenzeit: ', num2str(round(totalTime/60,1)), ' Minuten');
Gegner = horzcat(myName, '  VS  ', enemyName);
SatzA = horzcat('Von ', num2str(Durchgaenge), ' Spielen wurden ', num2str(sumWins), ' gewonnen');
SatzAB = horzcat('Von ', num2str(Durchgaenge), ' Spielen wurden ', num2str(sumWins), ' in durchschnittlich ', num2str(medianTime), ' Sekunden gewonnen.');
Quote1 = horzcat('Gewonnen: ', num2str(sumWins/Durchgaenge*100),' %');
Quote2 = horzcat('   davon den Gegner erwischt: ', num2str(gegErwischt/sumWins*100),' %');
Quote3 = horzcat('Verloren: ', num2str(sumLose/Durchgaenge*100),' %');
Quote4 = horzcat('   davon in Mine gefahren: ', num2str(MineGetr/sumLose*100),' %');
Quote5 = horzcat('   davon in Bande gefahren: ', num2str(BandeGetr/sumLose*100),' %');
Quote6 = horzcat('Unentschieden im Angriff: ', num2str(sumAttack/Durchgaenge*100),' %');
Quote7 = horzcat('Unentschieden in der Verteidigung: ', num2str(sumDefense/Durchgaenge*100),' %');
Quote8 = horzcat('Error: ', num2str(sumERROR/Durchgaenge*100),' %');

Statistische_Erhebung = data(:,1:5);
clc
disp(Gegner);
disp(' ');

    if  sumWins == 0
        disp(SatzA);
    else
        disp(SatzAB);
    end
    
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
disp(time);
disp(' ');
disp(Statistische_Erhebung);


%%
%%%%%%%%%%%%%%%%%%%% Part 2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all
close all
clc
mex -setup C++


% Auswahl der Seedquelle
seedSource = 0          % 1 für alle Seeds, 0 für Problem-Seeds


% Seeds laden, Variablen anpassen
if seedSource == 1
   load('seedsAll.mat')
   seedData = seedsAll;
   seedNum = length(seedsAll);
elseif seedSource == 0
   load('seedsFaulty.mat')
   seedNum = length(seedData);
end


Durchgaenge = seedNum;
rows = Durchgaenge+1;
Farbe = Farbe;



% Datentabelle erzeugen
data = cell(rows,6);
data{1,1} = 'Durchlauf:';
data{1,2} = 'Ereignis:';
data{1,3} = 'Grund f. Ergeinis:';
data{1,4} = 'Zeit:';
data{1,5} = 'Seed:';
% data{1,6}: 0=Verloren, 1=Gewonnen, 1.1=Gewonnen durch Gegner getroffen, 2=Crash in Mine, 3=Crash in Bande,
%            4=me.getankt>enemy.getankt, 5=me.getankt<enemy.getankt


% rng auf "zw" setzen, damit über r in der For-Schleife ein immer gleicher, reproduzierbarer
% Zufallswert erzeugt werden kann (r könnte sonst vom letzten durchgang erhalten sein)
zw = round(rand(1)*1000000);
zwVar = rng(zw);
totalTime = 0;

if isempty(seedData)
    disp ('Keine Seeds vorhanden')
    return;
end 

for i = 1 : Durchgaenge
    
           r = seedData(i,1);
           rng(r)
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
        data{i+1,1} = num2str(i);
 
        
% data füllen: wenn verloren, dann den Seed und den Grund ausgeben
        if me.punkte == 1
           data{i+1,2} = 'Gewonnnen';
           data{i+1,3} = enemy.ereignis;
           data{i+1,4} = spiel.i_t/100;
           data{i+1,5} = r;
           data{i+1,6} = 1;
           if strcmp(me.ereignis,'Rot trifft Blau.') || strcmp(me.ereignis,'Blau trifft Rot.')
                data{i+1,6} = 1.1;
           end
           
        elseif enemy.punkte == 1
           data{i+1,2} = 'Verloren';
           data{i+1,3} = me.ereignis;
           data{i+1,4} = spiel.i_t/100;
           data{i+1,5} = r;
           data{i+1,6} = 0;
           if strcmp(me.ereignis,'Rot trifft Mine.') || strcmp(me.ereignis,'Blau trifft Mine.')
               data{i+1,6} = 2;
           elseif strcmp(me.ereignis,'Rot trifft Bande.') || strcmp(me.ereignis,'Blau trifft Bande.')
               data{i+1,6} = 3;
           end
           
        elseif me.getankt > enemy.getankt
           data{i+1,2} = 'Unent. Angriff';
           data{i+1,3} = me.ereignis;
           data{i+1,4} = spiel.i_t/100;
           data{i+1,5} = r;
           data{i+1,6} = 4;
           
        elseif me.getankt < enemy.getankt
           data{i+1,2} = 'Unent. Verteidigung';
           data{i+1,3} = me.ereignis;
           data{i+1,4} = spiel.i_t/100;
           data{i+1,5} = r;
           data{i+1,6} = 5;
           
        else
           data{i+1,2} = 'Error';
           data{i+1,3} = me.ereignis;
           data{i+1,4} = spiel.i_t/100;
           data{i+1,5} = r;
           data{i+1,6} = 99;
        end
        
        totalTime = totalTime + toc
        i
               
% clear variables mit Ausnahmen
        clearvars -except data Durchgaenge Farbe myName enemyName zw zwVar seedData totalTime seedSource
        close all
end


% weitere Variablen/Cells deklarieren
sumWins = 0;
sumLose = 0;
sumAttack = 0;
sumDefense = 0;
sumERROR = 0;
gegErwischt = 0;
MineGetr = 0;
BandeGetr = 0;
time = 0;

loseSeedsNew = cell(sumLose,1);
unentDefenseNew = cell(sumDefense,1);
unentAttackNew = cell(sumAttack,1);


% Ereignissummen Berechnen 
for i = 1 : Durchgaenge
        if data{i+1,6} == 1
           sumWins = sumWins + 1;
        
        elseif data{i+1,6} == 1.1
           sumWins = sumWins + 1;
           gegErwischt = gegErwischt + 1;
            
        elseif data{i+1,6} == 0
           sumLose = sumLose + 1;
        
        elseif data{i+1,6} == 2
           sumLose = sumLose + 1;
           MineGetr = MineGetr + 1;
        
        elseif data{i+1,6} == 3
           sumLose = sumLose + 1;
           BandeGetr = BandeGetr + 1;
            
        elseif data{i+1,6} == 4
           sumAttack = sumAttack + 1;
            
        elseif data{i+1,6} == 5
           sumDefense = sumDefense + 1;
            
        elseif data{i+1,6} == 99
           sumERROR = sumERROR + 1;
        end
end               

% Durchschnittszeit  
for i = 1 : Durchgaenge
        if data{i+1,6} == 1 || 1.1
           time = time + data{i+1,4}; 
        end

        medianTime = time/sumWins;
        
        
% relevante Seeds speichern
        if data{i+1,6} == 0
           loseSeedsNew{i,1} = data{i+1,5};
           
        elseif data{i+1,6} == 2
           loseSeedsNew{i,1} = data{i+1,5};
           
        elseif data{i+1,6} == 3
           loseSeedsNew{i,1} = data{i+1,5};

        elseif data{i+1,6} == 4
           unentAttackNew{i,1} = data{i+1,5};

        elseif data{i+1,6} == 5
           unentDefenseNew{i,1} = data{i+1,5};
        end   
end


% leere Cells löschen
loseSeedsNew = loseSeedsNew(~cellfun('isempty',loseSeedsNew));
unentDefenseNew = unentDefenseNew(~cellfun('isempty',unentDefenseNew));
unentAttackNew = unentAttackNew(~cellfun('isempty',unentAttackNew));


% Seeddaten sammeln und speichern
seedData = [cell2mat(unentDefenseNew); cell2mat(unentAttackNew); cell2mat(loseSeedsNew)];
seedNum = (length(unentDefenseNew)+length(unentAttackNew)+length(loseSeedsNew));
seedsAll = cell2mat(data(2:end,5))


if seedSource == 1
   save seedsAll.mat seedsAll Farbe zw
elseif seedSource == 0
   save seedsStillFaulty.mat seedData Farbe zw 
end


% Auswertung
time = horzcat('Gesamtrechenzeit: ', num2str(round(totalTime/60,1)), ' Minuten');
Gegner = horzcat(myName, '  VS  ', enemyName);
SatzA = horzcat('Von ', num2str(Durchgaenge), ' Spielen wurden ', num2str(sumWins), ' gewonnen');
SatzAB = horzcat('Von ', num2str(Durchgaenge), ' Spielen wurden ', num2str(sumWins), ' in durchschnittlich ', num2str(medianTime), ' Sekunden gewonnen.');
Quote1 = horzcat('Gewonnen: ', num2str(sumWins/Durchgaenge*100),' %');
Quote2 = horzcat('   davon den Gegner erwischt: ', num2str(gegErwischt/sumWins*100),' %');
Quote3 = horzcat('Verloren: ', num2str(sumLose/Durchgaenge*100),' %');
Quote4 = horzcat('   davon in Mine gefahren: ', num2str(MineGetr/sumLose*100),' %');
Quote5 = horzcat('   davon in Bande gefahren: ', num2str(BandeGetr/sumLose*100),' %');
Quote6 = horzcat('Unentschieden im Angriff: ', num2str(sumAttack/Durchgaenge*100),' %');
Quote7 = horzcat('Unentschieden in der Verteidigung: ', num2str(sumDefense/Durchgaenge*100),' %');
Quote8 = horzcat('Error: ', num2str(sumERROR/Durchgaenge*100),' %');

Statistische_Erhebung = data(:,1:5);
clc
disp(Gegner);
disp(' ');

    if  sumWins == 0
        disp(SatzA);
    else
        disp(SatzAB);
    end
    
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
disp(time);
disp(' ');
disp(Statistische_Erhebung);

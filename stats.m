function [] = stats(Durchgaenge, Farbe, seedSource)

% Seeds laden, Variablen anpassen
if seedSource == 0
   load('seedsFaulty.mat')
 if isempty(seedsFaulty)
    disp ('Keine Seeds vorhanden')
    return
 end
   seedNum = length(seedsFaulty);
   Durchgaenge = seedNum;
   Farbe = Farbe;
   rng(zw);
   
elseif seedSource == 1
   load('seedsAll.mat')   
 if isempty(seedsAll)
    disp ('Keine Seeds vorhanden')
    return
 end
   seedNum = length(seedsAll);
   Durchgaenge = seedNum;
   Farbe = Farbe;
   rng(zw);
   
elseif seedSource == 2
   Durchgaenge = Durchgaenge;
   zwVar = rng('shuffle');
   zw = zwVar.Seed;
   
else 
   disp('Fehler! Keine Seedquelle angegeben')
   return;
end


% Datentabelle erzeugen
data = cell(Durchgaenge+1,6);
data{1,1} = 'Durchlauf:';
data{1,2} = 'Ereignis:';
data{1,3} = 'Grund f. Ergeinis:';
data{1,4} = 'Zeit:';
data{1,5} = 'Seed:';
% data{1,6}: 0=Verloren, 1=Gewonnen, 1.1=Gewonnen durch Gegner getroffen, 2=Crash in Mine, 3=Crash in Bande, 4=me.getankt>enemy.getankt, 5=me.getankt<enemy.getankt


% Rechenzeitvariable einführen
totalTime = 0;


% Hauptschleife
for i = 1 : Durchgaenge

% Alle Fenster bis auf das letzte Spaceballspielfeldfenster schließen    
close all

% Wert für r ermitteln    
         if seedSource == 0    
             r = seedsFaulty(i,1);
             
         elseif seedSource == 1      
             r = seedsAll(i,1);
              
         elseif seedSource == 2
             r = round(rand(1)*1000000);
         end   

        rng(r);
        
% Rechenzeit und Schritt zum aktuellen Zeitpunkt ausgeben
        CurrentGame = horzcat('Gerade läuft Spiel Nr: ', num2str(i),' -- Seed Nr: ', num2str(r),' -- Gesamtlaufzeit bisher: ', num2str(totalTime), ' Sekunden.');
        disp(CurrentGame);
        
%Spiel durchführen
        try
            spaceballs
        catch
        end
        
%Zeit aktualisieren
        totalTime = totalTime + toc;
                      
% Zuweisung der Farbe        
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

        
% Aktueller Schritt in data Zeile i, Spalte 2 eintragen
        data{i+1,1} = num2str(i);
 
        
% Datentabelle füllen
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
                
% Clear variables mit Ausnahmen
        clearvars -except data Durchgaenge Farbe myName enemyName zw zwVar totalTime seedSource seedsFaulty seedsAll
        
end


% Weitere Variablen/Cells deklarieren
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


% Auswertungsschleife 
for i = 1 : Durchgaenge
    
% Ereignissummen Berechnen 
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
        
% Durchschnittszeit berechnen
        if data{i+1,6} == 1 || 1.1
           time = time + data{i+1,4}; 
        end

        medianTime = time/sumWins;
        
% Relevante Seeds speichern
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
  
% Leere Cells löschen
loseSeeds = loseSeeds(~cellfun('isempty',loseSeeds));
unentDefense = unentDefense(~cellfun('isempty',unentDefense));
unentAttack = unentAttack(~cellfun('isempty',unentAttack));


% Seeddaten sammeln und speichern
seedsFaulty = [cell2mat(unentDefense); cell2mat(unentAttack); cell2mat(loseSeeds)];
seedsAll = cell2mat(data(2:end,5));
save seedsFaulty.mat seedsFaulty Farbe zw 
save seedsAll.mat seedsAll Farbe zw


% Auswertung 
time = horzcat('Gesamtrechenzeit: ', num2str(round(totalTime/60,1)), ' Minuten');
Gegner = horzcat(myName, '  VS  ', enemyName);
SatzA = horzcat('Von ', num2str(Durchgaenge), ' Spielen wurden keine gewonnen :-(');
SatzAB = horzcat('Von ', num2str(Durchgaenge), ' Spielen wurden ', num2str(sumWins), ' in durchschnittlich ', num2str(medianTime), ' Sekunden gewonnen.');
Quote1 = horzcat('Gewonnen: ', num2str(round(sumWins/Durchgaenge*100,0)),' %');
Quote2 = horzcat('   davon den Gegner erwischt: ', num2str(round(gegErwischt/sumWins*100,0)),' %');
Quote3 = horzcat('Verloren: ', num2str(round(sumLose/Durchgaenge*100,0)),' %');
Quote4 = horzcat('   davon in Mine gefahren: ', num2str(round(MineGetr/sumLose*100,0)),' %');
Quote5 = horzcat('   davon in Bande gefahren: ', num2str(round(BandeGetr/sumLose*100,0)),' %');
Quote6 = horzcat('Unentschieden im Angriff: ', num2str(round(sumAttack/Durchgaenge*100,0)),' %');
Quote7 = horzcat('Unentschieden in der Verteidigung: ', num2str(round(sumDefense/Durchgaenge*100,0)),' %');
Quote8 = horzcat('Error: ', num2str(round(sumERROR/Durchgaenge*100,0)),' %');

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

end
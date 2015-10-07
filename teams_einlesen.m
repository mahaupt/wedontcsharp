function spiel = teams_einlesen (spiel)

cd teams/rot

spiel.rot.beschleunigung_handle = @beschleunigung;

rot_team = team_daten;

spiel.rot.name = rot_team.name;

spiel.rot.mitarbeiter = rot_team.mitarbeiter;

spiel.rot.logo = imread ('media/logo.jpg');

cd ../..

cd teams/blau

spiel.blau.beschleunigung_handle = @beschleunigung;

blau_team = team_daten;

spiel.blau.name = blau_team.name;

spiel.blau.mitarbeiter = blau_team.mitarbeiter;

spiel.blau.logo = imread ('media/logo.jpg');

cd ../..
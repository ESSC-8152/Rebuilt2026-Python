@echo off
set TEAM_NUMBER=8152
:: Calcul de l'IP (Standard FRC : 10.TE.AM.2)
set ROBOT_IP=10.%TEAM_NUMBER:~0,2%.%TEAM_NUMBER:~2,2%.2

echo --------------------------------------------------
echo   Deploiement du code - Équipe %TEAM_NUMBER%
echo --------------------------------------------------

:: 1. Test de connexion
echo Recherche de RoboRIO à %ROBOT_IP%...
ping -n 1 -w 1000 %ROBOT_IP% >nul
if %errorlevel% neq 0 (
    echo [ERROR] Robot non trouvé!
    echo Vérifier la connexion du Rio ou verifier le numéro d'équipe configuré.
    pause
    exit /b
)

echo [OK] Robot détecté.

:: 2. Execution du déploiement
echo Envoie du code au RIO...
py -3.13 -m robot.py deploy --nc

if %errorlevel% neq 0 (
    echo.
    echo [ERREUR] Déploiement annulé.
) else (
    echo.
    echo [SUCCÈS] Déploiement réussi!
)

pause
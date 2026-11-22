@echo off
rem IS29 car dashboard: checks that Docker's MQTT broker (container "mqtt")
rem listens on port 1883, starts the dashboard server and opens the page.
rem It never starts a broker of its own. Close the server window to stop it.
cd /d "%~dp0"

netstat -ano | findstr /r /c:":1883 .*LISTENING" >nul
if errorlevel 1 (
    echo No MQTT broker is listening on port 1883.
    echo Start Docker Desktop, then run:  docker start mqtt
    echo The dashboard keeps retrying the broker, so it connects once the broker is up.
) else (
    echo MQTT broker found on port 1883.
)

netstat -ano | findstr /r /c:":8080 .*LISTENING" >nul
if not errorlevel 1 (
    echo Port 8080 is already in use, probably by a dashboard that is already running.
    echo Opening the page; stop that process first if you want a fresh server.
    start "" http://localhost:8080
    goto :eof
)

for /f %%d in ('powershell -NoProfile -Command "Get-Date -Format yyyy-MM-dd"') do set DT=%%d
set LOG=%~dp0..\..\testing\system\logs\%DT%_dashboard.txt

echo Starting the dashboard on http://localhost:8080  (log: %LOG%)
start "IS29 dashboard server" cmd /k python "%~dp0server.py" --log "%LOG%"
timeout /t 2 /nobreak >nul
start "" http://localhost:8080

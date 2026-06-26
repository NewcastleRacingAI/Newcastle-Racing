@echo off
set "host=127.0.0.1"
set "port=8000"
g++ .\nrai_carmaker_sensors.cpp -lws2_32 -o sensor_client.exe
start .\sensor_client.exe -p 2210 -d %host% -x %port%
start .\sensor_client.exe -p 2211 -d %host% -x %port%

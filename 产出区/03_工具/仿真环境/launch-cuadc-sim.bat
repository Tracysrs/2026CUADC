@echo off
title CUADC Rescue Sim (gz-sim 10 Jetty)
rem CUADC scout-and-rescue field, adapted from 查阅区/08_参考/hgd_cudac/sim/cuadc_sim
rem Paths are resolved from this script location (%~dp0), so the folder can be moved/renamed freely.
cd /d D:\GAZEBO
set "SIM_DIR=%~dp0"
rem NOTE: do NOT leave a trailing ";" in GZ_SIM_RESOURCE_PATH -- this gz build
rem fails to resolve models if the path list has an empty trailing entry.
if defined GZ_SIM_RESOURCE_PATH (set "GZ_SIM_RESOURCE_PATH=%SIM_DIR%cuadc_rescue_sim\models;%GZ_SIM_RESOURCE_PATH%") else (set "GZ_SIM_RESOURCE_PATH=%SIM_DIR%cuadc_rescue_sim\models")
set "WORLD=%SIM_DIR%cuadc_rescue_sim\worlds\cuadc_rescue_single.sdf"
echo Starting CUADC rescue field ...
echo World: %WORLD%
echo Model path: %GZ_SIM_RESOURCE_PATH%
echo (Close this window to stop Gazebo)
"C:\Users\27514\AppData\Local\pixi\bin\pixi.exe" run gz sim -v3 "%WORLD%"

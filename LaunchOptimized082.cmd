@echo off
setlocal
call "%~dp0Config\environment.cmd"
if errorlevel 1 exit /b 1
cd /d "%~dp0"
"%AIMOOE_SLICER_EXE%" --additional-module-paths "%~dp0AimooeNavigation" --python-script "%~dp0Config\open_optimized082.py" %*

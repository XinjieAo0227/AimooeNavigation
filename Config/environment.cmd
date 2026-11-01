@echo off
if not defined AIMOOE_SLICER_EXE (
  echo Set AIMOOE_SLICER_EXE to the full path of your Slicer.exe.
  exit /b 1
)
if not exist "%AIMOOE_SLICER_EXE%" (
  echo Slicer executable not found. Check AIMOOE_SLICER_EXE.
  exit /b 1
)
exit /b 0

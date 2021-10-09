@echo off
setlocal

set "psCommand="(new-object -COM 'Shell.Application')^
.BrowseForFolder(0,'Please choose a folder for fbx sdk.',0,0).self.path""

for /f "usebackq delims=" %%I in (`powershell %psCommand%`) do set "folder=%%I"

setlocal enabledelayedexpansion
echo You chose !folder!
endlocal

cmake -B ./Build -G "Visual Studio 15 2017" -A x64 -DFBX_VENDER_DIR="%folder%"
pause
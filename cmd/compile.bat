@echo off

E:\aeroviz\.env313\Scripts\python.exe setup.py build_ext --inplace 

REM Determine source artifact (with or without .pyd)
set SRC_FILE=%~dp0..\src\HesaiLidar_driver_wrapper.cp313-win_amd64.pyd

REM Destination (two levels up to sibling repo)
set DEST_DIR=E:\aeroviz\aeroviz\sensors\hesai

if not exist "%SRC_FILE%" (
    echo Source artifact not found: "%SRC_FILE%"
    goto end
)

if not exist "%DEST_DIR%" (
    echo Creating destination directory "%DEST_DIR%"
    mkdir "%DEST_DIR%" || (
        echo Failed to create destination directory.
        goto end
    )
)

echo Copying "%SRC_FILE%" to "%DEST_DIR%\"
copy /Y "%SRC_FILE%" "%DEST_DIR%" >nul || (
    echo Copy failed.
    goto end
)
echo Copy succeeded.

:end
pause
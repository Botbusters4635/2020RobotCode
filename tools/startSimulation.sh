./gradlew installFrcUserProgramLinuxx86-64DebugExecutable

SCRIPT=$(readlink -f "$0")
SCRIPT_PATH=$(dirname "$SCRIPT")/../

export HALSIM_EXTENSIONS=$SCRIPT_PATH/build/tmp/expandedArchives/halsim_gui-2021.1.2-linuxx86-64debug.zip_2c5750537159c7a63673a43d91aa09e9/linux/x86-64/shared/libhalsim_guid.so:/home/ajahueym/Documents/Gits/Botbusters_Rebirth/build/tmp/expandedArchives/halsim_ds_socket-2021.1.2-linuxx86-64debug.zip_e83dec48c1fd92ad5f7b53c1ff612b27/linux/x86-64/shared/libhalsim_ds_socketd.so
export LD_LIBRARY_PATH=$SCRIPT_PATH/build/install/frcUserProgram/linuxx86-64/debug/lib
export DYLD_FALLBACK_LIBRARY_PATH=$SCRIPT_PATH/build/install/frcUserProgram/linuxx86-64/debug/lib
export DYLD_LIBRARY_PATH=$SCRIPT_PATH/build/install/frcUserProgram/linuxx86-64/debug/lib
$SCRIPT_PATH/build/install/frcUserProgram/linuxx86-64/debug/frcUserProgram

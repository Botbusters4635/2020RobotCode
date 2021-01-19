#!/bin/bash
timeout --preserve-status 5s ./build/gradlerio_simulateFrcUserProgramLinuxx86-64ReleaseExecutable.sh
PROGRAM_INFO=$?

if [ "$PROGRAM_INFO" == "0" ]; then
	exit -1
fi
exit 0

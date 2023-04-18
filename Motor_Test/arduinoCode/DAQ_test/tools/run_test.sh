#!/bin/bash

set -e -o pipefail

function echodo () {
        set -e -o pipefail
        echo "[BEGIN] $@" >&2
        "$@"
        echo "[END  ] $@" >&2
}

echodo pushd builds/release
echodo sudo ./characterization_test -t 1 --save -d /home/rapid/git/Rapid-ADCS/Motor_Test/arduinoCode/DAQ_test/builds/release/Speed_v_torque.csv
echodo sudo ./characterization_test -t 2 --save -d /home/rapid/git/Rapid-ADCS/Motor_Test/arduinoCode/DAQ_test/builds/release/Speed_control_precision.csv
echodo sudo ./characterization_test -t 3 --save -d /home/rapid/git/Rapid-ADCS/Motor_Test/arduinoCode/DAQ_test/builds/release/Max_speed.csv
echodo sudo ./characterization_test -t 4 --save -d /home/rapid/git/Rapid-ADCS/Motor_Test/arduinoCode/DAQ_test/builds/release/Current_draw.csv
echodo sudo ./characterization_test -t 5 --save -d /home/rapid/git/Rapid-ADCS/Motor_Test/arduinoCode/DAQ_test/builds/release/Frequency_response.csv
echodo popd
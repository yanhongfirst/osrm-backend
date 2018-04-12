#!/bin/sh

set -e

REQUESTS=$1
OUTPUT_DIR=$2

for overlap in "1.25" "1.33" "1.50" "1.75"; do
    for unpack in "2" "3" "4"; do
        for longer in "0.25" "0.33" "0.5" "0.75"; do
            for different in "0.5" "0.75" "0.85"; do
                for optimal in "0.1" "0.2" "0.3"; do
                    OUTPUT="${overlap}_${unpack}_${longer}_${different}_${optimal}_mld.csv"
                    echo "###### $OUTPUT ######"
                    OSRM_OVERLAP=$overlap OSRM_TO_UNPACK=$unpack OSRM_AT_MOST_LONGER=$longer OSRM_AT_LEAST_DIFFERENT=$different OSRM_AT_LEAST_OPTIMAL=$optimal ./build/osrm-routed -s -a mld &
                    sleep 5
                    ./scripts/osrm-runner.js -q $REQUESTS -f "$.routes..geometry" > $OUTPUT && killall osrm-routed
                    case $OUTPUT_PATH in
                        s3*)
                            aws s3 cp $OUTPUT $OUTPUT_DIR;;
                        *)
                            cp $OUTPUT $OUTPUT_DIR;;
                    esac
                done
            done
        done
    done
done

for overlap in "1.25" "1.33" "1.50" "1.75"; do
    for longer in "0.25" "0.33" "0.5" "0.75"; do
        for different in "0.5" "0.75" "0.85"; do
            OUTPUT="${overlap}_${longer}_${different}_ch.csv"
            echo "###### $OUTPUT ######"
            OSRM_OVERLAP=$overlap OSRM_AT_MOST_LONGER=$longer OSRM_AT_LEAST_DIFFERENT=$different ./build/osrm-routed -s -a ch &
            sleep 5
            ./scripts/osrm-runner.js -q $REQUESTS -f "$.routes..geometry" > $OUTPUT && killall osrm-routed
            case $OUTPUT_PATH in
                s3*)
                    aws s3 cp $OUTPUT $OUTPUT_DIR;;
                *)
                    cp $OUTPUT $OUTPUT_DIR;;
            esac
        done
    done
done

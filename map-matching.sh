#!/bin/bash

# bail if any errors ...
set -e

if [ "$JAVA_HOME" != "" ]; then
    JAVA=$JAVA_HOME/bin/java
fi

if [ "$JAVA" = "" ]; then
    JAVA=java
fi

echo "using $JAVA"

function set_jar {
    local module=$1
    local pattern="matching-$module/target/graphhopper-map-matching-*-dependencies.jar"
    if ! ls $pattern > /dev/null 2>&1; then
        mvn --projects hmm-lib -DskipTests=true install
        mvn --projects matching-$module,matching-core -DskipTests=true install assembly:single
    fi
    JAR=$(ls matching-$module/target/graphhopper-map-matching-*-dependencies.jar)
}

if [ "$1" = "action=start-server" ]; then
    set_jar "web"
    ARGS="graph.location=./graph-cache jetty.resourcebase=matching-web/src/main/webapp"
elif [ "$1" = "action=test" ]; then
    export MAVEN_OPTS="-Xmx400m -Xms400m"
    mvn clean test verify
    # return exit code of mvn
    exit $?
elif [ "$1" = "action=measurement" ]; then
    set_jar "tools"
    fname="measurement.$(date +%Y%m%d_%H%M%S)"
    ARGS="config=$CONFIG graph.location=$GRAPH datareader.file=$2 prepare.ch.weightings=fastest graph.flag_encoders=car prepare.min_network_size=10000 prepare.min_oneway_network_size=10000"
    current_commit=$(git log -n 1 --pretty=oneline | cut -d' ' -f1)
    function startMeasurement {
        echo -e "\nperforming measurement for commit $current_commit"
        mvn --projects matching-tools -DskipTests=true clean install assembly:single
        rm -f "$measurement_fname"
        "$JAVA" $JAVA_OPTS -cp "$JAR" com.graphhopper.matching.tools.Measurement $ARGS measurement.location="$measurement_fname"
    }

    # use all <last_commits> versions starting from HEAD
    last_commits=$3
    if [ -z "$last_commits" ]; then
        measurement_fname=$fname
        startMeasurement
    else
        # the following checks out the last commits, and tests each one. The code looks a bit
        # messier than that, as we merge the results into a single file (to make it easier to
        # compare.
        echo -e "commits (in order tested):\n--------------------------\n" >> "$fname"
        values=${fname}.values
        measurement_fname=${fname}.tmp
        commits=$(git rev-list HEAD -n "$last_commits")
        first=true
        empty_pad=""
        for commit in $commits; do
            git checkout "$commit"
            git log -n 1 --pretty=oneline >> "$fname"
            startMeasurement
            while read -r line; do
                key=${line%%=*}
                value=$(printf "%-20s" "${line##*=}")
                if [ "$first" = true ] ; then
                    printf "%-30s%s\n" "$key" "$value" >> "$values"
                else
                    if grep "$key" "$values"; then
                        sed -ri "s/($key.*)/\1$value/g" "$values"
                    else
                        # add a new row, using $empty_pad to get the column in the right place
                        printf "%-30s%s%s\n" "$key" "$empty_pad" "$value" >> "$values"
                    fi
                fi
            done < "$measurement_fname"
            first=false
            empty_pad=$(printf "%s%-20s" "$empty_pad" "")
        done
        echo -e "\nmeasurements:\n-------------\n" >> "$fname"
        cat "$values" >> "$fname"
        rm "$values"
        rm "$measurement_fname"
        # revert checkout
        git checkout "$current_commit"
    fi
    # in either case, echo the file:
    echo ""
    cat "$fname"
    exit 0
else
    set_jar "core"
    ARGS="$@"
fi

exec "$JAVA" $JAVA_OPTS -jar "$JAR" $ARGS prepare.min_network_size=0 prepare.min_one_way_network_size=0

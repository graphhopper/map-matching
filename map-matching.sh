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
        commit_info=$(git log -n 1 --pretty=oneline)
        echo -e "\nperforming measurement for commit $current_commit"
        "$JAVA" $JAVA_OPTS -cp "$JAR" com.graphhopper.matching.tools.Measurement $ARGS measurement.count=5000 measurement.location="$measurement_fname" measurement.gitinfo="$commit_info"
    }

    # use all <last_commits> versions starting from HEAD
    last_commits=$3
    if [ -z "$last_commits" ]; then
        measurement_fname=$fname
        tested_commit=$current_commit
        startMeasurement
    else
        echo "commits (in order tested):" >> $fname
        # TODO: check changes are committed?
        values=$fname.values
        measurement_fname=$fname.tmp
        commits=$(git rev-list HEAD -n $last_commits)
        first=true
        for commit in $commits; do
            git checkout $commit -q
            git log -n 1 --pretty=oneline >> $fname
            mvn --projects matching-tools -DskipTests=true clean install assembly:single
            rm -f $measurement_fname
            startMeasurement
            while read line; do
                key=${line%%=*}
                value=$(printf "%-10s" ${line##*=})
                if [ "$first" = true ] ; then
                    echo $key$value >> $values
                else
                    sed -ir "s/($key.*)/\1$value/g" $values
                fi
            done < $measurement_fname
        done
        echo -e "\nmeasurements:\n-------------\n" >> $fname
        cat $values >> $fname
        rm $values
        rm $measurement_fname
        # revert checkout
        git checkout $current_commit
    fi
else
    set_jar "core"
    ARGS="$@"
fi

exec "$JAVA" $JAVA_OPTS -jar $JAR $ARGS prepare.min_network_size=0 prepare.min_one_way_network_size=0

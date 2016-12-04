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
    set_jar "core"
    ARGS="config=$CONFIG graph.location=$GRAPH datareader.file=$2 prepare.ch.weightings=fastest graph.flag_encoders=car prepare.min_network_size=10000 prepare.min_oneway_network_size=10000"
    current_branch=$(git rev-parse --abbrev-ref HEAD)
    function startMeasurement {
        echo -e "\nperforming measurement for commit $current_commit"
        mvn --projects matching-core -DskipTests=true clean install assembly:single
        measurement_fname="measurement$(date +%Y-%m-%d_%H_%M_%S).properties"
        "$JAVA" $JAVA_OPTS -cp "$JAR" com.graphhopper.matching.util.Measurement $ARGS measurement.location="$measurement_fname"
    }

    # use all <last_commits> versions starting from HEAD
    last_commits=$3
    if [ -z "$last_commits" ]; then
        startMeasurement
        echo ""
        cat "$measurement_fname"
        exit 0
    else
        # the following checks out the last commits, and tests each one. The code looks a bit
        # messier than that, as we merge the results into a single file (to make it easier to
        # compare.
        combined="measurement_$(git log --format="%h" | head -n $last_commits | tail -n1)_$(git log --format="%h" | head -n1).properties"
        tmp_values="$combined.values"
        rm -f "$tmp_values"
        echo -e "commits (in order tested):\n--------------------------\n" > "$combined"
        commits=$(git rev-list HEAD -n "$last_commits" | tac) # NOTE: tac is to reverse so we start with oldest first
        first=true
        empty_pad=""
        header=$(printf "%30s" "")
        subheader=$header
        for commit in $commits; do
            git checkout "$commit"
            git log -n 1 --pretty=oneline >> "$combined"
            # do measurement for this commit
            startMeasurement
            # update headers:
            header=$(printf "%s%-20s" "$header" $(echo "$commit" | cut -c1-7))
            subheader=$(printf "%s%-20s" "$subheader" "-------")
            # now merge it:
            while read -r line; do
                key=${line%%=*}
                value=$(printf "%-20s" "${line##*=}")
                if [ "$first" = true ] ; then
                    printf "%-30s%s\n" "$key" "$value" >> "$tmp_values"
                else
                    if grep "$key" "$tmp_values" > /dev/null; then
                        sed -ri "s/($key.*)/\1$value/g" "$tmp_values"
                    else
                        # add a new row, using $empty_pad to get the column in the right place
                        printf "%-30s%s%s\n" "$key" "$empty_pad" "$value" >> "$tmp_values"
                    fi
                fi
            done < "$measurement_fname"
            first=false
            empty_pad=$(printf "%s%-20s" "$empty_pad" "")
        done
        echo -e "\nmeasurements:\n-------------\n" >> "$combined"
        echo "$header" >> "$combined"
        echo "$subheader" >> "$combined"
        cat "$tmp_values" >> "$combined"
        echo "" >> "$combined"

        # show as is:
        echo ""
        cat "$combined"

        # now plot (if supported)
        plot=1
        if ! type "bc" > /dev/null; then
            echo "install `bc` if you want plots"
            plot=0
        fi
        if ! type "gnuplot" > /dev/null; then
            echo "install `gnuplot` if you want plots"
            plot=0
        fi
        if [ $last_commits -lt 2 ]; then
            echo "plots are only performed if you specify a history of more than one commit"
            plot=0
        fi

        # remove tmp file when done
        trap "rm '$tmp_values'; git checkout $current_branch" EXIT INT TERM

        # plot all to file first, then plot again for interactivity:
        if [ $plot -eq 1 ]; then
            for first in {0..1}; do
                while read -u 3 -r line; do
                    data=
                    i=0
                    title=$(echo $line | cut -d" " -f1)
                    mx=
                    for commit in $commits; do
                        val=$(echo $line | cut -d" " -f"$((i+2))")
                        [[ ( $i -eq 0 || $(echo "$val > $mx" | bc) -eq 1 ) ]] && mx=$val
                        data="$data$i\t$(echo $commit | cut -c1-7)\t$val\n"
                        i=$((i + 1))
                    done
                    if [ $(echo "$mx > 0" | bc) -eq 1 ]; then
                        mn=0
                    else
                        mn=$mx
                        mx=0
                    fi
                    chart=$(echo -e "$data" | gnuplot -e "set terminal dumb; set yrange [$mn:$mx]; set boxwidth 0.5; set style fill solid; set border 1; unset tics; set xtics border; unset key; set title '$title'; plot '<cat' using 1:3:xtic(2) with boxes")
                    if [ $first -eq 0 ]; then
                        echo -e "$chart" >> "$combined"
                    else
                        echo ""
                        echo -e "$chart"
                        read -rsp $'Press any key to view the next chart, or CTRL + C to exit ...\n' -n1 key
                    fi
                done 3< "$tmp_values"
            done
        fi

        # done:
        exit 0
    fi
else
    set_jar "core"
    ARGS="$@"
fi

exec "$JAVA" $JAVA_OPTS -jar "$JAR" $ARGS prepare.min_network_size=0 prepare.min_one_way_network_size=0

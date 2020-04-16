#!/bin/sh
if [ "$IMPORT" != "False" ]; 
	then  
		echo "Importing file: $IMPORT..." && rm -rf /map-matcher/graph-cache/ || true && java $JAVA_OPTS -jar matching-web/target/graphhopper-map-matching-web-1.0-SNAPSHOT.jar import /map-matcher/map-data/$IMPORT --vehicle $VEHICLES && echo "Finished importing file: $IMPORT" ;
	else
		echo "Nothing to import, skipping..." ;
fi

echo "Starting map-matcher web UI..."
java $JAVA_OPTS -jar matching-web/target/graphhopper-map-matching-web-1.0-SNAPSHOT.jar server /map-matcher/map-data/config.yml
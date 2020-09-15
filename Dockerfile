FROM openjdk:8-jdk
#Arguments for --build-arg

ARG memory="-Xmx17g -Xms8g"

ENV VEHICLES="car"
ENV IMPORT=False
ENV JAVA_OPTS="-server -Xconcurrentio -XX:+UseG1GC -Ddw.server.applicationConnectors[0].bindHost=0.0.0.0 -Ddw.server.applicationConnectors[0].port=8989 $memory"

RUN mkdir -p /map-matcher && mkdir -p /map-matcher/graph-cache

# install node - only required for JS UI
RUN apt-get install -y wget \
       && curl -sL https://deb.nodesource.com/setup_13.x | bash - \
       && apt-get install -y nodejs \
	   && apt-get install -y maven

COPY . /map-matcher/

WORKDIR /map-matcher

#Folder that must contain the map to be imported and the config.yml for the web server
VOLUME [ "/map-matcher/map-data" ]
#Folder which will contain and preserve the graph-cache on container restart
VOLUME [ "/map-matcher/graph-cache" ]

EXPOSE 8989

#Package the jar
RUN mvn package -DskipTests && chmod +x import-and-run.sh

#Everything under this runs on container start
#Checks if the IMPORT environment variable equals the default value False, if it does skips import, if it doesn't, deletes the graph-cache folder and builds a new graph from the file specified
CMD ["sh","-c", "/map-matcher/import-and-run.sh"]

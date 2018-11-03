FROM openjdk:8-jdk

ADD ./ /usr/share/map-matching/

WORKDIR /usr/share/map-matching/

RUN apt-get update && apt-get install -y maven

RUN mvn package -DskipTests

EXPOSE 8989 8990

CMD ["java", "-jar", "matching-web/target/graphhopper-map-matching-web-0.12-SNAPSHOT.jar", "server", "docker/config.yml"]

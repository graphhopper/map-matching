FROM ubuntu:bionic

#############################
# Set up Java development kit
#############################

RUN apt-get update && apt-get -y install \
        default-jdk \
        maven

ENV PATH="/usr/lib/jvm/default-java/bin/:${PATH}"
ENV JAVA_HOME="/usr/lib/jvm/default-java"
ENV CLASSPATH="/usr/lib/jvm/default-java/lib"

#############################
# A current fix for java because of a corrupted key store.
# The error message is as follow:
# java.lang.RuntimeException: Unexpected error: java.security.InvalidAlgorithmParameterException: the trustAnchors parameter must be non-empty
#############################
RUN /usr/bin/printf '\xfe\xed\xfe\xed\x00\x00\x00\x02\x00\x00\x00\x00\xe2\x68\x6e\x45\xfb\x43\xdf\xa4\xd9\x92\xdd\x41\xce\xb6\xb2\x1c\x63\x30\xd7\x92' > /etc/ssl/certs/java/cacerts && \
    /var/lib/dpkg/info/ca-certificates-java.postinst configure


#############################
# Set up Map Matching
#############################

ADD ./ /usr/share/map-matching/

WORKDIR /usr/share/map-matching/

RUN mvn package -DskipTests

CMD ["java", "-jar", "matching-web/target/graphhopper-map-matching-web-0.11-SNAPSHOT.jar", "server", "docker/config.yml"]

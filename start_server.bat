CALL docker build -t graphhopper-map-matching .
CALL docker run -p 8989:8989 -t graphhopper-map-matching

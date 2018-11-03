#!/usr/bin/env bash

set -e
docker build -t graphhopper-map-matching .
docker run -p 8989:8989 -t graphhopper-map-matching

#!/bin/sh

# remove existing running ros container
docker stop ros 
docker rm ros
# build and run new ros container
docker build --rm -t ros . &&
docker run -d --name ros ros



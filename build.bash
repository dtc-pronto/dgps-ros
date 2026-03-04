#!/bin/bash
docker build --build-arg user_id=$(id -u) --rm -t dgps:dev -f Dockerfile.dgps .

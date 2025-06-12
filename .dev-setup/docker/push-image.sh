#!/bin/bash

docker login eshansavla0512


docker push eshansavla0512/jigless-planner:base-1.0
docker push eshansavla0512/jigless-planner:ci-1.0
docker push eshansavla0512/jigless-planner:run-1.0
#!/bin/bash
# WARNING: This file was auto-generated by dev-setup/setup.py.


# ------- base -------
# Build the base image
# -------------------------
docker build --file .dev-setup/docker/Dockerfile.base \
    --secret id=gitcreds,src=$HOME/.git-credentials \
    --tag jigless-planner:base-1.0 \
    .
docker tag jigless-planner:base-1.0 eshansavla0512/jigless-planner:base-1.0


# ------- ci -------
# Build the ci image
# -------------------------
docker build --file .dev-setup/docker/Dockerfile.ci \
    --secret id=gitcreds,src=$HOME/.git-credentials \
    --tag jigless-planner:ci-1.0 \
    .
docker tag jigless-planner:ci-1.0 eshansavla0512/jigless-planner:ci-1.0

# ------- run -------
# Build the run image
# -------------------------
docker build --file .dev-setup/docker/Dockerfile.run \
    --secret id=gitcreds,src=$HOME/.git-credentials \
    --tag jigless-planner:run-1.0 \
    .
docker tag jigless-planner:run-1.0 eshansavla0512/jigless-planner:run-1.0

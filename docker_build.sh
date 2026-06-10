#!/bin/bash
# docker_build.sh: Script to build the simulation Docker image of the Prisma Rover

IMAGE_TAG="prisma_rover:simulation"

echo "=== Compiling the Simulation Docker Image ==="
echo "Image Target Tag: $IMAGE_TAG"
echo "============================================="

docker build --network=host \
  --build-arg USER_ID="$(id -u)" \
  --build-arg GROUP_ID="$(id -g)" \
  -t "$IMAGE_TAG" \
  -f docker/Dockerfile .

echo "=== Build Complete: $IMAGE_TAG ==="

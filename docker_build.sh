#!/bin/bash
# docker_build.sh: Script to build the hardware-optimized Docker image for Prisma Rover

IMAGE_TAG="prisma_rover:real"

echo "=== Compiling the Hardware Docker Image ==="
echo "Image Target Tag: $IMAGE_TAG"
echo "==========================================="

# Build the image with host networking and pass host user IDs
docker build --network=host \
  --build-arg USER_ID="$(id -u)" \
  --build-arg GROUP_ID="$(id -g)" \
  -t "$IMAGE_TAG" \
  -f docker/Dockerfile .

echo "=== Build Complete: $IMAGE_TAG ==="

#!/bin/bash
# docker_build.sh: Script to build the modular Docker image of the Prisma Rover

PROFILE=${1:-"base"}

# Validate profile
if [ "$PROFILE" != "base" ] && [ "$PROFILE" != "yolo" ] && [ "$PROFILE" != "quantum" ] && [ "$PROFILE" != "full" ]; then
    echo "Error: Invalid profile '$PROFILE'."
    echo "Usage: $0 [base | yolo | quantum | full]"
    exit 1
fi

IMAGE_TAG="prisma_rover:$PROFILE"

echo "=== Compiling the Modular Docker Image ==="
echo "Selected Profile: $PROFILE"
echo "Image Target Tag: $IMAGE_TAG"
echo "=========================================="

# Build the specific target stage
docker build --network=host \
  --build-arg USER_ID="$(id -u)" \
  --build-arg GROUP_ID="$(id -g)" \
  --target "$PROFILE" \
  -t "$IMAGE_TAG" \
  -f docker/Dockerfile .

echo "=== Build Complete: $IMAGE_TAG ==="

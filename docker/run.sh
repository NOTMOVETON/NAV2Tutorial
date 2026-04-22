#!/usr/bin/env bash
set -e

SCRIPT_NAME=$(readlink -f "${BASH_SOURCE[0]}")
cd "$(dirname "$SCRIPT_NAME")"

CONTAINER_NAME="nav2_tutorials"
DOCKER_COMPOSE_FILE="docker-compose.yml"
IMAGE_NAME=$(docker compose -f $DOCKER_COMPOSE_FILE config | grep "image:" | head -n1 | awk '{print $2}')

print_help() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --build            Build image and run colcon build, but do not start containers"
    echo "  --colcon-only      Run colcon build inside temporary container (incremental)"
    echo "  --colcon-clean     Clean build/install/log volumes, then run colcon build"
    echo "  -j, --parallel-workers NUMBER  Set the number of parallel workers for colcon build"
    echo "  -cn, --container-name NAME  Set the container name (default: nav2_tutorials)"
    echo "  -h, --help         Show this help message and exit"
}

colcon_build() {
    local CLEAN=$1
    local PARALLEL_FLAG=""
    if [[ -n "$PARALLEL_WORKERS" ]]; then
        PARALLEL_FLAG="--parallel-workers $PARALLEL_WORKERS"
    fi

    echo "======================================"
    echo "Running colcon build inside temporary container..."
    echo "======================================"
    
    if [[ "$CLEAN" == "true" ]]; then
        docker compose -f "$DOCKER_COMPOSE_FILE" run --rm --no-deps \
            -e ROS_DOMAIN_ID=0 \
            "$CONTAINER_NAME" \
            bash -lc "echo \"[INFO] ROS_DISTRO: \$ROS_DISTRO\" \
                && source /opt/ros/\$ROS_DISTRO/setup.bash \
                && cd /ros2_ws && rm -rf build/* install/* log/* && colcon build $PARALLEL_FLAG"
    else
        docker compose -f "$DOCKER_COMPOSE_FILE" run --rm --no-deps \
            -e ROS_DOMAIN_ID=0 \
            "$CONTAINER_NAME" \
            bash -lc "echo \"[INFO] ROS_DISTRO: \$ROS_DISTRO\" \
                && source /opt/ros/\$ROS_DISTRO/setup.bash \
                && cd /ros2_ws && colcon build $PARALLEL_FLAG"
    fi
}

# Parse arguments
MODE="up"
PARALLEL_WORKERS=""
while [[ $# -gt 0 ]]; do
    case "$1" in
        --build)
            MODE="build"
            shift
            ;;
        --colcon-only)
            MODE="colcon-only"
            shift
            ;;
        --colcon-clean)
            MODE="colcon-clean"
            shift
            ;;
        -j|--parallel-workers)
            if [[ -n "$2" && "$2" =~ ^[0-9]+$ ]]; then
                PARALLEL_WORKERS="$2"
                shift 2
            else
                echo "Error: --parallel-workers requires a numeric value"
                exit 1
            fi
            ;;
        -cn|--container-name)
            if [[ -n "$2" && ! "$2" =~ ^- ]]; then
                CONTAINER_NAME="$2"
                shift 2
            else
                echo "Error: --container-name requires a value"
                exit 1
            fi
            ;;
        -h|--help)
            print_help
            exit 0
            ;;
        *)
            echo "Unknown argument: $1"
            echo "Use -h or --help for usage information."
            exit 1
            ;;
    esac
done

case "$MODE" in
    build)
        echo "Building image..."
        docker compose -f $DOCKER_COMPOSE_FILE --profile build build
        colcon_build true
        ;;
    colcon-only)
        colcon_build false
        ;;
    colcon-clean)
        colcon_build true
        ;;
    up)
        if ! docker image inspect "$IMAGE_NAME" >/dev/null 2>&1; then
            echo "Error: image '$IMAGE_NAME' not found."
            echo "Run: $0 --build"
            exit 1
        fi

        echo ""
        echo "======================================"
        echo "To enter the container, run:"
        echo "  docker exec -it $CONTAINER_NAME /bin/bash"
        echo "======================================"
        echo ""

        docker compose -f $DOCKER_COMPOSE_FILE up
        ;;
esac

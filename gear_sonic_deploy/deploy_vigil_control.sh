#!/bin/bash
set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

MODE="sim"
PORT="5556"
STATE_HOST="localhost"
STATE_PORT="5557"
STATE_TOPIC="g1_debug"
ODOM_SOURCE="auto"
DDS_INTERFACE="lo"
DDS_DOMAIN="0"
AUTO_START=""
PYTHON_BIN=""
LOG_FILE="auto"
MOVE_MODEL_FILE="auto"
MODEL_CHUNK_PAUSE="0.2"
MOVE_SETTLE_TIME="0.8"
MAX_MOVE_SPEED="1.0"
GRID_MAX_EXTRA_TIME="0.4"
GRID_TRIALS="3"
GRID_TIME_STEP="0.1"
GRID_FAR_DISTANCE_STEP="0.5"
GRID_PLAN_DIR="outputs/vigil_grid_plans"

show_usage() {
    echo "Usage: $0 [sim|real] [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --port PORT              ZMQ command/planner PUB port (default: 5556)"
    echo "  --state-host HOST        g1_debug state host (default: localhost)"
    echo "  --state-port PORT        g1_debug state port (default: 5557)"
    echo "  --state-topic TOPIC      g1_debug state topic (default: g1_debug)"
    echo "  --odom-source auto|dds|off  Use MuJoCo rt/odostate if available (default: auto)"
    echo "  --dds-interface IFACE    DDS interface for sim odom (default: lo)"
    echo "  --dds-domain ID          DDS domain id for sim odom (default: 0)"
    echo "  --python PATH            Python interpreter to run controller"
    echo "  --log-file PATH          CSV action log path (default: timestamped auto)"
    echo "  --move-model-file PATH|auto|off  Calibrated move model JSON (default: auto)"
    echo "  --model-chunk-pause SEC  Pause between split model-move chunks (default: 0.2)"
    echo "  --move-settle-time SEC   Idle time before measuring move result (default: 0.8)"
    echo "  --max-move-speed MPS     Max allowed move speed (default: 1.0)"
    echo "  --grid-max-extra-time SEC  Max grid extra time above distance/rate (default: 0.4)"
    echo "  --grid-trials N         Trials per grid point (default: 3)"
    echo "  --grid-time-step SEC    Grid execute-time step (default: 0.1)"
    echo "  --grid-far-distance-step M  Distance step when abs(distance)>1m (default: 0.5)"
    echo "  --grid-plan-dir PATH    Directory for resumable grid XML plans"
    echo "  --auto-start             Send start command after launch"
    echo "  -h, --help               Show this help"
    echo ""
    echo "Expected WBC launch in another terminal:"
    echo "  bash deploy.sh sim --input-type zmq_manager --output-type all"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        sim|real)
            MODE="$1"
            shift
            ;;
        --port)
            PORT="$2"
            shift 2
            ;;
        --state-host)
            STATE_HOST="$2"
            shift 2
            ;;
        --state-port)
            STATE_PORT="$2"
            shift 2
            ;;
        --state-topic)
            STATE_TOPIC="$2"
            shift 2
            ;;
        --odom-source)
            ODOM_SOURCE="$2"
            shift 2
            ;;
        --dds-interface)
            DDS_INTERFACE="$2"
            shift 2
            ;;
        --dds-domain)
            DDS_DOMAIN="$2"
            shift 2
            ;;
        --python)
            PYTHON_BIN="$2"
            shift 2
            ;;
        --log-file)
            LOG_FILE="$2"
            shift 2
            ;;
        --move-model-file)
            MOVE_MODEL_FILE="$2"
            shift 2
            ;;
        --model-chunk-pause)
            MODEL_CHUNK_PAUSE="$2"
            shift 2
            ;;
        --move-settle-time)
            MOVE_SETTLE_TIME="$2"
            shift 2
            ;;
        --max-move-speed)
            MAX_MOVE_SPEED="$2"
            shift 2
            ;;
        --grid-max-extra-time)
            GRID_MAX_EXTRA_TIME="$2"
            shift 2
            ;;
        --grid-trials)
            GRID_TRIALS="$2"
            shift 2
            ;;
        --grid-time-step)
            GRID_TIME_STEP="$2"
            shift 2
            ;;
        --grid-far-distance-step)
            GRID_FAR_DISTANCE_STEP="$2"
            shift 2
            ;;
        --grid-plan-dir)
            GRID_PLAN_DIR="$2"
            shift 2
            ;;
        --auto-start)
            AUTO_START="--auto-start"
            shift
            ;;
        -h|--help)
            show_usage
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown argument: $1${NC}" >&2
            show_usage
            exit 1
            ;;
    esac
done

echo -e "${CYAN}"
echo "╔══════════════════════════════════════════════════════════════════════╗"
echo "║                    VIGIL WBC PRIMITIVE CONTROLLER                   ║"
echo "╚══════════════════════════════════════════════════════════════════════╝"
echo -e "${NC}"

echo -e "${BLUE}[Mode]${NC} $MODE"
echo -e "${BLUE}[Controller PUB]${NC} command/planner -> tcp://*:${PORT}"
echo -e "${BLUE}[State SUB]${NC} ${STATE_TOPIC} <- tcp://${STATE_HOST}:${STATE_PORT}"
echo -e "${BLUE}[Sim Odom]${NC} source=${ODOM_SOURCE}, dds=${DDS_INTERFACE}, domain=${DDS_DOMAIN}"
echo -e "${BLUE}[Log]${NC} ${LOG_FILE}"
echo -e "${BLUE}[Move Model]${NC} ${MOVE_MODEL_FILE}"
echo ""

echo -e "${YELLOW}Run these terminals for simulation:${NC}"
echo "  terminal 1: python gear_sonic/scripts/run_sim_loop.py"
echo "  terminal 2: cd gear_sonic_deploy && bash deploy.sh sim --input-type zmq_manager --output-type all"
echo "  terminal 3: cd gear_sonic_deploy && bash deploy_vigil_control.sh sim"
echo ""

REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
if [[ -z "$PYTHON_BIN" ]]; then
    if [[ -x "$REPO_ROOT/.venv_sim/bin/python" ]]; then
        PYTHON_BIN="$REPO_ROOT/.venv_sim/bin/python"
    elif [[ -x "$REPO_ROOT/.venv_teleop/bin/python" ]]; then
        PYTHON_BIN="$REPO_ROOT/.venv_teleop/bin/python"
    else
        PYTHON_BIN="python3"
    fi
fi

echo -e "${BLUE}[Python]${NC} $PYTHON_BIN"
echo ""

if ! "$PYTHON_BIN" - <<'PY'
import msgpack
import zmq
import cyclonedds
PY
then
    echo -e "${RED}Missing Python dependencies.${NC}"
    echo "Use the sim venv or install: pyzmq msgpack cyclonedds"
    exit 1
fi

echo -e "${GREEN}Starting primitive controller...${NC}"
echo ""

"$PYTHON_BIN" scripts/vigil_primitive_controller.py \
    --port "$PORT" \
    --state-host "$STATE_HOST" \
    --state-port "$STATE_PORT" \
    --state-topic "$STATE_TOPIC" \
    --odom-source "$ODOM_SOURCE" \
    --dds-interface "$DDS_INTERFACE" \
    --dds-domain "$DDS_DOMAIN" \
    --log-file "$LOG_FILE" \
    --move-model-file "$MOVE_MODEL_FILE" \
    --model-chunk-pause "$MODEL_CHUNK_PAUSE" \
    --move-settle-time "$MOVE_SETTLE_TIME" \
    --max-move-speed "$MAX_MOVE_SPEED" \
    --grid-max-extra-time "$GRID_MAX_EXTRA_TIME" \
    --grid-trials "$GRID_TRIALS" \
    --grid-time-step "$GRID_TIME_STEP" \
    --grid-far-distance-step "$GRID_FAR_DISTANCE_STEP" \
    --grid-plan-dir "$GRID_PLAN_DIR" \
    $AUTO_START

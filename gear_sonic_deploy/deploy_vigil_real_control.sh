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

PORT="5556"
STATE_HOST="localhost"
STATE_PORT="5557"
STATE_TOPIC="g1_debug"
PYTHON_BIN=""
MOVE_MODEL_FILE="auto"
MODEL_CHUNK_PAUSE="0.2"
MOVE_SETTLE_TIME="0.8"
MAX_MOVE_SPEED="0.50"
ROTATE_CORRECTION_RETRIES="2"
ROTATE_CORRECTION_BOOST_DEG="10.0"
ROTATE_FEEDBACK_GAIN="1.0"
ROTATE_FEEDBACK_LIMIT_DEG="20.0"
AUTO_START=""

show_usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --port PORT              ZMQ command/planner PUB port (default: 5556)"
    echo "  --state-host HOST        g1_debug state host (default: localhost)"
    echo "  --state-port PORT        g1_debug state port (default: 5557)"
    echo "  --state-topic TOPIC      g1_debug state topic (default: g1_debug)"
    echo "  --python PATH            Python interpreter to run controller"
    echo "  --move-model-file PATH|auto|off  Calibrated move model JSON (default: auto)"
    echo "  --model-chunk-pause SEC  Pause between split model-move chunks (default: 0.2)"
    echo "  --move-settle-time SEC   Idle time after each move (default: 0.8)"
    echo "  --max-move-speed MPS     Real-robot max move speed clamp (default: 0.50)"
    echo "  --rotate-correction-retries N  Residual rotate correction attempts (default: 2)"
    echo "  --rotate-correction-boost-deg DEG  Extra facing angle for correction (default: 10)"
    echo "  --rotate-feedback-gain K Yaw error feedback gain (default: 1.0)"
    echo "  --rotate-feedback-limit-deg DEG  Max extra facing angle (default: 20.0)"
    echo "  --auto-start             Send start command after launch"
    echo "  -h, --help               Show this help"
    echo ""
    echo "Expected WBC launch in another terminal:"
    echo "  ./deploy.sh real --input-type zmq_manager --output-type all"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
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
        --python)
            PYTHON_BIN="$2"
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
        --rotate-correction-retries)
            ROTATE_CORRECTION_RETRIES="$2"
            shift 2
            ;;
        --rotate-correction-boost-deg)
            ROTATE_CORRECTION_BOOST_DEG="$2"
            shift 2
            ;;
        --rotate-feedback-gain)
            ROTATE_FEEDBACK_GAIN="$2"
            shift 2
            ;;
        --rotate-feedback-limit-deg)
            ROTATE_FEEDBACK_LIMIT_DEG="$2"
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
echo "║                 VIGIL REAL WBC PRIMITIVE CONTROLLER                 ║"
echo "╚══════════════════════════════════════════════════════════════════════╝"
echo -e "${NC}"

echo -e "${BLUE}[Controller PUB]${NC} command/planner -> tcp://*:${PORT}"
echo -e "${BLUE}[State SUB]${NC} ${STATE_TOPIC} <- tcp://${STATE_HOST}:${STATE_PORT}"
echo -e "${BLUE}[Move Model]${NC} ${MOVE_MODEL_FILE}"
echo -e "${BLUE}[Max Move Speed]${NC} ${MAX_MOVE_SPEED} m/s"
echo -e "${BLUE}[Rotate Feedback]${NC} gain=${ROTATE_FEEDBACK_GAIN}, limit=${ROTATE_FEEDBACK_LIMIT_DEG} deg"
echo ""

echo -e "${YELLOW}Run WBC in another terminal:${NC}"
echo "  cd $SCRIPT_DIR"
echo "  ./deploy.sh real --input-type zmq_manager --output-type all"
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
PY
then
    echo -e "${RED}Missing Python dependencies.${NC}"
    echo "Install: python -m pip install pyzmq msgpack"
    exit 1
fi

echo -e "${GREEN}Starting real primitive controller...${NC}"
echo ""

"$PYTHON_BIN" scripts/vigil_real_controller.py \
    --port "$PORT" \
    --state-host "$STATE_HOST" \
    --state-port "$STATE_PORT" \
    --state-topic "$STATE_TOPIC" \
    --move-model-file "$MOVE_MODEL_FILE" \
    --model-chunk-pause "$MODEL_CHUNK_PAUSE" \
    --move-settle-time "$MOVE_SETTLE_TIME" \
    --max-move-speed "$MAX_MOVE_SPEED" \
    --rotate-correction-retries "$ROTATE_CORRECTION_RETRIES" \
    --rotate-correction-boost-deg "$ROTATE_CORRECTION_BOOST_DEG" \
    --rotate-feedback-gain "$ROTATE_FEEDBACK_GAIN" \
    --rotate-feedback-limit-deg "$ROTATE_FEEDBACK_LIMIT_DEG" \
    $AUTO_START

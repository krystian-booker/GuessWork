#!/usr/bin/env bash
# Soak monitor for guesswork (docs/hardware-bringup.md Stage 8): polls the
# status endpoints on an interval, appends one CSV row of headline numbers
# per tick, and renders a PASS/FAIL verdict on counters that must stay flat
# over a healthy soak. Exits nonzero on FAIL.
#
# Usage:
#   scripts/soak_check.sh [--host localhost:8080] [--duration-s 10800]
#                         [--interval-s 10] [--out soak_<ts>.csv]
#                         [--max-reinits 3] [--pid auto]
set -u

HOST="localhost:8080"
DURATION_S=10800
INTERVAL_S=10
OUT="soak_$(date +%Y%m%d_%H%M%S).csv"
MAX_REINITS=3
PID="auto"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --host)        HOST="$2";        shift 2 ;;
        --duration-s)  DURATION_S="$2";  shift 2 ;;
        --interval-s)  INTERVAL_S="$2";  shift 2 ;;
        --out)         OUT="$2";         shift 2 ;;
        --max-reinits) MAX_REINITS="$2"; shift 2 ;;
        --pid)         PID="$2";         shift 2 ;;
        *) echo "unknown flag: $1" >&2; exit 2 ;;
    esac
done

# JSON field extraction: jq when present, python3 fallback.
if command -v jq > /dev/null 2>&1; then
    jget() { echo "$1" | jq -r "$2 // 0" 2>/dev/null || echo 0; }
else
    jget() {
        echo "$1" | python3 -c '
import json, sys
path = sys.argv[1].lstrip(".").split(".")
try:
    v = json.load(sys.stdin)
    for k in path:
        v = v[k]
    print(v if v is not None else 0)
except Exception:
    print(0)
' "$2" 2>/dev/null || echo 0
    }
fi

fetch() { curl -s --max-time 5 "http://${HOST}$1" 2>/dev/null; }

rss_mb() {
    local pid="$PID"
    [[ "$pid" == "auto" ]] && pid=$(pgrep -x guesswork | head -1)
    [[ -z "$pid" ]] && { echo 0; return; }
    local kb
    kb=$(ps -o rss= -p "$pid" 2>/dev/null | tr -d ' ')
    echo $(( ${kb:-0} / 1024 ))
}

echo "ts,uptime_s,imu_rate_hz,imu_crc_errors,imu_fw_drops,odom_rate_hz,tag_rate_hz,vio_rate_hz,fusion_mode,solve_p95_ms,pose_staleness_p95_ms,tag_pulse_to_fusion_p95_ms,queue_dropped,reinits,update_exceptions,output_send_errors,rss_mb" > "$OUT"

START=$(date +%s)
FAILED_POLLS=0
FIRST_CRC=""; FIRST_EXC=""; FIRST_REINITS=""; FIRST_QDROP=""
RSS_BASELINE=""
LAST_CRC=0; LAST_EXC=0; LAST_REINITS=0; LAST_QDROP=0; LAST_STALENESS=0; LAST_RSS=0

while true; do
    NOW=$(date +%s)
    UPTIME=$(( NOW - START ))
    [[ $UPTIME -ge $DURATION_S ]] && break

    IMU=$(fetch /api/imu/status)
    FUSION=$(fetch /api/fusion/status)
    if [[ -z "$IMU" || -z "$FUSION" ]]; then
        FAILED_POLLS=$(( FAILED_POLLS + 1 ))
        echo "$(date +%s),$UPTIME,POLL_FAILED" >> "$OUT"
        sleep "$INTERVAL_S"
        continue
    fi

    IMU_RATE=$(jget "$IMU" ".rate_hz")
    CRC=$(jget "$IMU" ".crc_errors")
    IMU_DROPS=$(jget "$IMU" ".fw_drops")
    ODOM_RATE=$(jget "$FUSION" ".sources.odom.rate_hz")
    TAG_RATE=$(jget "$FUSION" ".sources.tag.rate_hz")
    VIO_RATE=$(jget "$FUSION" ".sources.vio.rate_hz")
    MODE=$(jget "$FUSION" ".mode")
    SOLVE_P95=$(jget "$FUSION" ".latency.solve.p95_ms")
    STALENESS_P95=$(jget "$FUSION" ".latency.pose_staleness.p95_ms")
    TAGLAT_P95=$(jget "$FUSION" ".latency.tag_pulse_to_fusion.p95_ms")
    QDROP=$(jget "$FUSION" ".output.queue_dropped")
    REINITS=$(jget "$FUSION" ".reinits")
    EXC=$(jget "$FUSION" ".output.update_exceptions")
    SEND_ERR=$(jget "$FUSION" ".output.send_errors")
    RSS=$(rss_mb)

    [[ -z "$FIRST_CRC" ]] && { FIRST_CRC=$CRC; FIRST_EXC=$EXC; FIRST_REINITS=$REINITS; FIRST_QDROP=$QDROP; }
    # RSS baseline at the 5-minute mark (allocator warm-up excluded).
    [[ -z "$RSS_BASELINE" && $UPTIME -ge 300 ]] && RSS_BASELINE=$RSS
    LAST_CRC=$CRC; LAST_EXC=$EXC; LAST_REINITS=$REINITS; LAST_QDROP=$QDROP
    LAST_STALENESS=$STALENESS_P95; LAST_RSS=$RSS

    echo "$NOW,$UPTIME,$IMU_RATE,$CRC,$IMU_DROPS,$ODOM_RATE,$TAG_RATE,$VIO_RATE,$MODE,$SOLVE_P95,$STALENESS_P95,$TAGLAT_P95,$QDROP,$REINITS,$EXC,$SEND_ERR,$RSS" >> "$OUT"
    sleep "$INTERVAL_S"
done

# --- verdict ----------------------------------------------------------------
FAIL=0
note_fail() { echo "FAIL: $1"; FAIL=1; }

[[ -z "$FIRST_CRC" ]] && note_fail "no successful polls at all"
[[ $FAILED_POLLS -gt 0 ]] && note_fail "$FAILED_POLLS failed polls"
if [[ -n "$FIRST_CRC" ]]; then
    [[ $(( LAST_CRC - FIRST_CRC )) -gt 0 ]] && note_fail "crc_errors grew by $(( LAST_CRC - FIRST_CRC ))"
    [[ $(( LAST_EXC - FIRST_EXC )) -gt 0 ]] && note_fail "update_exceptions grew by $(( LAST_EXC - FIRST_EXC ))"
    [[ $(( LAST_REINITS - FIRST_REINITS )) -gt $MAX_REINITS ]] && note_fail "reinits grew by $(( LAST_REINITS - FIRST_REINITS )) (> $MAX_REINITS)"
    [[ $(( LAST_QDROP - FIRST_QDROP )) -gt 0 ]] && note_fail "fusion queue_dropped grew by $(( LAST_QDROP - FIRST_QDROP ))"
    awk "BEGIN { exit !($LAST_STALENESS > 50) }" && note_fail "pose_staleness p95 ${LAST_STALENESS} ms > 50 ms"
    if [[ -n "$RSS_BASELINE" && "$RSS_BASELINE" -gt 0 ]]; then
        LIMIT=$(( RSS_BASELINE * 12 / 10 ))
        [[ $LAST_RSS -gt $LIMIT ]] && note_fail "RSS grew ${RSS_BASELINE} -> ${LAST_RSS} MB (> 20%)"
    fi
fi

if [[ $FAIL -eq 0 ]]; then
    echo "PASS ($OUT)"
    exit 0
fi
echo "results: $OUT"
exit 1

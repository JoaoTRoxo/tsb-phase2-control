#!/usr/bin/env bash
# Run every experiment in headless Gazebo, in parallel containers, and regenerate
# docs/results. Needs the tsb-control-deps image and a built workspace (make build).
#
#   experiments/run.sh                 # everything
#   experiments/run.sh square steps    # only these runs
set -euo pipefail
cd "$(dirname "$0")/.."

IMAGE=${IMAGE:-tsb-control-deps}
BAGS=experiments/bags
RESULTS=docs/results

# name | seconds of simulation | launch arguments | plot title
RUNS=(
  "square|95|mission:=square|Square mission, classic LOS"
  "zigzag|80|mission:=zigzag|Zigzag mission, classic LOS"
  "steps|40|scenario:=steps|Step responses, default gains"
  "steps_no_feedforward|40|scenario:=steps variant:=no_feedforward|Step responses, without feedforward"
  "steps_no_gain_schedule|40|scenario:=steps variant:=no_gain_schedule|Step responses, without gain scheduling"
  "windup_back_calculation|29|scenario:=windup|Unreachable speed, back-calculation anti-windup"
  "windup_clamp|29|scenario:=windup variant:=clamp|Unreachable speed, conditional integration"
  "windup_none|29|scenario:=windup variant:=no_anti_windup|Unreachable speed, no anti-windup"
)

wanted=("$@")
selected() {
  [ ${#wanted[@]} -eq 0 ] && return 0
  local run
  for run in "${wanted[@]}"; do [ "$run" = "$1" ] && return 0; done
  return 1
}

# Containers on the same Docker bridge discover each other through DDS, so each run
# needs its own ROS domain or the robots start obeying each other's commands.
in_container() {
  docker run --rm --memory=3g -e ROS_DOMAIN_ID="$domain" -u "$(id -u):$(id -g)" -e HOME=/tmp -v /etc/passwd:/etc/passwd:ro -v "$PWD:/ws" -w /ws "$IMAGE" \
    bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && $1"
}

mkdir -p "$BAGS" "$RESULTS"
pids=()
domain=30
for entry in "${RUNS[@]}"; do
  IFS='|' read -r name seconds arguments title <<<"$entry"
  domain=$((domain + 1))
  selected "$name" || continue
  rm -rf "${BAGS:?}/$name"
  (
    # Gazebo needs about 15 s to start and spawn the robot before the run begins.
    in_container "timeout -s INT -k 10 $((seconds + 15)) ros2 launch tsb_control sim.launch.py \
      $arguments bag:=$BAGS/$name; ros2 run tsb_control plot_run.py $BAGS/$name $RESULTS \
      --title '$title'" >"$BAGS/$name.log" 2>&1
    echo "finished $name"
  ) &
  pids+=($!)
done
for pid in "${pids[@]}"; do wait "$pid"; done

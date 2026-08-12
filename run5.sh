#!/bin/bash
source /ws/install/setup.bash
python3 -c "
import json
d = json.load(open('/ws/scenarios/index.json'))
d.sort(key=lambda s: s['detour_ratio'])
for s in d[:5]:
    print(s['scenario_id'], s['world'], s['map'], s['goal_x'], s['goal_y'], s['optimal_path'])
" | while read SID W M X Y O; do
  echo "=== $SID ==="
  ros2 run harness_runner run_scenario --world "$W" --map "$M" \
    --x "$X" --y "$Y" --optimal-path "$O" --goal-timeout 90 \
    --scenario-id "$SID" --out "/ws/results/$SID.json"
  echo "leaked: $(ps aux | grep -cE '[g]z sim|[r]os2 launch')"
done

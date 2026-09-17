#!/bin/bash
# Send one semantic mission through the web UI and record the outcome for the paper.
#   scripts/run_semantic_mission.sh '{"cmd":"goto_object","target":"tv"}' [label] [timeout_s]
# Appends a row to ~/ammr_twin/eval/semantic_missions_<date>.csv and prints progress.
CMD="$1"; LABEL="${2:-mission}"; TMO="${3:-180}"
OUT=~/ammr_twin/eval; mkdir -p $OUT; CSV=$OUT/semantic_missions_$(date +%Y%m%d).csv
[ -f "$CSV" ] || echo "label,command,t_start,start_x,start_y,resolved,goal_x,goal_y,end_x,end_y,duration_s,arrive_d_m,result" > "$CSV"
# state = "x y|<newest status entries since mission start, joined by ' ## '>"
state() { curl -s -m 2 http://localhost:8080/api/state | python3 -c "import sys,json; d=json.load(sys.stdin); p=d['robot_pose']; L=[l if isinstance(l,str) else json.dumps(l) for l in d['status_log']]; i=max((k for k,l in enumerate(L) if l.startswith('command:')), default=len(L)-1); print(f\"{p[0]:.3f} {p[1]:.3f}|\" + ' ## '.join(L[i:]))" 2>/dev/null; }
S=$(state); SX=${S%% *}; SY=$(echo "$S" | cut -d' ' -f2 | cut -d'|' -f1)
T0=$(date +%s); TS=$(date +%H:%M:%S)
L0=$(wc -l < ~/bringup_logs/bridge.log)
curl -s -m 3 -X POST http://localhost:8080/api/command -d "$CMD" >/dev/null; echo "[$TS] $LABEL sent: $CMD  start=($SX,$SY)"
RES=""; RESOLVED=""; GX=""; GY=""
while [ $(( $(date +%s) - T0 )) -lt "$TMO" ]; do
  sleep 2; S=$(state); ST=${S#*|}
  R=$(echo "$ST" | grep -oE "resolved [0-9]+ goal\(s\): \[[^]]*\]"); [ -n "$R" ] && RESOLVED=$(echo "$R" | grep -oE "\[[^]]*\]")
  G=$(echo "$ST" | grep -oE "\(-?[0-9.]+,-?[0-9.]+\)" | head -1); [ -n "$G" ] && [ -z "$GX" ] && { GX=$(echo "$G" | tr -d '()' | cut -d, -f1); GY=$(echo "$G" | tr -d '()' | cut -d, -f2); }
  printf "  t=%3ds %s\n" $(( $(date +%s) - T0 )) "${ST##* ## }"
  case "$ST" in *"mission complete"*) RES=complete; break;; *REFUSED*|*refused*) RES=refused; break;; *"nav ended"*|*skip*|*unavailable*|*STOPPED*) RES=failed; break;; esac
done
[ -z "$RES" ] && RES=timeout
S=$(state); EX=${S%% *}; EY=$(echo "$S" | cut -d' ' -f2 | cut -d'|' -f1)
DUR=$(( $(date +%s) - T0 ))
D=$(tail -n +$((L0+1)) ~/bringup_logs/bridge.log | grep -oE "ARRIVED d=[0-9.]+" | tail -1 | grep -oE "[0-9.]+$")
echo "$LABEL,\"$CMD\",$TS,$SX,$SY,\"$RESOLVED\",$GX,$GY,$EX,$EY,$DUR,${D:-},$RES" >> "$CSV"
echo "==> $LABEL: $RES  resolved=$RESOLVED goal=($GX,$GY) end=($EX,$EY) ${DUR}s arrive_d=${D:-n/a}  (csv: $CSV)"

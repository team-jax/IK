#!/bin/bash
# Whole-repo test in one command (run it from anywhere):
#
#   ./run_tests.sh              quick: static checks, motion2 checks, motion1 smoke tests   (about 3 min)
#   ./run_tests.sh --full       also the 100-seed delivery and 50-seed stacking success rates,
#                               the reach map and two videos                                (about 12 min more)
#   ./run_tests.sh --viewers    also keeps each MuJoCo viewer open for 9 s (windows pop up; the screen must
#                               be awake: with the display off or locked GLFW finds no monitor and the viewer segfaults)
#
# A step passes only if it exits 0 AND its output has the expected text: an exit code alone once let
# a nonsense tune_grasp table through. No display or webcam is needed unless --viewers is given.
export PYTHONDONTWRITEBYTECODE=1
cd "$(dirname "$0")" || exit 1
P="$PWD/venv/bin/python"
FULL=0; VIEWERS=0
for arg in "$@"; do
  case $arg in
    --full) FULL=1 ;;
    --viewers) VIEWERS=1 ;;
    *) echo "usage: $0 [--full] [--viewers]"; exit 2 ;;
  esac
done
LOGS=$(mktemp -d); PASS=0; FAIL=0

tracked() {   # tracked files with uncommitted changes, in both repositories
  { git status --short; [ -d motion1/.git ] && git -C motion1 status --short | sed 's|^|motion1: |'; } 2>/dev/null \
    | grep -v '^??' | grep -v '^motion1: ??' | grep -v '\.omc\|\.claude/'
}
BEFORE=$(tracked)

ok()  { echo "PASS  $1"; PASS=$((PASS + 1)); }
bad() { echo "FAIL  $1"; FAIL=$((FAIL + 1)); [ -n "$2" ] && tail -n 5 "$2" | sed 's/^/          /'; }
step() {      # step <name> <regex the output must contain, or ''> <command...>
  local name=$1 expect=$2; shift 2
  "$@" > "$LOGS/$name.log" 2>&1; local rc=$?
  if [ $rc -ne 0 ]; then bad "$name (exit $rc)" "$LOGS/$name.log"
  elif [ -n "$expect" ] && ! grep -Eq -- "$expect" "$LOGS/$name.log"; then bad "$name (output lacks: $expect)" "$LOGS/$name.log"
  else ok "$name"; fi
}
alive() {     # alive <name> <process pattern> <command...>: the viewer must still be open after 9 s
  local name=$1 pattern=$2; shift 2
  "$@" > "$LOGS/alive_$name.log" 2>&1 &
  local pid=$!
  sleep 9
  if kill -0 $pid 2>/dev/null; then
    ok "viewer stays open: $name"; kill $pid 2>/dev/null; wait $pid 2>/dev/null
  else
    wait $pid; local rc=$?
    bad "viewer exited early: $name (exit $rc)" "$LOGS/alive_$name.log"
    [ $rc = 139 ] && echo "          (exit 139 = segfault: MuJoCo/GLFW crashes when the display is off or locked; wake the screen and rerun)"
  fi
  pkill -f "$pattern" 2>/dev/null; sleep 1
}
delivered() { # delivered <log>: the N in the last "N delivered" line
  tail -n 1 "$1" | sed -nE 's/^ *([0-9]+) delivered.*/\1/p'
}

echo "== static"
step py_syntax "files parse" $P -c "
import ast, glob
files = glob.glob('motion2/*.py') + glob.glob('motion2/arm/*.py') + glob.glob('motion1/scripts/*.py') + glob.glob('motion1/src/humanoid_grasp_rl/*.py') + glob.glob('*.py')
[ast.parse(open(f, encoding='utf-8').read(), f) for f in files]
print(len(files), 'files parse')"
step sh_syntax "" bash -c 'for f in motion2/*.sh run_tests.sh; do bash -n "$f" || exit 1; done'
step venv_tools "pip [0-9]" bash -c "venv/bin/pip --version && venv/bin/mjpython -c 'print(1)'"
step docs "0 unresolved" $P check_docs.py

echo "== motion2"
GEN_BEFORE=$(cat motion2/arm/arm.xml motion2/arm/arm.urdf motion2/arm/meshes/* | cksum)
step m2_generate "wrote" $P motion2/arm/make_arm_urdf.py
[ "$GEN_BEFORE" = "$(cat motion2/arm/arm.xml motion2/arm/arm.urdf motion2/arm/meshes/* | cksum)" ] \
  && ok "generator output == the model files in the repo" || bad "the generator changed the model files (regenerate and commit them)"
step m2_ik "position error: +0.00 mm" $P motion2/arm/ik.py
step m2_ik_target "position error: +0.00 mm" $P motion2/arm/ik.py --target 0.3 0.2 0.5
step m2_ik_unreachable "position error" $P motion2/arm/ik.py --target 2.0 2.0 2.0
step m2_grasp "lifted 9 /9" $P motion2/check_grasp.py
step m2_sender "sender: ok" $P motion2/check_sender.py
step m2_teleop "teleop: 3/3 scenarios passed" $P motion2/check_teleop.py
step m2_dashboard "dashboard: ok" $P motion2/check_dashboard.py

if [ -d motion1/scripts ]; then
  echo "== motion1"
  step m1_smoke "smoke test passed" $P motion1/scripts/smoke_test.py
  step m1_tune "g = 0\.86" $P motion1/scripts/tune_grasp.py
  grep -q "^ 0.00 .*89.7mm" "$LOGS/m1_tune.log" && ok "tune_grasp: pad gap 89.7 mm at g=0 (env.py says 90)" \
    || bad "tune_grasp: pad gap at g=0 is not about 90 mm" "$LOGS/m1_tune.log"
  step m1_layouts "layouts at randomization" $P motion1/scripts/layouts.py --scale 1.0 --out "$LOGS/layouts.json"
  step m1_replay "3 delivered" $P motion1/scripts/classify_failures.py --layouts "$LOGS/layouts.json" --seeds 3
  step m1_render "scene.png" $P motion1/scripts/render_frame.py

  if [ $FULL = 1 ]; then
    echo "== motion1, full"
    $P motion1/scripts/classify_failures.py --scales 1.0 --seeds 100 > "$LOGS/m1_delivery100.log" 2>&1 & DELIVERY=$!
    $P motion1/scripts/classify_failures.py --stack --scales 1.0 --seeds 50 > "$LOGS/m1_stack50.log" 2>&1 & STACK=$!
    cp motion1/src/humanoid_grasp_rl/reachmap.npz "$LOGS/reachmap.bak"
    step m1_reachmap "둘다 [0-9]+" $P motion1/scripts/build_reachmap.py
    cmp -s motion1/src/humanoid_grasp_rl/reachmap.npz "$LOGS/reachmap.bak" \
      && ok "rebuilt reach map == the one in the repo" || bad "the rebuilt reach map differs from the committed one"
    wait $DELIVERY; n=$(delivered "$LOGS/m1_delivery100.log")
    [ "${n:-0}" -ge 100 ] && ok "delivery, seeds 0-99: $n/100 delivered" || bad "delivery, seeds 0-99: ${n:-0}/100 delivered (README says 100)" "$LOGS/m1_delivery100.log"
    grep -q " False " "$LOGS/m1_delivery100.log" && bad "a delivered cube is marked picked=False" || ok "picked flag consistent with delivered"
    wait $STACK; n=$(delivered "$LOGS/m1_stack50.log")
    [ "${n:-0}" -ge 46 ] && ok "stacking, seeds 0-49: $n/50 delivered" || bad "stacking, seeds 0-49: ${n:-0}/50 delivered (README says 46)" "$LOGS/m1_stack50.log"
    step m1_video_delivery "written to .*1920, 1080" $P motion1/scripts/record_delivery.py --seed 3 --randomization-scale 1.0 --output "$LOGS/delivery.mp4"
    step m1_video_stack "written to .*1920, 1080" $P motion1/scripts/record_delivery.py --stack --seed 3 --randomization-scale 1.0 --output "$LOGS/stack.mp4"
  fi
fi

if [ $VIEWERS = 1 ]; then
  echo "== viewers (windows pop up)"
  VIEW=venv/bin/mjpython; [ "$(uname)" = Darwin ] || VIEW=venv/bin/python
  alive receiver "mujoco_receiver.py" motion2/run_receiver.sh
  alive ik_view "arm/ik.py" $VIEW motion2/arm/ik.py --view
  alive test_py "motion2/test.py" $VIEW motion2/test.py
  [ -d motion1/scripts ] && alive watch_grasp "watch_grasp.py" $VIEW motion1/scripts/watch_grasp.py --seed 3
fi

echo "== repository"
AFTER=$(tracked)
[ "$BEFORE" = "$AFTER" ] && ok "the tests left every tracked file as it was" || bad "the tests changed tracked files: $(diff <(echo "$BEFORE") <(echo "$AFTER") | tr '\n' ' ')"
echo
echo "$PASS passed, $FAIL failed   (logs in $LOGS)"
[ $FAIL = 0 ]

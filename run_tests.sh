#!/bin/bash
# Whole-repo test in one command (run it from anywhere):
#
#   ./run_tests.sh              quick: static checks, motion2 checks, motion1 smoke tests   (about 1 min)
#   ./run_tests.sh --full       also the 100-seed delivery and 50-seed stacking success rates,
#                               the reach map and two videos                                (about 3 min more)
#   ./run_tests.sh --viewers    also keeps each MuJoCo viewer open for 9 s (windows pop up; the screen must
#                               be awake and unlocked: with the display off GLFW finds no monitor and the viewer
#                               segfaults, and with the console locked mjpython itself can hang)
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
delivered() { # delivered <log>: the N of "N delivered" in the last line, wherever it stands: classify_failures
  # sorts the tally by count, so "39 placed short,  10 delivered" once read as 0
  tail -n 1 "$1" | sed -nE 's/^(.*[^0-9])?([0-9]+) delivered.*/\2/p'
}

echo "== static"
step py_syntax "files parse" $P -c "
import ast, glob
required = ['motion2/src/*.py', 'motion2/scripts/*.py', 'motion2/arm/*.py', '*.py']
optional = ['motion1/scripts/*.py', 'motion1/src/humanoid_grasp_rl/*.py']    # motion1 may be absent: motion2 runs without it
for pattern in required:
    assert glob.glob(pattern), 'no files match ' + pattern                   # a stale glob must not pass silently
files = [f for pattern in required + optional for f in glob.glob(pattern)]
[ast.parse(open(f, encoding='utf-8').read(), f) for f in files]
print(len(files), 'files parse')"
step sh_syntax "" bash -c 'for f in motion2/scripts/*.sh run_tests.sh; do bash -n "$f" || exit 1; done'
step venv_tools "pip [0-9]" $P -c "
import glob, os, subprocess, sys
print(subprocess.run(['venv/bin/pip', '--version'], capture_output=True, text=True, check=True).stdout.strip())
bad = []                                    # a renamed folder leaves stale shebangs; running mjpython here would need a live screen
for f in glob.glob('venv/bin/*'):
    if os.path.isfile(f):
        head = open(f, 'rb').readline(300)
        if head.startswith(b'#!') and not os.access(head[2:].split()[0].decode(), os.X_OK):
            bad.append(f + ' -> ' + head[2:].split()[0].decode())
print('venv/bin scripts with a missing interpreter:', bad or 'none')
sys.exit(1 if bad else 0)"
step docs "0 unresolved" $P check_docs.py

echo "== motion2"
GEN_BEFORE=$(cat motion2/arm/arm.xml motion2/arm/arm.urdf motion2/arm/meshes/* | cksum)
step m2_generate "wrote" $P motion2/arm/make_arm_urdf.py
[ "$GEN_BEFORE" = "$(cat motion2/arm/arm.xml motion2/arm/arm.urdf motion2/arm/meshes/* | cksum)" ] \
  && ok "generator output == the model files in the repo" || bad "the generator changed the model files (regenerate and commit them)"
step m2_ik "position error: +0.00 mm" $P motion2/arm/ik.py
step m2_ik_target "position error: +0.00 mm" $P motion2/arm/ik.py --target 0.3 0.2 0.5
step m2_ik_unreachable "position error" $P motion2/arm/ik.py --target 2.0 2.0 2.0
step m2_grasp "lifted 9 /9" $P motion2/scripts/check_grasp.py
step m2_sender "sender: ok" $P motion2/scripts/check_sender.py
step m2_teleop "teleop: 3/3 scenarios passed" $P motion2/scripts/check_teleop.py
step m2_dashboard "dashboard: ok" $P motion2/scripts/check_dashboard.py

if [ -d motion1/scripts ]; then
  echo "== motion1"
  step m1_smoke "smoke test passed" $P motion1/scripts/smoke_test.py
  step m1_tune "g = 0\.86" $P motion1/scripts/tune_grasp.py
  grep -q "^ 0.00 .*89.7mm" "$LOGS/m1_tune.log" && ok "tune_grasp: pad gap 89.7 mm at g=0 (env.py says 90)" \
    || bad "tune_grasp: pad gap at g=0 is not about 90 mm" "$LOGS/m1_tune.log"
  step m1_layouts "layouts at randomization" $P motion1/scripts/layouts.py --scale 1.0 --out "$LOGS/layouts.json"
  # At the measured pad friction (0.4) the second layout's cube slips out in the carry: 2 of 3.
  step m1_replay "[23] delivered" $P motion1/scripts/classify_failures.py --layouts "$LOGS/layouts.json" --seeds 3
  step m1_render "scene.png" $P motion1/scripts/render_frame.py
  # The real setup (shoulder axis 1212.5 mm, table 735 mm above the floor, the table's front edge 22.5 cm
  # in front of the shoulder axis, the robot turned -90 degrees so its FRONT side faces it) builds that
  # way on its own, current reach map (a stale one warns, made an error here). No success rate: the
  # hand there cannot carry a cube (motion1/STATUS.md).
  step m1_real_setup "real setup ok" env JAX_SETUP=real $P -W error::UserWarning -c "
import sys; sys.path.insert(0, 'motion1/src')
import humanoid_grasp_rl.env as E
from humanoid_grasp_rl import JaxDeliveryEnv
env = JaxDeliveryEnv(); env.reset(seed=0)
t, d, m = env.table_geom_id, env.data, env.model
top, edge = d.geom_xpos[t][2] + m.geom_size[t][2], d.geom_xpos[t][0] + m.geom_size[t][0]
shoulder = d.xanchor[env.arm_joint_ids[0]][2]
assert abs(shoulder - 1.2125) < 1e-9 and abs(top - 0.735) < 1e-9, (shoulder, top)
sx = d.xpos[env.model.body('base_link').id][0]
assert abs(sx - E.SHOULDER_X) < 1e-9 and abs(E.SHOULDER_X - edge - 0.225) < 1e-9, (sx, edge)
assert abs(d.xquat[env.model.body('column').id][3] + 0.5 ** 0.5) < 1e-9 and len(E.REACH_CELLS) > 0
print(f'real setup ok: shoulder axis {shoulder:.4f} m, table {top:.4f} m, {len(E.REACH_CELLS)} reachable cells')"
  # Real coordinates -> sim world (realcoords.py): loads without MuJoCo, reproduces the spec's worked
  # example, and its constants match env.py's real setup and the model's shoulder roll joint.
  step m1_realcoords "realcoords ok" env JAX_SETUP=real $P -c "
import sys; sys.path.insert(0, 'motion1/src/humanoid_grasp_rl')
import realcoords as RC
assert not {'mujoco', 'gymnasium'} & set(sys.modules), 'realcoords pulled in MuJoCo'
import numpy as np
assert np.allclose(RC.to_sim((0.10, -0.30, -0.4725), origin=(0.34, 0.0, 0.85)), (0.04, -0.10, 0.3775), atol=1e-12)
sys.path.insert(0, 'motion1/src')
import humanoid_grasp_rl.env as E
from humanoid_grasp_rl import JaxDeliveryEnv
env = JaxDeliveryEnv(); env.reset(seed=0)
roll = env.data.xanchor[env.arm_joint_ids[0]]
assert np.allclose(roll, (*RC.ORIGIN_XY, RC.TABLE_Z + RC.MAP_D), atol=1e-9), roll
s = E.SETUPS['real']
assert (RC.TABLE_Z, RC.MAP_D, RC.CUBE_SIZE) == (s['table_z'], s['d'], s['cube_size'])
assert RC.real_to_sim(0.10, -0.30, 0.4775)['reachable'] and not RC.real_to_sim(0.0, -0.10, 0.4775)['reachable']
print('realcoords ok')"
  # Cube yaw on the real setup: realcoords' fold and hand turns are env's, its per-yaw map is current,
  # a turned cube the env can serve is accepted with the turn the map names, an unreachable one is
  # refused, and run_real.py delivers the yaw-0 example and writes its trajectory.
  step m1_real_yaw "real yaw ok" env JAX_SETUP=real $P -c "
import sys; sys.path.insert(0, 'motion1/src/humanoid_grasp_rl'); sys.path.insert(0, 'motion1/src')
import numpy as np, realcoords as RC
import humanoid_grasp_rl.env as E
from humanoid_grasp_rl import JaxDeliveryEnv
for v in (-180, -95, -45, -44.9, 0, 20, 44.9, 45, 90, 135, 200):
    assert RC.fold_yaw(v) == E.fold_yaw(v) and RC.hand_turns(v) == E.hand_turns(v), v
with np.load(RC.YAW_REACH_FILE) as m:
    assert np.allclose(m['signature'], E.map_signature()), 'reachmap_real_yaw.npz is stale: build_reachmap.py --yaws'
p = RC.plan((0.10, -0.30, 0.0), (0.05, -0.27), 0.4775)
assert p['reachable'] and p['hand_yaw'] == 0.0 and RC.plan((0.10, -0.30, 90.0), (0.05, -0.27), 0.4775)['yaw'] == 0.0
env = JaxDeliveryEnv()
p = RC.plan((0.03, -0.30, -20.0), (0.10, -0.30), 0.4775)
env.reset(seed=0, options={'objects': [[*p['cube'][:2], p['yaw']]], 'targets': [p['target']]})
assert abs(np.degrees(env._hand_yaws[0]) + 20.0) < 1e-9
q = env.data.qpos[env.model.jnt_qposadr[env.object_joint_ids[0]]:][:7]
assert abs(2 * np.degrees(np.arctan2(q[6], q[3])) + 20.0) < 1e-9
try:
    env.reset(seed=0, options={'objects': [[0.5, 0.5, 0.0]], 'targets': [p['target']]}); raise SystemExit('accepted')
except ValueError: pass
print('real yaw ok')"
  step m1_run_real "sim: delivered" env JAX_SETUP=real $P motion1/scripts/run_real.py --cube 0.10 -0.30 0 --target 0.05 -0.27 --out "$LOGS/trajectory.json"

  if [ $FULL = 1 ]; then
    echo "== motion1, full"
    $P motion1/scripts/classify_failures.py --scales 1.0 --seeds 100 > "$LOGS/m1_delivery100.log" 2>&1 & DELIVERY=$!
    $P motion1/scripts/classify_failures.py --stack --scales 1.0 --seeds 50 > "$LOGS/m1_stack50.log" 2>&1 & STACK=$!
    cp motion1/src/humanoid_grasp_rl/reachmap.npz "$LOGS/reachmap.bak"
    step m1_reachmap "둘다 [0-9]+" $P motion1/scripts/build_reachmap.py
    cmp -s motion1/src/humanoid_grasp_rl/reachmap.npz "$LOGS/reachmap.bak" \
      && ok "rebuilt reach map == the one in the repo" || bad "the rebuilt reach map differs from the committed one"
    wait $DELIVERY; n=$(delivered "$LOGS/m1_delivery100.log")
    [ "${n:-0}" -ge 68 ] && ok "delivery, seeds 0-99: $n/100 delivered" || bad "delivery, seeds 0-99: ${n:-0}/100 delivered (README says 68)" "$LOGS/m1_delivery100.log"
    grep -q " False " "$LOGS/m1_delivery100.log" && bad "a delivered cube is marked picked=False" || ok "picked flag consistent with delivered"
    wait $STACK; n=$(delivered "$LOGS/m1_stack50.log")
    [ "${n:-0}" -ge 39 ] && ok "stacking, seeds 0-49: $n/50 delivered" || bad "stacking, seeds 0-49: ${n:-0}/50 delivered (README says 39)" "$LOGS/m1_stack50.log"
    step m1_video_delivery "written to .*1920, 1080" $P motion1/scripts/record_delivery.py --seed 3 --randomization-scale 1.0 --output "$LOGS/delivery.mp4"
    step m1_video_stack "written to .*1920, 1080" $P motion1/scripts/record_delivery.py --stack --seed 3 --randomization-scale 1.0 --output "$LOGS/stack.mp4"
  fi
fi

if [ $VIEWERS = 1 ]; then
  echo "== viewers (windows pop up)"
  VIEW=venv/bin/mjpython; [ "$(uname)" = Darwin ] || VIEW=venv/bin/python
  alive receiver "mujoco_receiver.py" motion2/scripts/run_receiver.sh
  alive ik_view "arm/ik.py" $VIEW motion2/arm/ik.py --view
  alive viewer_check "check_viewer.py" $VIEW motion2/scripts/check_viewer.py
  [ -d motion1/scripts ] && alive watch_grasp "watch_grasp.py" $VIEW motion1/scripts/watch_grasp.py --seed 3
fi

echo "== repository"
AFTER=$(tracked)
[ "$BEFORE" = "$AFTER" ] && ok "the tests left every tracked file as it was" || bad "the tests changed tracked files: $(diff <(echo "$BEFORE") <(echo "$AFTER") | tr '\n' ' ')"
echo
echo "$PASS passed, $FAIL failed   (logs in $LOGS)"
[ $FAIL = 0 ]

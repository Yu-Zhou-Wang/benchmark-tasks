This task opens a door that is still held by a security chain. A successful
robot supports the door to remove the load, moves the chain fitting along the
slot to its wide opening, withdraws the retaining head, and then opens the door.
The plate, head, chain and moving panel all collide in simulation.

The scene uses a compact plywood door in a fixed table fixture so two YAM arms
can reach the hardware. The latch is hand sized; the panel is 360 × 310 × 18 mm.
This is a bench test of operating household door hardware. It does not claim
full-size door dynamics, forced-entry resistance, material damage, or locomotion.

Run from an activated YAM Isaac Lab Arena installation. The tested environment
is YAM commit `0059d02`, IsaacLab-Arena `release/0.3.1`, Isaac Sim 6.1 and Python
3.12, using a single physical GPU 0. On the prepared workstation, prefix each
simulation command below with its approved `run_on_gpu.sh 0` launcher. The
launcher is an environment-level GPU selection tool, not a task dependency.

```bash
# Robot-only full demonstration. A fresh output directory is required.
python tasks/door-security-chain/demo.py --variant full --out runs/door-full

# Passive reset and stability check, with the arms holding READY.
python tasks/door-security-chain/demo.py --variant idle --no-record --out runs/door-idle

# Physical controls, also using only robot Actions.
python tasks/door-security-chain/demo.py --variant pull_latched --no-record --out runs/door-pull-latched
python tasks/door-security-chain/demo.py --variant no_slide --no-record --out runs/door-no-slide
python tasks/door-security-chain/demo.py --variant no_slack --no-record --out runs/door-no-slack

# Rebuild the included original CAD if desired; simulation uses the checked-in asset.
python tasks/door-security-chain/build_assets.py

# CPU rubric counterexamples.
python tasks/door-security-chain/test_scoring.py
```

The demonstration sends ordinary 14-value YAM joint-target Actions. Its
controller uses fixed waypoints derived from the specified starting layout,
elapsed action index and the robot's URDF inverse kinematics. It does not read
object transforms, simulator contacts, segmentation or scorer output to correct
its actions. The separate diagnostics log actual geometry after each action;
those measurements are never passed to the controller. Rendering can be skipped
for development with `--no-record`, which leaves physics and actions unchanged.

The source files and asset hashes, all robot actions, per-step measurements,
result, initial/final camera images and H.264 camera videos are written to the
output directory. Failed trials are retained. A result with `success: false`
is not a full-score demonstration even if intermediate progress is visible.

The scorer measures the head in the moving door frame. Partial credit requires
either alignment with the wide exit while the head is captive, or the whole
retaining head being at least 4 mm outside the housing envelope. Full score
also requires at least 50° of opening, low door and fitting speeds, intact
chain continuity, and clear grippers for 1.5 seconds. Holding the separate door
handle is allowed. Neither elapsed time alone nor an unlock flag can earn
success. Every task callback is observational.

The chain uses rigid open-centre ring colliders connected by permanent pin
joints with alternating bending axes. Adjacent ring collisions are filtered
because the pin joints represent their interlinking. Other ring collisions,
door contact, fitting contact and the slotted plate remain physical. The
starting chain follows the free edge of the door; a straight initial chord
would incorrectly intersect the panel. Native velocity bounds and light joint
damping keep this small-link approximation numerically stable. No fitting is
welded, snapped, teleported or driven by an object controller.

See [asset provenance](assets/PROVENANCE.md) for primary manufacturer references,
dimension choices and modeling limitations.

The closest screened public tasks are [open-barred-drawer #215](https://github.com/robocurve/benchmark-tasks/pull/215)
and [cylinder-chain-secure #1929](https://github.com/robocurve/benchmark-tasks/pull/1929).
The drawer's rigid bar slides out of sleeves; the cylinder task routes a chain
and drops a link onto a hook. This task starts with a loaded chain, a captive
retaining head, and a moving door. Closing the door changes the available chain
length; only the wide opening permits physical withdrawal. The public screen
covered 2,341 PR titles, bodies, labels and states on 2026-11-16, with focused
source inspection of these two comparisons. The private task board required
sign-in and was not independently checked.

Dream-1 evaluation, model video and task-board media ID are pending. No remote
submission or media upload has been performed. The code and local evidence are
preparation for review, not a claim of acceptance.

The physics timestep is 1/240 s; control is 20 Hz. The declared limit is 1,800
actions (90 simulated seconds). A preliminary full demo reached 1.0 in 866
actions (43.3 seconds), followed by two 200-action zero-score resets. Final
recorded-run measurements and physical controls are summarized in
`VALIDATION.md`. Rendering time is hardware dependent; the local unrecorded
full run plus reset checks took about four minutes on an RTX 5090.

Final Dream evaluation command, once credentials are available (not run during
preparation; prefix with the approved GPU-0 launcher on the prepared machine):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/door-security-chain/task.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Release the door security chain and open the door at least 50 degrees. Finish with the chain fitting free of the slotted plate and both grippers clear of the fitting. You may keep holding the door handle." \
  --scorer task_score --max-steps 1800 --seed 0 --store-frames \
  --no-prompt --log-dir runs/door-security-chain-dream-final
```

`task.json` intentionally leaves `media` empty. Replace it with the real media
ID only after an authorized upload. No model score, model video or media ID is
inferred from the scripted result.

`chord_allowance_m` is chain length minus straight-line endpoint distance. It
is a diagnostic, not a tension measurement: a taut chain wrapping the door edge
has nonzero chord allowance. Retention is established by measured limited door
motion under the hinge spring and robot pulls, plus the extraction controls.

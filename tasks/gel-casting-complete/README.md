# Secure and dock a gel-casting cassette

The robot closes the two green pressure cams around pre-aligned glass plates, lifts the cassette from a visible alignment cradle, and seats it on the separate casting stand. One gripper holds the stand's spring lever open while the other docks the cassette. Both hands then withdraw, leaving the top pressure shoe on the tall rear plate and the cassette upright and stable.

This is a hardware-preparation task. It does not include liquid handling, polymerization, comb insertion, or electrophoresis. See `SCOPE_REVIEW.md` for the comparison with all 18 currently public accepted tasks.

**Verified scripted demonstration:** the recorded full run succeeds with score **1.0** in **1,413 control steps / 70.65 simulated seconds**. All three H.264 videos decode completely: 707 frames each at 10 fps and 960 × 528. The current sources and generated assets are unchanged across the run. Fresh retention and staging benches, all seven physical seating checks, the corrected approach check and all 36 scoring tests pass. The original straight-extraction diagnostic remains false because the existing angled lever obstructs that path; only the offset assembly approach is verified. Packaged evidence in `demo/` comes exclusively from `runs/refined-final`. This establishes feasibility for the stated deterministic reset, not generalization, manufacturer fit or a liquid seal. Strict bimanual necessity remains untested.

**Dream-1 evaluation:** one recorded trial of `dreamlabs/dream-1-beta` scored **0.0** and reached the **2,400-step limit**. The evaluator completed without an errored trial; its execution status `success` does not mean task success. Configuration, unchanged source fingerprints and verification results for all three model-run videos are retained in `model_run/`. The transcript records repeated recovered session expirations; their effect on task performance is not established. This is one measured trial, not an estimate of a general success rate.

**Submission media:** Task-board media ID `M-EVJ8Z6` contains the model-run video `model_run/video/~f1~scene-0-e0~top_cam.mp4`, scripted demonstration `demo/top_cam.mp4`, and setup screenshot `demo/setup.png`. The same ID is recorded in `task.json`.

The six MP4 recordings are excluded from the GitHub code archive to meet its 25 MB limit. The model-run and scripted-demo top-camera videos remain attached through media ID `M-EVJ8Z6`; the setup screenshot is also kept in the repository. All six original recordings are preserved locally and in Git commit `1bb885c3a799aec39a2204cca0586e008d629802`. Their sizes and SHA-256 checksums are retained under `omitted_recordings` in `PACKAGE_CONTENTS.json` and `model_run/EVIDENCE_CONTENTS.json`. Source code, simulation assets, evaluator logs, measured traces and validation reports remain in this submission. `package_task.py` continues to build the full local development archive when supplied with the original recordings.

## Files

| File | Purpose |
|---|---|
| `task.py` | Scene setup, implicit two-well cam springs, state monitor, success and partial scoring |
| `scenekit.py` | Task-local adapter for custom USD rigid bodies, articulations, measured joint states and step/reset callbacks |
| `build_task_assets.py` | Creates separate manipulation assets from the visual source assets without editing those sources |
| `demo.py` | Cartesian robot controller, measured-state feedback, camera recordings and result logs |
| `test_scoring.py` | Synthetic-state checks of scoring counterexamples; these do not test physical feasibility |
| `test_retention.py` | Separate physics bench for glass retention and an open-cam negative control; no robot |
| `test_staging.py` | Five-second idle jig check and nominal collision-clearance sweep; no robot or simulated lift |
| `test_seating.py` | Physical seating bench with closed/open clip controls and identical disturbances |
| `source_fingerprint.py` | SHA-256 fingerprints of the executable sources and generated assets used by each run |
| `check_cassette_path.py`, `contact_geometry_audit.py` | CPU check of the corrected cassette approach against enabled stand collision shapes |
| `geometry.py`, `assets/` | Reference-matched source geometry and its dimensions |
| `task_assets/` | Generated frame, separate plates, stand with installed sealing pad, and alignment cradle |
| `runtime_environment.json` | Installed package versions, source revisions and verified physics/control configuration |
| `demo/` | Verified run results, source manifest, measured gripper telemetry, close-up and physics-rate settling evidence; video attached through the task board |
| `model_run/` | Dream-1 official evaluator JSON, explicit configuration, source manifest, result summary, video checksums and stage images |

## Scene and reset

Use the `station` scene with zero object jitter. World coordinates are metres: x forward from the arm bases, y left, z up. The table top is at z = 0.7475 m. Both apparatus assemblies are rotated 180 degrees around z so their fronts face the robot.

- Initial frame origin: (0.300, 0.000, 0.7385). The frame feet meet the table; the glass bottoms stand 3 mm above it on the cradle pad.
- Stand origin: (0.160, 0.000, 0.7475). Its pad is installed as part of the stand's rigid body.
- Rear plate: 101 × 82 × 3 mm; front plate: 101 × 73 × 3 mm. The nominal 3 mm glass thickness is a modeling choice. Bonded spacers create a 1 mm cavity.
- The glass plates remain independent, dynamic bodies. They start aligned inside the frame, supported by the visible fixed cradle; they are not welded to the frame.
- Initial side-cam angles are -100 degrees and +100 degrees. The stand lever starts closed at zero degrees.
- Reset restores this prepared arrangement and clears the score history. It is not a robot-executed disassembly procedure. Nothing is consumed.

The visible alignment cradle has a 3 mm leveling pad and two front fingers reaching frame-local z = 50 mm, with 0.05 mm nominal face clearance to the short plate. Shallow 2.5 mm collars surround the two frame feet with 0.05 mm clearance. The collars brace frame translation and yaw while its rear rails support the rear plate; the front fingers support the loose short plate. The collars end below the lower hinge ears, and all supports are open upward. The fixture adds no attachment to the frame or glass and cannot accompany the cassette during transfer. The fixture calls for at least 45 mm of upward extraction; the controller lifts 140 mm over 4 seconds before crossing above the taller casting stand.

## Mechanics and fidelity

The task changes the visual kit's always-closing side-cam drives into a nominal bistable torsion model. Each cam uses an implicit spring drive with a weak free-travel detent: 0.015 N m/rad stiffness and 0.00035 N m s/rad damping. In the closed well, these gains rise linearly to 0.30 N m/rad and 0.003 N m s/rad over oriented angle -10 to 0 degrees, then remain at those values. This preserves final contact preload without imposing that resistance throughout open-cam travel. The torque limit is 0.08 N m. The generated base asset has zero cam drive gains; `task.py` supplies the runtime gains. Physics resolves spring and contact forces through its implicit solver; the task does not apply an explicit effort updated at 20 Hz.

Only the measured mechanical angle selects the spring well and gains. For oriented angle `s*q`, where `s = +1` for the left cam and `s = -1` for the right, the target is -100 degrees below the -49-degree threshold and +2 degrees otherwise. The right joint's actual target has the opposite sign. Target and gain selection run at the 20 Hz control rate through `cam_drive_parameters`. They do not inspect the robot, plate location, or score to trigger a latch, and never write an object pose after reset.

The cam's contact with the glass resists final closure and establishes preload. This two-well spring, its friction, and the resulting clamping preload are research approximations, not measured manufacturer properties or a claim of an exact commercial detent mechanism.

Only the two frame side-post collision shapes have a nominal torsional contact patch: 4 mm radius with a 3.2 mm minimum, within their 10 mm-wide gripping faces. This represents a finite-area gripper-jaw contact that can resist twisting. It does not add a grasp constraint or change the glass, cam, rail or stand contact parameters. The patch is a modeling assumption, not a measured manufacturer contact property.

Task bodies and articulation roots explicitly request 64 position and 8 velocity solver iterations. Physics runs at 240 Hz, with 12 substeps per 20 Hz control command. The standalone benches use the same articulation iteration counts.

The top lever uses a nominal return spring: 0.045 N m/rad stiffness, 0.002 N m s/rad damping, and a 0.05 N m torque cap. Its rest target is +8 degrees, beyond the contact-limited seated angle, to create preload through physical contact. The estimated additional downward force is about 0.25 N at the nominal geometry; this is a designed approximation, not a measured commercial load rating. Opening the lever lifts its pressure shoe; one hand holds it open while the other docks the cassette. Whether one arm can exploit a fixture or another strategy remains untested.

Four clear rear supports are centred at stand-local y = ±38 mm and z = 19/81 mm, with 6 × 8 mm contact faces. Their front faces at x = −12.3 mm leave 0.3 mm nominal insertion clearance to the collidable rear frame ribs at x = −12 mm. Both visible ribs and their collision proxies span 88 mm, centred at y = 0 (ends at ±44 mm). They cover the support faces through y = ±41 mm while leaving the side-post gripping corridor clear. Low lateral guides surround the frame feet with 0.3 mm clearance and remain open upward. These support surfaces limit rocking and lateral displacement through collision contact. The full stand also contains the existing angled lever, which obstructs the nominal straight vertical cassette path even when open; insertion therefore needs a checked offset approach. Whole-assembly straight-up removal and manufacturer tolerances are not claimed.

Glass and rubber are rigid. There is no fluid, gasket compliance, glass fracture, plate bending, or verified watertight seal. Masses, centres of mass, bounding-box inertias, friction and detent properties are nominal. Two slender stand guides retain their rounded visual geometry but use native box collision proxies filling the 0.3 mm bevels. There are no fixed joints connecting plates to the frame or cassette to the stand.

## Run

From this task folder on the configured Brev machine:

```bash
source ~/IsaacLab-Arena/yam.env
python build_task_assets.py --source-assets assets --output task_assets
python test_scoring.py
python test_retention.py --seconds 2
python test_staging.py
python test_seating.py
python check_cassette_path.py --task-root . --out cassette_path_check.json
python demo.py --variant idle --out runs/idle --no-record
python demo.py --variant cams --out runs/cams
python demo.py --variant full --out runs/refined-final --max-steps 2400 --record-every 2 --physics-trace --close-view --width 960 --height 528
python analyze_settling.py runs/refined-final/physics_trace.json --report runs/refined-final/settling_check.json
```

The final two commands produce the designated new recording and check its release settling. Only verified results from `runs/refined-final/` may be copied into the package's `demo/` directory. The rollout also writes `run_manifest.json` and measured `gripper_trace.json`; packaging checks that source and asset bytes were unchanged during execution and still match the files being packaged.

`test_seating.py` retains a nonzero exit and an overall false report because the nominal straight-extraction diagnostic is obstructed. Its seven physical checks are evaluated separately alongside the passing corrected-path report and the full robot run. The CPU path checker uses SciPy, trimesh and USD from the configured environment. This combination does not erase or relabel the failed diagnostic.

The full controller operates at 20 Hz with a default limit of 2,400 control steps, corresponding to 120 seconds of simulated task time. The verified run took **376.96 seconds of wall time through the close-up**, including 16.99 seconds for startup/reset, 349.78 seconds in the controller loop and 9.32 seconds exporting video. The reset uses `seed=None` and zero object jitter. Default recording resolution is 640 × 352; use `--width` and `--height` to change it. `--no-record` suppresses video collection and its otherwise hidden renders. `--record-every 2` records at 10 fps while control remains at 20 Hz. `--physics-trace` records fresh poses, quaternions, joint angles and velocity buffers at 240 Hz after release. The controller writes results, source fingerprints, measured gripper telemetry, camera videos and final images. The additional `seating_closeup.png` changes only the camera: its recorded physics-step counts are both 17,076.

The standard environment entry point remains `yam_arena` with `-E task=task.py`, `-E object_jitter=0`, `-E control_hz=20`, and scorer `task_score`. `scenekit.py` hooks the task process locally; it does not edit the shared simulator installation. `demo.py` drives the embodiment directly because its feedback controller is not the simple waypoint `yam_scripted` policy.

## Dream-1 trial

The recorded policy run used Inspect Robots 1.0.0, inspect-robots-dream 0.1.0, dreamlabs-client 1.0.14 and yam-arena 0.1.0. The exact original command and before/after source hashes are in `model_run/run_manifest.json`; the official evaluator log is `model_run/eval/adhoc_91d53ebb.json`. The following command reproduces its settings from the task folder, writing a separate run directory:

```bash
source ~/IsaacLab-Arena/yam.env
inspect-robots run --config model_run/explicit-config.ini \
  --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 -E top_cam=calibrated \
  -E wrist_cam_mount=link -E headless=true \
  -P model=dreamlabs/dream-1-beta -P rows_per_request=48 \
  -P preroll_steps=32 -P dry_run=false -P record=false -P timeout_s=300 \
  --instruction "Close both green pressure cams to secure the aligned glass plates. Lift the cassette from its alignment cradle, hold the clear stand lever open, and place the cassette on the gray sealing pad. Release the lever onto the tall plate and withdraw both grippers, leaving the assembly upright and stable." \
  --scorer task_score --max-steps 2400 --seed 0 --epochs 1 \
  --no-prompt --grader none --fail-on-error 1 --store-frames \
  --no-rerun --no-rerun-save --log-dir runs/dream1-repeat/eval
inspect-robots video <completed-evaluator-log.json>
```

The explicit configuration file contains no overrides; all trial options are on the command line. Standard action guardrails remain enabled. The policy observes calibrated top and link-mounted left/right wrist cameras at 320 × 176, with 20 Hz control, 48-row chunks and a 32-step preroll. Hosted recording is disabled, while local frames are recorded. Seed 0 controls evaluation/reset and object jitter is zero; the hosted model is unseeded, so an identical command does not promise an identical policy trajectory.

The single trial `scene-0-e0` reached `task_score: 0.0`, terminated by `max_steps` after 2,400 steps (120 simulated seconds), and recorded zero errored trials. Official evaluator duration was **783.221109 seconds**; the measured runner wall time was **785.438133 seconds**. The robot moved, but the equipment remained unassembled. `model_run/result.json` separately records `execution_success: true` and `task_success: false`, and verifies the resolved configuration, unchanged executable sources/assets and all three fully decoded H.264 videos. Each video has **2,401 frames at 20 fps and 320 × 176**, including the initial frame. The original videos are named `model_run/video/~f1~scene-0-e0~{left_cam,right_cam,top_cam}.mp4`; they are excluded from the GitHub code archive as described above. Initial, middle and final images remain in `model_run/stages/`.

The official policy transcript contains 50 successfully returned 48-row chunks, requested from step 32 through step 2,384. One request succeeded on its first attempt; 49 required a second attempt after session-expiry recovery, with 50 distinct session IDs overall. The client adopted replacement sessions and the adapter retried; the evaluation continued without an errored trial. These renewals are retained as a run limitation, and are not established as the cause of the task failure. The original run directory retains raw frames and action records; the raw evaluator JSON in `model_run/` keeps their original locations.

## Scripted manipulation

1. Use one gripper at a time to push a pressure wing with closed fingers along its arc; repeat for the other side. The visible foot pockets brace the frame during this step. The downward wrist approaches at a TCP height offset of 33 mm above the cam pivot, keeping the fingers clear of the upper hinge ear and glass. Contact waypoints account for the jaw's projected width as the wing rotates. After closure, unload contact by retreating 20 mm horizontally along the measured cam normal before rising 70 mm and returning home. This prevents the withdrawing finger from lifting the frame by its hinge ear.
2. Grasp the secured frame, lift 140 mm over 4 seconds, and carry it above the entire stand with the carrying wrist tilted 30 degrees outward.
3. Approach the rear tab with the pressing wrist tilted 25 degrees backward and 30 degrees outward. Press near the hinge edge and verify a measured lever opening of at least 82 degrees before lowering; the additional pressing path aims toward -83 degrees. The lever's opening limit is -85 degrees.
4. Correct the carrying waypoint using the measured frame pose. Keep the cassette 2 mm forward in stand-local +x while lowering from 140 to 20 mm above its seated height, then blend the offset to zero by 8 mm. Pause at that height for 3 seconds of measured frame alignment, lower with pose feedback over 3 seconds, and allow 2 seconds to settle. The lever must remain at or beyond -82 degrees throughout; the carrying hand refuses to release unless the measured geometry is docked. Exact geometry checks cover the nominal approach at lever angles of -85 and -82 degrees; the nominal straight path is obstructed. Release first to a 20 mm jaw gap, then open to a nominal 93.9 mm while moving the TCP forward by half the additional gap (36.95 mm), avoiding the stand's rear panel. Let the top lever return.
5. Move both hands away and hold the final state for verification.

Commands go only to arm joint targets and gripper openings. The controller uses measured object poses to update contact waypoints; it does not teleport objects, directly set cam angles, create grasp attachments, or set success flags.

## Scoring as currently implemented

The task uses end-of-episode scoring (`SCORE_OVER = "end"`). Settling first requires one full second of pose history; full success must then hold continuously for an additional 3 seconds. The following thresholds are in `task.py` and are authoritative if they change during tuning.

An aligned cassette requires both plate-origin errors relative to the frame below 2 mm and plate/frame orientation differences below 4 degrees. Four sampled cavity gaps must each be 1.00 ± 0.25 mm, and four sampled plate-bottom heights must be within 0.5 mm of the intended sealing level. This rules out a visually similar but excessively wide cavity or plates floating above their intended bottom alignment.

- A left cam counts closed strictly between -1 and +4 degrees; a right cam counts closed strictly between -4 and +1 degrees. Each must first have been observed more than 60 degrees open during the trial.
- Transport history requires the frame origin to rise more than 25 mm above its initial height.
- Docking requires aligned plates and both closed cams, frame/stand horizontal origin error below 2.5 mm, vertical origin error below 0.6 mm, and their orientation difference below 3 degrees. Four rear-rib sample points must lie between -0.15 and +1.0 mm from their corresponding rear support faces, with lateral frame offset no greater than 0.7 mm. The actual glass-bottom samples must also be within 0.5 mm of the stand's gasket plane. These direct stand-relative checks supplement frame-origin alignment.
- The top clip counts closed within 6 degrees of zero only when its transformed pressure-pad bottom points lie within 0.5 mm of the tall plate's top and within the stated 3 mm centring tolerance across the plate thickness. These are geometric proximity checks, not measured clamping force. The stand must be within 5 degrees of upright.
- Stability uses measured poses for the frame, both plates and stand: each body's maximum pairwise position diameter must be at most 0.2 mm, and its normalized-quaternion geodesic diameter at most 0.2 degrees. Both cam angles and the clip angle must each span at most 0.5 degrees. Coarse raw-velocity guards reject any body at or above 50 mm/s or 2 rad/s; they guard against fast aliased motion rather than define settling.
- Hand clearance currently uses both tool-centre points: each must be more than 145 mm from the stand's local point (0, 0, 60 mm). This is a TCP-distance test, not an all-link contact or force test.
- If any frame, plate or stand origin falls more than 25 mm below the table top, an irreversible `part_fell_off_table` violation is recorded.

The pose candidate starts only while alignment, docking, closed clip, upright stand and hand clearance all pass. It needs 21 samples spanning a full second at 20 Hz, using the environment's actual control timestep. Its history then expands through the additional 3-second framework hold, capped at 4 seconds; it is not a succession of independent 1-second windows. Loss of a gate, missing or invalid readings, an excursion, or a coarse-velocity violation clears qualification. This rejects gradual accumulated drift as well as sudden motion. Reset clears the history. The rule measures settling within the stated pose bounds; it does not claim that backend velocity buffers read exactly zero.

| Outcome | Score |
|---|---:|
| A recorded fall or plate misalignment | 0 |
| Each cam closed after having been observed open, with plates aligned | 0.15 each; up to 0.30 |
| Cassette docked after the required opening and lift history | 0.60 |
| Docked, top clip closed, stand upright | 0.90 |
| Above conditions plus stable assembly and both hands clear for 3 seconds, with no violation | 1.00 through the success scorer |

`score()` returns at most 0.90; the framework reports 1.00 when `success()` completes its hold. A final pose alone cannot substitute for the observed opening and transport history. Conversely, the current scorer does not measure sealing pressure, clamping force, actual spring-tab contact, or continuous plate retention throughout transport. Those remain physical validation questions, not facts established by the numeric score.

## Validation results

The recorded full run in `demo/result.json` reports `success: true`, `task_score: 1.0`, `variant: full`, no controller failure and no task violation. Both cams were observed open, the cassette was lifted, and all final alignment, rear-registration, gasket, pressure-shoe, settling and hand-clearance checks pass. `demo/run_manifest.json` records unchanged executable-source and asset fingerprints; earlier failed or differently shaped runs are excluded.

| Current full-run measurement | Result |
|---|---:|
| Control steps / simulated time | 1,413 / 70.65 s |
| Measured wall time through close-up | 376.96 s |
| Final cavity gap samples | 0.878–0.998 mm |
| Rear-support clearances | 0.033–0.321 mm |
| Glass-to-gasket signed gaps | -0.046–+0.207 mm |
| Pressure-shoe signed gaps | -0.204–-0.173 mm |
| Largest measured pad gap at the TCP plane | 93.963369 mm; no samples above 94 mm |
| Independent final settling window | 4 s; 961 consecutive 240 Hz samples |
| Largest body position / orientation diameter | 0.377 µm / 0.000460 degrees |

Signed negative surface gaps are small overlaps in the rigid contact model, within the stated scorer tolerance; they are not measurements of gasket compression. The gripper result comes from 1,414 measured states, including reset, at 20 Hz. Raw finger positions and small solver-limit excursions remain in `demo/gripper_trace.json`; telemetry was not clipped to make the gap pass. The maximum gap is specific to the TCP plane of the tapered pads. All three videos decoded in full, and the close-up was captured without advancing physics. The settling report hashes its own physics trace and checks all four body poses and three mechanism angles; its sub-micrometre values are simulator measurements, not hardware accuracy claims. All 36 synthetic scoring tests pass.

Fresh retention and staging benches pass for the final 88 mm rib geometry, and all seven physical seating checks pass with unchanged source fingerprints. Under the identical 2.2 N upward pulses, the maximum upward excursion is **0.10144 mm with the spring-loaded clip** and **43.2829 mm with the lever held open**. Final closed-case glass alignment, gasket proximity and pressure-shoe proximity remain valid. The reports retain actual load durations, application points, support clearances and traces. These checks establish only their stated simulated conditions, not a general load rating or watertight seal. Previous full-width-rib results are preserved outside this package and are not assigned to the new asset hashes.

`seating_check.json` deliberately retains `passed: false`: its separate whole-stand straight-extraction check detects interference with the existing angled lever. Exact collision analysis confirms a real obstruction on that nominal path; it is not treated as a harmless bounding-box false positive. The new pads and guides are not the interfering shapes. `cassette_path_check.json` separately passes the corrected path at -85 and -82 degrees, checking 141 nominal cassette poses per angle at 1 mm height intervals with no penetration greater than the stated 0.05 mm test depth. The checker reads the controller's actual `dock_forward_offset` function and records matching source fingerprints. This discrete geometry result is not a continuous swept-path or dynamic guarantee; the recorded full run independently demonstrates the actual assembly motion for this reset. No straight-up removal claim is made, and neither the original failed check nor its source fingerprint is rewritten to obtain a passing report.

Each bench records executable-source and asset fingerprints before and after execution. The designated full recording is `runs/refined-final`; its `run_manifest.json` must show unchanged before/after fingerprints matching the final package. The settling report must hash that run's actual physics trace. Documentation is excluded from executable fingerprints so measured results can be documented after the run. Old or unselected success runs cannot satisfy the refined package checks.

One Dream-1 trial is recorded above. Multi-trial learned-policy performance, generalization, strict bimanual necessity, manufacturer fit, gasket compliance and liquid sealing are not established by this development work.
## Sources and code provenance

The physical assembly follows the [Bio-Rad Mini-PROTEAN Tetra manual](https://www.bio-rad.com/webroot/web/pdf/lsr/literature/10007296D.pdf); the [published plate specifications](https://www.bio-rad.com/sites/default/files/2024-05/Bulletin_5535.pdf) provide the plate widths and heights. The generated appearance is reference-informed original geometry, not manufacturer CAD.

The task-local custom-USD adapter is adapted from the accepted [lock-before-loading `scenekit.py`](https://github.com/robocurve/benchmark-tasks/blob/main/tasks/lock-before-loading/scenekit.py), with changes for authored drive gains/targets, free articulations, current Isaac Lab APIs and this task's state access. The accepted [DIN-rail mounting implementation](https://github.com/robocurve/benchmark-tasks/tree/main/tasks/din-rail-mount) also informed the mechanism/controller integration approach. No explicit upstream license was identified for the borrowed integration code; this README does not assign MIT or another license to it. The original geometry and original authoring-code licenses supplied with the asset kit do not relicense upstream code.

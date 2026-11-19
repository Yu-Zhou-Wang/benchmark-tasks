# Reinstall a microwave turntable after cleaning

Center the roller ring in the microwave, seat the glass tray on its three drive pins, and place the orange mug off center. Close the door and press the green start button. Full credit requires the mug to ride continuously through a complete driven revolution.

The setup follows Whirlpool’s instructions to install the support first and align the underside of the glass with the hub. The bench uses a motor-only cycle to test the installation. [Manufacturer’s installation guide](https://producthelp.whirlpool.com/Cooking/Wall_Ovens_and_Ranges/Wall_Ovens_new/Cleaning_and_Maintenance/Microwave/Cleaning_the_Turntable_-_Wall_Oven)

![Starting layout with the loose roller ring, glass tray and mug beside the open oven](setup.png)

## Run a trial

Use the RoboCurve YAM environment with Python 3.12, MuJoCo 3.11, Inspect Robots 1.0, the Inspect Robots Dream plugin, and YAM Arena revision `0059d02`. The task reads the arm meshes, URDF and camera calibration from that installation. The microwave, motor, ring, tray, mug and drying stands are original procedural geometry in `scene.py`. [YAM environment](https://github.com/kairosiann/yam-isaac-lab-arena/tree/0059d02)

From the configured workstation’s RoboCurve workspace, run the following commands with fresh output paths. Other installations can run the same Python entry points in the activated environment with an EGL renderer.

```bash
task_dir=worktrees/microwave-turntable/tasks/microwave-turntable
./scripts/run_on_gpu.sh 0 python "$task_dir/demo.py" --record --out runs/microwave-demo-new
./scripts/run_on_gpu.sh 0 python "$task_dir/dream_run.py" --out runs/microwave-dream-new
```

The scripted command writes a score, per-action trace, setup image and H.264 video. The video switches to a view through the closed door during rotation. The Dream command uses the account’s configured Dream credentials and runs one hosted model trial. Inspect Robots stores all three camera streams and scores the last step with `task_score`. The camera recordings can be exported after a run with `inspect-robots video <evaluator-log.json>`.

Both controllers send 14 absolute joint and gripper targets at 20 Hz. Dream receives the calibrated top and two wrist RGB feeds at 320 × 176, joint positions, gripper openings and the instruction. The scripted controller computes inverse kinematics from fixed setup dimensions and its own commanded robot joints. Only the scorer and the diagnostic trace read the simulated objects. The physics timestep is 2 ms. Seed 0 resets the same fixed layout. The limit is 1,400 actions, or 70 simulated seconds.

## Scoring

The score uses the highest milestone satisfied at the end of the trial. The ring, tray and mug placement requirements accumulate. Measurements use the hub axis and oven floor.

| Score | Physical condition |
|---|---|
| 0 | The ring fails its placement check. |
| 0.25 | The released ring lies within 12 mm of center, with its wheel axles 10 ± 3 mm above the floor and tilt at most 4°. |
| 0.50 | The released tray lies within 5 mm of center, at a center height of 24 ± 2.5 mm, tilted at most 2°. At least two rollers touch its underside. The underside sockets align within 22° of the nearest of the three keyed positions. |
| 0.75 | The released mug contacts the tray, stays within 5° of upright and sits 35–78 mm from the hub. Its base is 4 ± 3 mm above the tray’s center plane. |
| 1.00 | With the door within 2° of closed, the energized shaft, glass and mug each advance at least 360° during the same uninterrupted interval. Each advances at 3.3–8.7 rpm at every 50 ms sample and the drive pins bear at least 0.02 N against the tray sockets. Losing any condition restarts the turn measurement. |

Pressing the spring-loaded key by 2 mm with the door closed starts the 30-second motor timer. The motor drives a revolute spindle rated at 6 rpm. All three loose objects move through rigid contact, including the mug riding on the glass. Opening the door cuts power. The ring has three independently rotating wheels. The tray uses opaque frosted-glass shading so its underside obscures the coupling. The appliance models the installation and mechanical rotation cycle. Heating, liquids and glass breakage are outside the simulation.

## Recorded results

| Trial | Score | Actions | Outcome |
|---|---:|---:|---|
| Scripted demo | 1.00 | 1,347 | The shaft turned 361.98°, the glass 361.26°, and the mug 361.16°. The mug finished 65.2 mm off center with all three rollers supporting the glass. |
| Dream-1 beta | 0.00 | 1,400 | One scored trial reached the action limit, with 29 model requests and zero errored trials. |
| Missed-hub control | 0.25 | 1,400 | The same script aimed the glass 18 mm sideways. The motor ran and the ring earned credit. The tray and mug earned zero degrees of driven rotation. |

The recorded demo took 6.4 seconds of wall time and Dream took 22.4 seconds on the test workstation. Hosted inference time can vary. The control can be reproduced with `demo.py --miss-hub --out <fresh-directory>`. [Machine-readable results](results.json)

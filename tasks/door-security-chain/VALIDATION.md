Local validation used the same final source manifest for the recorded demo and all three physical controls. All simulator commands ran through the physical GPU-0 launcher. The controller sent robot joint-target Actions only; measured object poses were logged and scored, never used to adjust its commands.

| Trial | Result | Actions | Measured outcome |
| --- | ---: | ---: | --- |
| Complete recorded demonstration | 1.0 | 866 | Door 56.6469°, released head, clear grippers, stable for 1.5 s |
| Pull latched door toward 55° | 0 | 331 | Maximum opening 23.9724°; head never released |
| Withdraw without sliding | 0 | 536 | The engaged fitting pulled the panel about 7° open but stayed captive behind the narrow lips |
| Slide while holding the door open | 0 | 496 | Head moved 34.62 mm, then stalled 32.08 mm short of the exit centre |
| Native resets after full success | 0 throughout | 2 × 200 | Both returned to the captured initial configuration and passed |
| CPU scorer counterexamples | 12 passed | — | Includes moving/held fitting, retained open-door geometry, broken continuity and a rotated head still intersecting the housing |

The complete trial took 43.30 simulated seconds, within the 1,800-action / 90-second limit. The recorded run plus two resets took 306.9 wall-clock seconds on this workstation. Physics timestep: 1/240 s; control timestep: 1/20 s. Native measured hinge gains are 0.600000 N·m/rad and 0.160000 N·m·s/rad; every chain joint has zero stiffness and 0.000300 N·m·s/rad damping. The greatest measured joint endpoint gap in the complete trial was 2.33e-07 m.

The narrow-slot control actually engaged the fitting: pulling it moved the door with the head still captured. The loaded-chain control also engaged it, evidenced by its measured slider travel and camera frames. These controls used the same robot interface and physical model as the complete demo.

The first release occurred at action 499; wide-exit alignment was first measured at action 467. Full score comes from the adapter's native success termination after the geometric predicate remains true for 1.5 s. Its per-step partial-score channel remains 0.5 during that final hold.

Task source SHA-256: `08cb66eddf45ebaa2120b480bdea1083377abcb936bd908562cc445c2ce68845`

Asset SHA-256: `557b3dfa4f49f48241cafad32b8cb7626f626ada03a4bdd8c515e083bd833b53`

The source asset rebuilds byte-for-byte from the included CPU generator. The prepared evidence bundle keeps result JSON, manifests, all robot actions, per-step traces, initial/final images, the three camera recordings, and the composite H.264 demonstration outside the source archive. The setup PNG is the unchanged first top-camera image of that recorded trial.

Earlier failures remain in the evidence bundle: unstable chain-joint attempts, a straight initial chain incorrectly intersecting the door edge, a pushing-only handle orientation that released the fitting but stopped at 35°, and a subsequent grasp that missed the fitting because the door had not quite closed. They are not represented as successful trials.

Dream-1 beta completed 1,800 actions (90 simulated seconds) with native task_score 0.0, max_steps termination and zero errored trials. Wall time was 550.7 seconds. The client recovered from 12 session-expiration messages and 12 hosted pool-full messages within this single scored trial. The runtime Python and USD hashes match the reviewed demonstration and physical controls. The model video contains all 1,801 frames from each of the three policy cameras, is H.264/yuv420p at 20 fps, and passes a complete decode check. Sampled first, middle and final video frames were inspected; the fitting remains retained in the inspected final frame.

An earlier model attempt ended externally with SIGTERM before the evaluator wrote a final score. Its 1,230 frames per camera, console log and incomplete live log are preserved as an unscored interruption. The user authorized the replacement trial reported above. No result is inferred for the interrupted attempt. DREAM_EVALUATION.json records the completed trial and its artifact hashes.

The authenticated task board was checked at 2026-11-17 04:06 UTC. Its 587 active/planned entries match the board summary counts; all 165 held revision entries were nonfinal and within 48 hours. No direct match was identified. OVERLAP_REVIEW.json records nearby task comparisons. The media ID is still empty; media upload, remote push and PR creation remain pending.

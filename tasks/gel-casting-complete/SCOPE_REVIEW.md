# Scope: secure and dock a gel-casting cassette

Reviewed on 2026-11-09: all **18 tasks currently merged into the public `robocurve/benchmark-tasks` repository**, matched to 18 merged task PRs. The [repository rules](https://github.com/robocurve/benchmark-tasks/blob/main/README.md) state that acceptance is represented by merging. This is complete coverage of the current public repository. The total across historical cohorts on the signed-in task board could not be verified; do not describe this as a majority of all historical accepted tasks.

By our classification, 13 of the 18 tasks use one manipulation family, sometimes repeated across several objects or combined with selection. Five use a short dependent sequence. These are analytical groupings, not official board labels or counts of robot actions. Run limits are 60 seconds for six tasks, 70 seconds for one, and 120 seconds for eleven. Every inspected task focuses on a bounded workplace outcome rather than a complete laboratory or manufacturing process.

| Accepted precedent | Prepared starting state and required work |
|---|---|
| [Access-panel closeout](https://github.com/robocurve/benchmark-tasks/blob/main/tasks/access-panel-closeout/task.json) | The frame and retainers are prepared; seat the panel, then place two retainers. |
| [DIN rail mounting](https://github.com/robocurve/benchmark-tasks/blob/main/tasks/din-rail-mount/task.json) | A module with its hook and latch already installed starts in a holder; hook, pivot and latch it onto a mounted rail. |
| [Dual-latch lid](https://github.com/robocurve/benchmark-tasks/blob/main/tasks/dual-latch-lid/task.json) | The lid and latch mechanisms are installed; release both latches, lift the lid and set it down. |
| [Balance centrifuge](https://github.com/robocurve/benchmark-tasks/blob/main/tasks/balance-centrifuge/task.json) | The rotor and sample tubes are prepared; seat two tubes in opposite pockets. The task excludes filling, closing a lid and spinning. |

The gel-casting task therefore covers three operation types:

1. Secure pre-aligned glass plates in the open frame by closing both side cams.
2. Transfer the retained cassette out of its alignment cradle and onto the stand's gasket.
3. Hold the spring lever open while docking, release it onto the tall plate, then withdraw both grippers.

The prepared starting state is explicit: spacers are bonded to the rear plate, the glass bottoms are aligned in a visible cradle, and the stand's sealing pad is installed. There are no plate-to-frame or cassette-to-stand attachment shortcuts. Pouring, polymerization and later comb insertion are outside this manipulation task.

This scope follows the accepted mounting and closeout examples while preserving the characteristic gel-casting mechanism. The intended two-hand dependency is the spring lever: one hand holds it open while the other docks the cassette. A successful bimanual demonstration and a single-arm counterexample check are still needed before claiming that two arms are necessary. Acceptance itself does not establish such a requirement: the accepted [fragile-stow task](https://github.com/robocurve/benchmark-tasks/blob/main/tasks/fragile-stow/task.json) explicitly leaves its right arm idle.

# Repository working notes

- Start with README.md, docs/features.md, docs/interfaces.md, docs/evaluation.md and docs/handoff.md.
- This is a perception benchmark. Algorithms belong in src/algorithms and consume sensor messages; do not feed ground truth into an algorithm to manufacture detections.
- Simulation realism enhancements are deferred at the user's request. Do not add reflectance, rotating-scan distortion, photorealism or suspension work to the current milestone.
- Parking is removed at the user's request. Do not reintroduce its geometry, truth labels or tasks.
- The active tasks are 3D object detection and geometric left/right road boundaries. Boundary cones are not target obstacles; reporting them as obstacles is a false positive. Standalone 2D image detection and surface-region scoring are outside the current milestone.
- This conversation develops the tasks, output contracts and scoring. The next algorithm milestone is a real demonstrator consuming sensors, not only the existing empty-message template; do not describe that demonstrator as implemented.
- Numeric scoring thresholds are a baseline, not a finalized acceptance agreement. Explain and document proposed changes before implementing a different scoring policy.
- Keep ground-truth overlays out of debug.rviz; truth comparison belongs in perception.rviz.
- Keep generated scenes, recordings, results and build/install/log files out of Git. Do not introduce large sample bags.
- Run ./test.sh after changes affecting scoring, message conversion, scenario safety or control. Live validation commands are documented in docs/acceptance.md; do not claim they passed unless actually run.

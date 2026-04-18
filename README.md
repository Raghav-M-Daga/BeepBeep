# BeepBeep

**Bezier-based path generation and following for FTC robots**, with odometry-backed correction for position and velocity along the path.

This repository extends the [Road Runner](https://github.com/acmerobotics/road-runner) quickstart for [FIRST Tech Challenge](https://www.firstinspires.org/robotics/ftc). It adds a custom controller that tracks **cubic Bézier curves** in the plane: motion is parameterized by **arc length** along the curve, while independent **PID loops** and **feedforward** terms (velocity and acceleration, plus static friction where tuned) correct for tracking error against the robot’s localized pose.

---

## What this project does

| Piece | Role |
|--------|------|
| **Bézier geometry** | Each axis (`x` and `y`) is a cubic Bézier; together they define a smooth 2D path. |
| **Arc length ↔ parameter `u`** | The curve length is estimated by numerical integration. Distance along the path maps to the Bézier parameter so the robot can “walk” the curve at a controlled speed. |
| **Motion profile** | A trapezoidal profile in **distance along the path** sets target position, velocity, and acceleration over time (`MotionProfile`). |
| **Control** | **PID** on world-frame `x`, `y`, and heading reduces position error; **feedforward** uses commanded velocity/acceleration (and tuned `kV` / `kA` / `kS`-style terms) so the drivetrain anticipates what the path demands. |
| **Localization** | Pose comes from Road Runner–style odometry (e.g. mecanum + dead wheels / IMU via `SampleMecanumDrive` and related classes). |

Together, this yields autonomous motion that follows curved paths while continuously adjusting for where the robot actually is and how fast it should move along the trajectory.

---

## Repository layout

- **`BeepBeep/`** — Android / FTC SDK project root (Gradle). **Open this folder in Android Studio** to build and deploy.
- **`BeepBeep/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`** — Team code:
  - **`beepbeep/`** — Core math and tuning: `BezierCurveCalc`, `MotionProfile`, `BeepDriveConstants`, linear test op modes (`LinearTest`, `FollowMultipleLinearTest`, etc.).
  - **`pointfollow/`** — `BezierCurve`, `CurvePointCalc`, and `FollowCurvePointTest` for curve sampling and experiments along a Bézier.
  - **`beepbeeplib/`** — Drive wrappers, `TrajFollower`, and utilities that connect controllers to hardware.
  - **`drive/`** — Road Runner drive constants, tuners, and baseline mecanum configuration (`DriveConstants`, `SampleMecanumDrive` in the main `drive` tree vs. the copy under `beepbeeplib` used by BeepBeep op modes—follow imports in the op mode you run).

Nested **`BeepBeep/README.md`** still describes the stock Road Runner quickstart install; this file focuses on the **BeepBeep** path-following stack.

---

## Building and running

1. Install [Android Studio](https://developer.android.com/studio) and the FTC toolchain as for any `ftc_app`-style project.
2. Open the **`BeepBeep`** subdirectory (not the repo root) as the Gradle project.
3. Build the **`TeamCode`** module and install the Robot Controller app on your Control Hub or phone.
4. Use **FTC Dashboard** (already wired in several op modes via `FtcDashboard`) to tune gains live where `@Config` is used—especially `BeepDriveConstants`.

If you hit multidex limits, the Road Runner quickstart notes applying ProGuard in `build.common.gradle`; see the nested README or [Road Runner quickstart docs](https://rr.brott.dev/docs/v0-5/quickstart/introduction/).

---

## Tuning (high level)

- **`BeepDriveConstants`** — PID gains for `x`, `y`, and heading; feedforward scalars; max linear/angular velocity and acceleration for profiles.
- **`drive/DriveConstants.java`** — Robot physical constants (track width, wheel radii, odometry) used by Road Runner localization; must match your bot for pose and feedforward to make sense.
- Run the usual Road Runner op modes under `drive/opmode/` (track width, follower PID, etc.) to calibrate the base drive, then refine BeepBeep-specific gains on real Bézier runs.

---

## References

- [Road Runner](https://github.com/acmerobotics/road-runner) — trajectory generation, kinematics, and localization patterns used here.
- [FTC SDK](https://github.com/FIRST-Tech-Challenge/FtcRobotController) — robot controller runtime and hardware API.

---

## License

See license files under `BeepBeep/TeamCode` and upstream FTC / Road Runner documentation for attribution and usage terms.

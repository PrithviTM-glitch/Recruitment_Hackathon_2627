# Team Simplicity

## Members

| Name | Matric Number | Email |
|---|---|---|
| Prithvi | IN FORM | IN FORM |
| To be added | To be added | To be added |
| To be added | To be added | To be added |

## Approach

We implemented a reactive Follow-the-Gap algorithm with disparity extension, guided by the circuit centreline. This combines LiDAR-based obstacle avoidance with a directional preference for following the track correctly.

Each LiDAR scan is smoothed to reduce noise, and range discontinuities are extended to account for the vehicle's half-width. This prevents the car from targeting gaps that are too narrow for it to pass through safely. We also apply a safety bubble around the closest obstacle to maintain clearance. The remaining free-space gaps are evaluated based on their alignment with the bearing to a centreline point 1.5 m ahead. The car then steers towards the selected gap while maintaining a margin from its edges.

We chose this approach because it relies primarily on LiDAR for obstacle avoidance, generalises to an unfamiliar circuit, and was relatively quick to implement and tune. However, a conventional Follow-the-Gap algorithm selects the widest available gap, which caused the car to turn in the wrong direction on the starting straight. We addressed this by incorporating the centreline bearing into gap selection.

More advanced approaches, including pure pursuit, racing-line optimisation, model predictive control (MPC), and learned policies, were considered but not implemented.

### Challenges and Improvements

Several early versions of the controller had issues that required iterative testing and tuning:

- **Incorrect direction selection:** Selecting the widest gap alone caused the car to drive in the wrong direction on the starting straight. Incorporating the centreline bearing corrected this behaviour.
- **Corner collisions:** An initial configuration using a 2.0 m lookahead and a 0.20 m disparity margin caused the car to cut inside corners and clip walls at turns 1–3. Reducing the lookahead and increasing the safety margins improved corner clearance.
- **Runtime failure:** An early version crashed because a required parameter was missing, leaving the car without active control and allowing it to coast into a wall. We subsequently addressed the configuration issue.

These iterations helped us establish a more reliable balance between following the track, avoiding obstacles, and maintaining speed.

## Input Usage

### LiDAR
LiDAR is the primary sensor used for reactive steering. The controller processes 1,080 beams within a 90-degree forward field of view. The scan is smoothed, range discontinuities are extended, and a safety bubble is applied before the remaining free-space gaps are evaluated. Forward clearance also contributes to target-speed selection.

### Pose
The controller uses ground-truth pose data from `/odom`, including position, yaw, and speed. Position and yaw are used to determine the bearing towards the centreline lookahead point, while the measured speed is fed into the speed controller. We rely on the simulator's ground-truth localisation rather than implementing a separate localisation system.

### Camera, IMU, and Encoders
These inputs are not used by our controller.

## Speed Control

We use a feed-forward plus proportional-integral (PI) controller, without a derivative term. The target speed decreases as the steering angle increases or the available forward clearance decreases, allowing the car to slow down when approaching sharper turns or obstacles.

The throttle command is calculated as:

`throttle = ff_gain * target_speed + kp * error + ki * integral`

The integral term is clamped to prevent excessive accumulation, and the output is constrained to the range [0, 1]. The controller does not actively apply braking when the car exceeds its target speed; instead, it reduces throttle and allows the car to slow down naturally.

The controller maintains the target speed reasonably well on straights. However, we evaluated its performance primarily through lap times and collision counts rather than measuring speed-tracking error on a dedicated test rig.

All tuning parameters are stored in `race_ws/src/team_driver/config/driver_params.yaml`. The driver automatically reloads the configuration within approximately one second of a file change, allowing parameters to be adjusted during a run without restarting the driver. The same default values are compiled into the driver as a fallback.

## Results on Our Development Machine

We committed four evaluation runs under `results/submitted/`. All four runs completed successfully, received scores, and were submitted without modifying their results.

| Metric | Result |
|---|---:|
| Best lap time | 13.441 s |
| Best 10-lap total | 136.52 s |
| Collisions | 0 across all four runs |
| Committed runs | 4 |

The individual results were:

| Run | Best Lap Time | 10-Lap Total |
|---|---:|---:|
| 1 | 13.566 s | 136.52 s |
| 2 | 13.441 s | 137.95 s |
| 3 | 13.554 s | 136.67 s |
| 4 | 13.516 s | 136.54 s |


### Development Environment

| Component | Specification |
|---|---|
| CPU | Apple M4 |
| GPU | Integrated GPU; no discrete GPU |
| RAM | 16 GB |
| Operating System | macOS 27.0.1 |
| Typical Real-Time Factor | Approximately 1.0 |

The simulator ran using its native macOS build on the host machine. The bridge, referee, and driver ran inside a Docker container, launched using `./scripts/evaluate.sh --no-simulator`.

### GitHub CI Performance

The GitHub Judge workflow produces significantly slower lap times for the same code. Our final commit records approximately 19 seconds per lap in CI, compared with approximately 13.4–13.6 seconds on our development machine. Both environments report zero collisions, and the CI run uses the Linux headless simulator with a four-core runner and a real-time factor of 1.00.

We have not yet identified the cause of this discrepancy. Potential explanations include differences between the macOS and Linux simulator builds, speed-controller parameters tuned on the development machine, and Python execution overhead on the CI runner. These remain hypotheses rather than confirmed causes.

Consequently, our local lap times should not be treated as a reliable prediction of judging performance. Further testing on the judging environment would be needed to establish representative results.

## Precomputed Data

We did not generate any custom maps, racing lines, or trained model weights.

The controller reads `maps/icra26_compete_centerline.csv`, which is included in the repository and was generated using the organisers' `scripts/track_tool.py`. We use this provided centreline to determine the preferred steering direction; all obstacle avoidance and gap selection are performed at runtime.

## Additional Dependencies

We did not add any dependencies. The existing `requirements.txt` and `apt-packages.txt` remain unchanged.

The driver uses NumPy, PyYAML, and the ROS 2 packages already available in the provided environment.

| Package | Installation Method | Purpose |
|---|---|---|
| None added | — | — |

## Third-Party Code and References

- **F1TENTH Follow-the-Gap:** We used the Follow-the-Gap approach and the concept of disparity extension from the F1TENTH community as references. The implementation in our driver was written by us.
- **Repository documentation:** `docs/04-algorithms.md` informed our implementation of the speed controller, particularly its feed-forward and PI terms, integral clamping, and throttle reduction instead of active braking.

## AI Assistance

We used Claude Code to help implement the driver. We reviewed the generated code and results, and every team member is prepared to explain the implementation and design decisions during the interview.

## Known Limitations and Future Improvements

- **Inconsistent evaluation performance:** Lap times in GitHub CI are approximately five seconds slower per lap than on our development machine. The cause remains undetermined.
- **Dependence on ground-truth pose:** Centreline-guided gap selection relies on ground-truth position and yaw. Without pose data, the controller falls back to selecting the widest gap, which can result in incorrect direction selection on the starting straight.
- **Machine-dependent tuning:** The speed controller was tuned on one development machine. Differences in simulator behaviour or execution timing may require adjustments to parameters such as `v_max` and `brake_dist`.
- **Reactive cornering limitations:** The controller responds to currently visible free space rather than anticipating upcoming corners. With additional development time, we would derive a racing line from the provided centreline and construct a speed profile that accounts for upcoming turns, enabling the car to brake earlier and maintain more consistent cornering speeds.


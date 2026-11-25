# Test plan

Acceptance criteria are defined before testing and stored in `amr_validation/config/scenarios.yaml`.
Parameters may be tuned to meet them; the limits themselves are not changed.

## Requirements and verification

| ID | Requirement | Verified by |
|---|---|---|
| R1 | Complete transport missions between named stations | `nominal_transport`, fleet manager endurance run |
| R2 | Localization error at most 0.10 m RMSE and 0.30 m maximum while driving | Ground truth compared with AMCL in every scenario |
| R3 | Recover from a 0.4 m / 0.2 rad initial error within 20 s of driving | `localization_recovery` |
| R4 | At most 50 % of the maximum speed (plus 15 % tolerance) inside the rack aisles | `speed_zone` |
| R5 | Never enter the office walkway | `keepout_respect` |
| R6 | No contact with static, moving or suddenly appearing obstacles | `min_clearance` in every scenario, `sudden_obstacle`, `forklift_crossing` |
| R7 | E-stop stops within 0.40 m, holds until reset, then the mission resumes | `emergency_stop`, safety unit tests |
| R8 | Loss of lidar data stops the robot | `test_safety_fields.py` |
| R9 | An unmapped blockage leads to a replan | `blocked_aisle_replan` |
| R10 | Docking accuracy at most 0.05 m and 0.05 rad | `docking` |
| R11 | Failed steps are retried with recovery, at most 3 attempts | `test_mission_logic.py` |
| R12 | VDA 5050 v2 order rules: validation, updates, cancel | `test_order_manager.py`, `tools/vda5050_mqtt_e2e.py` |
| R13 | World, map and zone masks are consistent | `tests/test_generated_files.py` |

## Test levels

1. Unit tests: pure Python logic without ROS, run on every push.
2. Package tests: `colcon test` in the ROS 2 Jazzy container in CI.
3. Launch test: the full stack starts headless; sensors, TF tree and Nav2 lifecycle states are checked.
4. Scenario tests: closed loop in simulation with ground truth, pass or fail per KPI.
5. Endurance: random VDA 5050 transport jobs; the success rate is reported with its 95 % Wilson lower bound.

## Running the scenario suite

```bash
bash tools/clean_sim.sh
ros2 launch amr_bringup bringup.launch.py headless:=true rviz:=false forklift:=false
ros2 run amr_validation scenario_runner --output test_results/run
```

Wait until `ros2 control list_controllers` shows both controllers active before starting the runner.
Each run writes `report.md`, `summary.json` and CSV logs of ground truth, AMCL and safety state per scenario.

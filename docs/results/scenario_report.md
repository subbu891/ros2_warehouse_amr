# Scenario test report

**7/9 scenarios passed**

| Scenario | Result | Duration [s] | Notes |
|---|---|---|---|
| nominal_transport | PASS | 104.7 | mission complete |
| speed_zone | PASS | 31.2 | mission complete |
| keepout_respect | PASS | 23.9 | mission complete |
| forklift_crossing | FAIL | 213.6 | localization_rmse_m, localization_max_m |
| sudden_obstacle | PASS | 29.1 | mission complete |
| emergency_stop | PASS | 31.0 | mission complete |
| localization_recovery | PASS | 35.0 | mission complete |
| blocked_aisle_replan | PASS | 52.6 | mission complete |
| docking | FAIL | 74.2 | mission_success, localization_rmse_m, dock_error_xy_m |

## nominal_transport

Pick at A, drop at B, return home. The everyday job.

| Check | Value | Limit | Result |
|---|---|---|---|
| mission_success | 1.000 | >= 1.0 | PASS |
| min_clearance_m | 0.458 | >= 0.03 | PASS |
| localization_rmse_m | 0.071 | <= 0.1 | PASS |
| localization_max_m | 0.137 | <= 0.3 | PASS |

## speed_zone

Drive through the rack aisle; speed must stay at or below 50 % of 0.8 m/s inside it.

| Check | Value | Limit | Result |
|---|---|---|---|
| mission_success | 1.000 | >= 1.0 | PASS |
| min_clearance_m | 0.712 | >= 0.03 | PASS |
| localization_rmse_m | 0.051 | <= 0.1 | PASS |
| localization_max_m | 0.093 | <= 0.3 | PASS |
| max_speed_in_rack_aisles_mps | 0.400 | <= 0.46 | PASS |

## keepout_respect

The direct line crosses the office walkway; the robot must go around it.

| Check | Value | Limit | Result |
|---|---|---|---|
| mission_success | 1.000 | >= 1.0 | PASS |
| min_clearance_m | 0.508 | >= 0.03 | PASS |
| localization_rmse_m | 0.039 | <= 0.1 | PASS |
| localization_max_m | 0.076 | <= 0.3 | PASS |
| keepout_violations | 0.000 | <= 0.0 | PASS |

## forklift_crossing

Shuttle across the forklift lane twice while the forklift moves.

| Check | Value | Limit | Result |
|---|---|---|---|
| mission_success | 1.000 | >= 1.0 | PASS |
| min_clearance_m | 0.395 | >= 0.03 | PASS |
| localization_rmse_m | 0.120 | <= 0.1 | FAIL |
| localization_max_m | 0.444 | <= 0.3 | FAIL |
| forklift_travel_m | 7.700 | >= 2.0 | PASS |

## sudden_obstacle

A box appears 1.2 m in front of the moving robot. No contact allowed.

| Check | Value | Limit | Result |
|---|---|---|---|
| min_clearance_m | 0.429 | >= 0.03 | PASS |
| localization_rmse_m | 0.064 | <= 0.1 | PASS |
| localization_max_m | 0.115 | <= 0.3 | PASS |

## emergency_stop

E-stop while driving; the robot must stop within its protective field, stay stopped and resume after reset.

| Check | Value | Limit | Result |
|---|---|---|---|
| mission_success | 1.000 | >= 1.0 | PASS |
| min_clearance_m | 1.216 | >= 0.03 | PASS |
| localization_rmse_m | 0.065 | <= 0.1 | PASS |
| localization_max_m | 0.136 | <= 0.3 | PASS |
| estop_stopping_distance_m | 0.067 | <= 0.4 | PASS |

## localization_recovery

AMCL starts 0.4 m and 0.2 rad off; it must converge while driving.

| Check | Value | Limit | Result |
|---|---|---|---|
| mission_success | 1.000 | >= 1.0 | PASS |
| min_clearance_m | 1.038 | >= 0.03 | PASS |
| localization_convergence_s | 7.000 | <= 20.0 | PASS |

## blocked_aisle_replan

An unmapped pallet blocks the middle aisle; the planner must find another way.

| Check | Value | Limit | Result |
|---|---|---|---|
| mission_success | 1.000 | >= 1.0 | PASS |
| min_clearance_m | 0.450 | >= 0.03 | PASS |
| localization_rmse_m | 0.057 | <= 0.1 | PASS |
| localization_max_m | 0.094 | <= 0.3 | PASS |

## docking

Dock at the charger and undock again.

| Check | Value | Limit | Result |
|---|---|---|---|
| mission_success | 0.000 | >= 1.0 | FAIL |
| min_clearance_m | 0.168 | >= 0.0 | PASS |
| localization_rmse_m | 0.148 | <= 0.1 | FAIL |
| localization_max_m | 0.196 | <= 0.3 | PASS |
| dock_error_xy_m | 0.685 | <= 0.05 | FAIL |
| dock_error_yaw_rad | 0.032 | <= 0.05 | PASS |

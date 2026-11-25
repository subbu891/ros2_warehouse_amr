# Safety concept

This project models the behaviour of a safety laser scanner so it can be tested in simulation.
A real product needs a certified safety scanner and safety controller (ISO 13849-1, ISO 3691-4).

## Safety functions

| ID | Function | Implementation | Reaction |
|---|---|---|---|
| SF1 | Speed-dependent protective field | `v * t_response + v^2 / (2 * a_brake) + margin` ahead of the footprint | Stop, automatic restart after 1 s clear |
| SF2 | Warning field | Protective length plus `v * t_warning + margin` | Speed reduced to at least 20 % |
| SF3 | Rotation field | 8 cm ring around the footprint while turning on the spot | Stop |
| SF4 | Emergency stop | `/safety/estop` | Stop, latched until `/safety/reset` |
| SF5 | Lidar watchdog | No scan for 0.5 s | Stop |
| SF6 | Command watchdog | No velocity command for 0.5 s | Zero velocity |
| SF7 | Speed zones | Nav2 speed filter, 50 % in the rack aisles | Lower maximum speed |
| SF8 | Keepout zones | Nav2 keepout filter with a 0.35 m buffer | Never planned through |
| SF9 | Docking field set | 2 cm protective field and 0.15 m/s speed limit, only while docking | Final approach to the charger |

With `t_response = 0.15 s`, `a_brake = 1.0 m/s^2` and a 0.10 m margin, the protective field at 0.8 m/s is 0.54 m.

## Hazards

| Hazard | Cause | Mitigation | Verified by |
|---|---|---|---|
| Collision with a person stepping in | Late detection | SF1, SF2, obstacle prediction in the costmap | `sudden_obstacle` |
| Collision with a forklift | Crossing traffic | Tracking and prediction, SF1 | `forklift_crossing` |
| Driving blind | Lidar or driver failure | SF5 | Safety unit tests |
| Runaway after a software crash | Stale command | SF6, controller command timeout | Safety unit tests |
| Entering a pedestrian area | Shortest path through the walkway | SF8 | `keepout_respect` |
| Too fast in narrow aisles | Planner speed | SF7, SF1 | `speed_zone` |
| Contact while docking | Reduced fields | SF9 limited to docking, speed limit, contact stop | `docking`, safety unit tests |

## Limitations

- The 2D lidar at 0.33 m height does not see overhanging loads.
- The fields are rectangles; curved paths would need dedicated field sets for turning.
- Automatic restart after a protective stop must be confirmed by the risk assessment of a real site.

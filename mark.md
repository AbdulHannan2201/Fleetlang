# FleetLang: Language-Grounded Multi-Robot Task Allocation and Navigation for Warehouse Logistics

*A portfolio research project designed as a precursor to a Master's thesis under Prof. Jun Ota, Mobile Robotics Laboratory (RACE), The University of Tokyo.*

---

## 1. Project Title

**FleetLang** — Language-Grounded Task Decomposition and Scalable Multi-Robot Allocation for Warehouse Logistics

---

## 2. Problem Statement

Warehouse and logistics fleets are typically reconfigured by hand: an operator translates a high-level goal ("clear aisle 3 before the next shift," "move all pallets tagged for outbound to the loading dock") into a structured task list that a fleet manager or allocation algorithm can consume. This translation step is manual, slow to adapt, and becomes a bottleneck as fleets and task complexity grow. FleetLang asks: can a fleet of mobile robots take a natural-language operational instruction directly, decompose it into a structured, scalable task set, allocate it across the fleet, and execute it — while remaining robust to robot failure and dynamic obstacles?

## 3. Research Motivation

Large-scale multi-agent task planners (e.g., neighborhood-search-based allocation methods) are efficient once a task set is given, but they treat that task set as an input, not an output — they say nothing about how it was derived. Separately, Vision-Language-Action and LLM-grounded approaches show that language can be turned into structured robot goals, but almost always for one robot or a very small team, not evaluated for scalability. FleetLang sits at the intersection: a language-to-task front end feeding a scalable multi-agent allocator, evaluated specifically as agent and task counts grow.

## 4. Novelty

- Most open-source multi-robot ROS 2 projects either (a) hand-specify a fixed task list, or (b) demo language-conditioned control on a single robot. FleetLang couples the two and treats **instruction-to-task fidelity at scale** as a first-class evaluation axis, not an afterthought.
- A semantic map (occupancy grid + open-vocabulary landmark labels) is used as the grounding layer between language and geometry, so "the loading dock" resolves to coordinates without hardcoded waypoint names.
- The allocator is designed to be swappable — a greedy/auction baseline first, with a defined interface to plug in a neighborhood-search-style allocator later, so the project has an explicit growth path toward thesis-level contribution rather than ending as a demo.

## 5. System Overview

```
Operator instruction (text)
        │
        ▼
 ┌─────────────────┐      ┌────────────────────┐
 │ Language-to-Task │─────▶│  Semantic Map       │
 │ Parser (LLM)     │◀─────│  (landmarks, zones) │
 └─────────────────┘      └────────────────────┘
        │  structured task list
        ▼
 ┌─────────────────┐
 │ Task Allocator   │  (greedy / auction, pluggable)
 └─────────────────┘
        │  per-robot task assignment
        ▼
 ┌─────────────────┐      ┌────────────────────┐
 │ Per-Robot Nav2   │◀────▶│ Fleet Coordinator   │
 │ Stack (xN)        │      │ (conflict/replan)   │
 └─────────────────┘      └────────────────────┘
        │
        ▼
   Sensors / Gazebo Sim / (future) physical robots
```

## 6. Architecture Diagram (Text)

```
[Operator CLI/Web UI]
        │ instruction (string)
        ▼
[/fleet/instruction_parser]  ──uses──▶ [LLM API or local model]
        │ TaskList.msg (custom)
        ▼
[/fleet/task_allocator] ──reads──▶ [/fleet/semantic_map]
        │ TaskAssignment.msg (per robot_id)
        ▼
[/robot_i/task_executor] (one per robot)
        │
   ┌────┴─────┐
   ▼          ▼
[Nav2 stack] [manipulation/placeholder]
   │
   ▼
[/robot_i/cmd_vel] → Gazebo diff-drive plugin
        │
        ▼
[/fleet/status_monitor] ──feeds back──▶ [/fleet/task_allocator] (failure → reassignment)
```

## 7. Hardware Architecture

**Simulation:** N × differential-drive robots (TurtleBot3 Waffle Pi model), each with 2D LiDAR, RGB camera, IMU, and wheel encoders, spawned with namespaced topics/TF in a single Gazebo warehouse world.

**Future physical implementation:** TurtleBot3 or Leo Rover units for mobility; RPLiDAR A2/A3 or equivalent for 2D scan; a fixed or head-mounted RGB(-D) camera for semantic landmark recognition; an ESP32 per robot as a low-cost telemetry/status beacon (battery level, e-stop status, LED indicator) reporting over Wi-Fi/MQTT into the ROS 2 bridge — reusing your existing ESP32/embedded-systems experience directly.

## 8. Software Architecture

Layered design:
1. **Perception layer** — per-robot AMCL/SLAM Toolbox localization, `robot_localization` EKF fusing wheel odometry + IMU (directly reusable from your existing sensor-fusion nav stack).
2. **Semantic layer** — offline or online landmark labeling of the occupancy grid using a zero-shot vision-language classifier (CLIP-style) applied to camera frames at mapped locations, producing a semantic map (`zone_name → polygon/coordinates`).
3. **Language layer** — LLM-based parser converting an instruction into a structured `TaskList` (task type, target zone/object, priority, constraints), grounded against the semantic map.
4. **Planning/allocation layer** — task allocator assigning tasks to robots (Section 13).
5. **Execution layer** — per-robot Nav2 stack + task executor state machine (go-to-zone, wait, report-done).
6. **Coordination layer** — fleet-level monitor handling conflicts, failures, and dynamic replanning.

## 9. ROS 2 Node Graph

| Node | Role |
|---|---|
| `instruction_parser_node` | Calls LLM, emits `TaskList` |
| `semantic_map_node` | Publishes/serves the semantic map |
| `task_allocator_node` | Assigns tasks to robots |
| `robotN/amcl` or `slam_toolbox` | Localization per robot |
| `robotN/ekf_node` (robot_localization) | Sensor fusion per robot |
| `robotN/bt_navigator`, `planner_server`, `controller_server` (Nav2) | Per-robot navigation |
| `robotN/task_executor_node` | Executes assigned task, reports status |
| `fleet_status_monitor_node` | Aggregates robot status, triggers reassignment |

## 10. Communication Topics

| Topic | Type | Publisher → Subscriber |
|---|---|---|
| `/fleet/instruction` | `std_msgs/String` | UI → `instruction_parser_node` |
| `/fleet/task_list` | `fleetlang_msgs/TaskList` | `instruction_parser_node` → `task_allocator_node` |
| `/fleet/semantic_map` | `fleetlang_msgs/SemanticMap` | `semantic_map_node` → `task_allocator_node`, `instruction_parser_node` |
| `/fleet/assignment` | `fleetlang_msgs/TaskAssignment` | `task_allocator_node` → `robotN/task_executor_node` |
| `/robotN/odom`, `/robotN/imu`, `/robotN/scan` | standard | sensors → `ekf_node`, Nav2 |
| `/robotN/task_status` | `fleetlang_msgs/TaskStatus` | `task_executor_node` → `fleet_status_monitor_node` |
| `/fleet/reassignment_request` | `fleetlang_msgs/TaskList` | `fleet_status_monitor_node` → `task_allocator_node` |

## 11. TF Tree

```
map
 ├── robot1/odom → robot1/base_link → robot1/base_scan, robot1/imu_link, robot1/camera_link
 ├── robot2/odom → robot2/base_link → robot2/base_scan, robot2/imu_link, robot2/camera_link
 └── robotN/odom → ...
```
Each robot runs in its own TF namespace with a static transform from `map` to `robotN/odom` maintained by that robot's localization node; the fleet coordinator only reasons in the shared `map` frame.

## 12. Navigation Pipeline

Per robot: Nav2 stack with `SmacPlannerHybrid` (global) + `RegulatedPurePursuit` (local controller), costmaps layered with static (map), obstacle (LiDAR), and inflation layers. A shared "reservation" layer (simple time-window occupancy of shared corridors) prevents two robots from being routed through the same narrow aisle simultaneously — implemented as a lightweight custom costmap plugin rather than full decentralized MRPP for scope reasons.

## 13. Task Allocation Algorithm

**Baseline (Phase 1):** greedy nearest-available-robot assignment with priority ordering — simple, gives a working end-to-end system quickly.

**Extension (Phase 2, thesis-track):** an auction-based / stepwise neighborhood-search-inspired allocator: robots bid on tasks based on travel cost, tasks are assigned iteratively, and a local neighborhood-search pass swaps assignments pairwise if it reduces total makespan — a small-scale, from-scratch analogue of the scalable allocation literature this project is designed to grow into.

## 14. VLA Integration

Two integration points, kept modular so either can be swapped independently:
- **Instruction → task structure:** an LLM (local or API) prompted with the semantic map's zone list to constrain outputs to valid, executable tasks — reducing hallucinated locations.
- **Landmark grounding:** zero-shot vision-language classification (CLIP-style) of camera frames captured during mapping, tagging occupancy-grid regions with open-vocabulary labels ("shelf,", "loading dock," "charging station") so the language layer has real referents to ground against — directly reusing your CLIP-based zero-shot grasping pipeline experience, applied here to scene regions instead of graspable objects.

## 15. Semantic Mapping Pipeline

1. Drive/teleop each robot through the environment once (or use a fixed exploration policy) while SLAM Toolbox builds the occupancy grid.
2. At intervals, capture RGB frames and robot pose; run zero-shot classification against a candidate label set.
3. Cluster labeled frames spatially into named zones; store as polygons in the `SemanticMap` message, versioned so the map can be updated incrementally without re-mapping from scratch.

## 16. Robot Communication Strategy

ROS 2 DDS multi-robot setup using domain IDs or namespacing (`/robot1/...`, `/robot2/...`) on a shared network; a single fleet-level node (`task_allocator_node`) acts as the coordination point rather than fully decentralized consensus, which keeps the system tractable for a Master's-portfolio scope while leaving a clear extension path to decentralized allocation later.

## 17. Coordination Algorithm

Event-driven loop: instruction arrives → parser emits task list → allocator assigns → executors report status → on completion or failure/timeout, the status monitor triggers the allocator to reassign only the affected tasks (not a full re-plan), keeping response latency low.

## 18. Path Planning

Global: `SmacPlannerHybrid` for kinematically-aware paths in aisle-constrained warehouse layouts. Local: `RegulatedPurePursuitController` for smooth tracking with velocity regulation near obstacles. Inter-robot collision avoidance handled at the costmap-reservation level (Section 12), not by a full decentralized MRPP solver, to keep the project scoped and completable.

## 19. Localization

`slam_toolbox` for initial mapping; `AMCL` against the saved map for each robot during task execution, seeded with a known spawn pose per robot in the Gazebo world file.

## 20. Sensor Fusion

`robot_localization` EKF per robot fusing wheel odometry + IMU (and optionally LiDAR scan-matched odometry) — the same fusion architecture as your existing 0.127 m RMSE navigation stack, applied per-robot in a multi-robot namespace.

## 21. Failure Handling

- **Timeout detection:** if a robot doesn't report progress within an expected window, it's flagged stuck.
- **Reassignment:** its incomplete task returns to the allocator's queue for reassignment to another robot.
- **Recovery behaviors:** Nav2's built-in recovery behavior tree (clear costmap, rotate, back up) attempted first before declaring failure.
- **Fleet-level:** if a robot goes fully offline, the status monitor excludes it from future allocation until it re-registers.

## 22. Simulation Design

A Gazebo warehouse world (shelving rows, charging station, loading dock, dynamic obstacle actors) with a parameterized launch system to spawn 2, 4, or 6 namespaced robots for scalability testing. World and robot spawn configs kept as YAML for reproducibility.

## 23. Real Robot Design

Future step: 2–3 TurtleBot3/Leo Rover units in a lab space mocked up as a mini-warehouse, ESP32 beacons for status telemetry, Wi-Fi-based ROS 2 DDS discovery, with the same node graph as simulation to keep sim-to-real transfer straightforward.

## 24. Datasets

- A custom set of 50–100 natural-language operational instructions (varying complexity: single-task, multi-step, conditional), each paired with a ground-truth task list — built specifically for this project, since no existing benchmark pairs language instructions with multi-robot warehouse task sets.
- A small custom image set of warehouse-like zones/objects for zero-shot landmark classification validation (can reuse/extend your existing YCB-based work for object-level labels).

## 25. Evaluation Metrics

- Instruction-to-task parsing accuracy (exact match / semantic match against ground truth)
- Task success rate (fraction of assigned tasks completed)
- Makespan (total time to complete all tasks) as a function of robot count (2/4/6)
- Allocation quality vs. a brute-force-optimal baseline on small instances
- Recovery latency after simulated robot failure

## 26. Ablation Studies

- With vs. without semantic map grounding (fixed waypoint names vs. open-vocabulary zones)
- Greedy vs. auction/neighborhood-search allocator
- LLM-parsed tasks vs. hand-specified task lists (isolating parser-induced errors)
- Centralized allocator vs. simple round-robin baseline
- Performance across 2 / 4 / 6 robots (scalability trend)

## 27. Expected Results

The LLM front end should reduce manual task-specification effort essentially to zero for well-formed instructions, at some accuracy cost on ambiguous ones; the auction/neighborhood-search allocator should outperform greedy assignment on makespan as robot/task counts grow, with the gap widening at higher scale — this scaling trend is the key result to highlight, since it's the direct evidence connecting the project to Prof. Ota's own scalability-focused research.

## 28. Future Research Extensions

- Replace the from-scratch allocator with a direct implementation of stepwise neighborhood search benchmarked against this project's simpler baseline.
- Continual learning of the semantic map as zones change over time (shelving reconfigured, new landmarks).
- Full decentralized multi-robot path planning instead of costmap reservations.
- Physical deployment and sim-to-real gap analysis.

## 29. Possible Conference Publications

A workshop paper at an ICRA/IROS workshop on language-grounded multi-robot systems or task planning; if scalability results are strong, a short RA-L submission extending toward Prof. Ota's neighborhood-search line of work.

## 30. Timeline (High-Level, 4 Phases)

| Phase | Duration | Focus |
|---|---|---|
| 1 | Weeks 1–3 | Single-robot Nav2 + semantic map pipeline working |
| 2 | Weeks 4–6 | Multi-robot spawn, greedy allocator, task executor |
| 3 | Weeks 7–9 | LLM instruction parser, grounding, failure handling |
| 4 | Weeks 10–12 | Scalability evaluation, ablations, writeup |

## 31. Complete Folder Structure

```
fleetlang/
├── fleetlang_msgs/            # custom message/interface definitions
│   └── msg/
│       ├── TaskList.msg
│       ├── TaskAssignment.msg
│       ├── TaskStatus.msg
│       └── SemanticMap.msg
├── fleetlang_bringup/         # launch files, world files, robot spawn configs
│   ├── launch/
│   └── worlds/
├── fleetlang_language/        # instruction_parser_node
├── fleetlang_semantic_map/    # semantic_map_node, landmark classifier
├── fleetlang_allocation/      # task_allocator_node (greedy + neighborhood-search)
├── fleetlang_execution/       # task_executor_node, per-robot state machine
├── fleetlang_monitor/         # fleet_status_monitor_node
├── fleetlang_eval/            # evaluation scripts, ablation configs, plots
├── docs/
│   ├── architecture.md
│   └── evaluation.md
├── tests/
├── docker/
│   └── Dockerfile
├── README.md
└── LICENSE
```

## 32. GitHub Repository Structure

Same as above, plus: `.github/workflows/ci.yml` (build + lint on push), `CONTRIBUTING.md`, versioned `CHANGELOG.md`, and a `results/` folder with saved evaluation plots/tables from each ablation run so the repo itself documents progress over time — this is what makes it read as a research log rather than a static demo.

## 33. README Structure

1. One-line project description + short demo GIF
2. Problem statement (2–3 sentences)
3. Architecture diagram (image, generated from Section 6)
4. Quickstart (Docker + launch commands)
5. Results summary (key scalability plot)
6. Repository structure
7. Roadmap (links to Section 28)
8. Citation block (if a workshop paper results)

## 34. Weekly Development Plan (12 Weeks)

| Week | Deliverable |
|---|---|
| 1 | Gazebo world + single TurtleBot3 spawn, teleop working |
| 2 | SLAM Toolbox mapping + AMCL localization working |
| 3 | `robot_localization` EKF fusion tuned; single-robot Nav2 goal-sending working |
| 4 | Multi-robot namespacing; 2 robots spawn and navigate independently |
| 5 | `fleetlang_msgs` defined; greedy `task_allocator_node` assigning hand-specified tasks |
| 6 | `task_executor_node` state machine; end-to-end hand-specified-task demo (2 robots) |
| 7 | Semantic map pipeline: landmark capture + zero-shot classification |
| 8 | `instruction_parser_node`: LLM prompt design, grounded against semantic map |
| 9 | Full pipeline integration: instruction → task list → allocation → execution |
| 10 | Failure handling: timeout detection + reassignment; scale to 4–6 robots |
| 11 | Evaluation suite: metrics, ablation configs, run experiments |
| 12 | Results writeup, README, architecture docs, demo video |

## 35. Skills Demonstrated

- Multi-robot ROS 2 system design (namespacing, TF, DDS)
- Nav2 stack configuration and tuning (global/local planners, costmaps, recovery behaviors)
- Sensor fusion (EKF, `robot_localization`) — directly extending your existing navigation stack
- Zero-shot vision-language grounding (CLIP-style) — directly extending your existing grasping pipeline
- LLM-based structured task generation with output grounding/validation
- Task allocation algorithm design (greedy baseline → auction/neighborhood-search extension)
- Scalability-focused evaluation methodology (ablations, multi-scale benchmarking)
- Reproducible research repo practices (Docker, CI, documented experiments)

## 36. Why This Project Would Impress Prof. Ota

It's not a language-model demo bolted onto a TurtleBot tutorial — it's structured, from the start, as a smaller-scale, from-scratch version of the exact problem his own recent neighborhood-search paper leaves open: where does the task set come from, and does allocation quality hold up as the system scales? The ablations (2/4/6 robots) and the explicit "Phase 2" allocator upgrade path are what turn this from a portfolio piece into something that reads as the first three months of the Master's proposal you'd already discussed with him — which is exactly the point.
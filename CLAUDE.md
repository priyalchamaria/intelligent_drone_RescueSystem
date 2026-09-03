PROJECT: Intelligent Multi-Drone Disaster Response and Rescue System
CONTEXT: This is a Unity (C#) academic capstone project, Stage 2 of a two-stage 
project. Stage 1 (already complete, separate/untouched) compared 5 static + 5 
dynamic path-planning algorithms and concluded that a Voronoi + ORCA hybrid is the 
best navigation approach. Stage 2 — what we're building now — wraps that navigation 
engine in an intelligent Mission Planner that decides which drone rescues which 
patient, and adapts automatically when the disaster environment changes.

You have Unity MCP access to this project. Use it to inspect the current project 
state before making assumptions — check what scenes, scripts, and assets already 
exist before creating anything new.

============================================================
PART 1 — CORE TECHNICAL SPECIFICATION
============================================================

## Data Structures

Patient {
  id: string
  location: Vector3
  priority: enum { Critical, Serious, Stable }   // NOT named "risk" — see below
}

Drone {
  id: string
  location: Vector3
  batteryPercent: float
  status: enum { Idle, Busy, Charging, Offline }
}

IMPORTANT NAMING RULE: Patient.priority orders the priority queue (which patient 
gets served first). It is a SEPARATE concept from RiskFactor(location), which is 
computed inside the drone-scoring function based on the patient's location (e.g. 
proximity to fire = high risk, on a clear road = low risk). Do not merge these two 
concepts or name them the same thing.

## Patient Priority Assignment (triage-inspired)

Patients do not self-report priority. At the moment a patient is spawned/detected, 
assign priority using a simple triage-style rule, inspired by real START triage 
categories (Critical / Serious / Stable) — NOT a full medical simulation. For 
demo scenarios, allow priority to be either (a) manually set per scenario for 
predictable demo behavior, or (b) weighted-random at spawn time for generated 
scenarios. Implement both — a scenario config flag chooses which mode is active.

## Patient Detection — Simulated, Not Networked

There is NO real communication network, no phone/call simulation. A "new patient" 
event simply instantiates a Patient object directly into the environment's shared 
state at the moment of detection. In comments, note this stands in for what would 
be thermal/camera-based autonomous detection in a real deployment (per the base 
paper) — do not build any actual networking/messaging layer for this.

## Drone Communication Model — Hub-and-Spoke, NOT Peer-to-Peer

Drones do NOT communicate with each other directly. Two separate flows only:
1. Drone → Mission Planner: each drone's live state (location, battery, status) 
   is readable by the Mission Planner via shared state (a live-updated DroneList) 
   — simulating a status uplink, not implementing real wireless.
2. Mission Planner → Drone: dispatch commands flow down the same way.

ORCA-based collision avoidance between drones is explicitly NOT a communication 
channel — each drone independently senses nearby drones' current position/velocity 
(a direct read of their AgentData in this simulation, standing in for onboard 
sensing in reality) and computes its own avoidance velocity, assuming the other 
drone reciprocates. Do not implement any message-passing between drone agents for 
collision avoidance — this must remain communication-free, consistent with ORCA's 
actual design (van den Berg et al., 2011).

## Mission Planner Algorithm

Step 1 — Pop the highest-priority patient from the priority queue.

Step 2 — Hard filter (not scored, excluded outright):
  - drone.status == Idle
  - totalDistance(drone, patient) <= range(drone.batteryPercent)
  where totalDistance = Distance(drone, patient) + Distance(patient, nearestHospital)
  and range = (batteryPercent / 100) * MAX_RANGE

Step 3 — Score remaining feasible candidates:
  batteryUtilization = totalDistance / range   // this IS the battery penalty —
                                                 // continuous ratio, not a flat
                                                 // cutoff. Near 0 = plenty of spare
                                                 // range. Near 1 = barely feasible,
                                                 // heavily penalized even though
                                                 // still technically feasible.
  riskFactor = RiskFactor(patient.location)     // fire-proximity = high, road = low
  Score = W1*totalDistance + W2*batteryUtilization + W3*riskFactor
  Lowest score wins. Make W1/W2/W3 tunable serialized fields, not hardcoded 
  constants, so they can be calibrated later without recompiling logic.

Step 4 — Dispatch: mark drone Busy, hand off to Navigation Engine (drone→patient, 
then patient→hospital), mark Idle (or Charging if battery is now low) on completion.

## Dynamic Events — Re-invocation Only, No New Algorithms

- BatteryLow(drone): drone.status = Charging; MissionPlanner.Reassign(drone.currentTask)
- FireSpread(zone): update obstacle map; re-run Voronoi for every active drone's 
  remaining route (do NOT re-run Mission Planner scoring for this — only re-path)
- DroneFailed(drone): drone.status = Offline; MissionPlanner.Reassign(drone.currentTask)
- NewEmergency(patient): priorityQueue.Push(patient)

Each of these must call back into the SAME Mission Planner / Navigation Engine 
functions used for normal dispatch — do not write separate handling logic per event.

============================================================
PART 2 — NAVIGATION ENGINE (port from Stage 1, don't rewrite)
============================================================

Before writing any new pathfinding/avoidance code: search this project via MCP for 
an existing script resembling "PathManager.cs" or similar Stage 1 navigation code. 
If found, PORT its logic — do not reimplement from scratch. If not found in this 
project, here is the exact logic to reconstruct, based on the validated Stage 1 
implementation:

- Grid-based obstacle map with a BFS "brushfire" distance-transform 
  (ComputeClearanceMap) giving each cell's distance to the nearest obstacle.
- Global path search: A*/Dijkstra over the grid where each node's priority is 
  gCost + clearancePenalty, with clearancePenalty = max(0, gridWidth - clearance) 
  * clearanceWeight. This is a CLEARANCE-WEIGHTED PATH SEARCH, not literal Voronoi 
  skeleton/roadmap extraction — name the function accordingly, e.g. 
  FindClearanceWeightedPath(), and note in a code comment that this approximates 
  Voronoi-style maximum-clearance routing without extracting explicit Voronoi edges.
- Local avoidance: each moving agent computes preferred velocity toward its next 
  waypoint, then adds a repulsion vector (inverse-distance-weighted, summed over 
  nearby agents and dynamic obstacles), then clamps to max speed. This is an 
  APF-STYLE reciprocal avoidance heuristic, not the literal ORCA velocity-obstacle 
  linear program — name the function accordingly, e.g. 
  ComputeReciprocalAvoidanceVelocity(), with a comment noting this is 
  ORCA-inspired, not a literal implementation of the half-plane LP from van den 
  Berg et al. (2011).
- IMPORTANT — isolate this behind a clean interface regardless of the above: 
  `Vector3 ComputeAvoidanceVelocity(AgentData self, List<AgentData> neighbors, 
  List<Obstacle> obstacles)` — implementation detail hidden behind this signature, 
  so a true ORCA implementation (e.g. porting RVO2) can be swapped in later 
  without touching any calling code.

## Required Refactor Structure

Split any ported/reconstructed logic into these files — no GameObject/visualization 
coupling in the logic itself:

- `Scripts/Navigation/DisasterGrid.cs` — owns grid + clearance map as pure data 
  (no per-tile GameObjects by default). Built once at scene load. Obstacle list 
  mutates and clearance map recomputes ONLY on a FireSpread event, not per-dispatch.
- `Scripts/Navigation/NavigationEngine.cs` — two public entry points, no 
  visualization coupling:
    `List<Vector3> FindSafePath(Vector3 start, Vector3 goal)`
    `Vector3 ComputeAvoidanceVelocity(AgentData self, List<AgentData> neighbors, List<Obstacle> obstacles)`
  Callable repeatedly for arbitrary start/goal pairs and arbitrary agent counts — 
  NOT hardcoded to one start/end pair or one agent like the Stage 1 demo script.
- Tile visualization (colored ground tiles) is OPTIONAL and gated behind a debug 
  flag — do not instantiate thousands of tile GameObjects by default in the 
  Stage 2 scene; it doesn't scale to repeated dispatch calls.

============================================================
PART 3 — METRICS & LOGGING
============================================================

Port the Stage 1 timing/CSV infrastructure (Stopwatch-based timing, CSV writer) — 
it's already correct, just generalize it to log per-mission-run instead of one 
hardcoded run. Track and log, per run:

1. Mission Completion Time
2. Average Response Time (per patient: queue-entry → drone-arrival, averaged)
3. Rescue Success Rate (% patients delivered to hospital)
4. Battery Utilization at Mission End (fleet avg + min remaining battery)
5. Collision Count (should stay ~0 due to avoidance)
6. Number of Reassignments (count of Mission Planner re-invocations mid-mission)
7. Coverage (% patients actually reached, regardless of hospital delivery)
8. System Throughput (patients rescued per unit simulated time)

Also log a human-readable event feed as it happens (this doubles as your 
dashboard "Alerts Feed" data later): e.g. "Drone 3 battery low -> reassigned", 
"New patient detected -> queued", "Drone 5 failed -> task reassigned to Drone 1".

============================================================
PART 4 — BUILD PHASES (work through IN ORDER, stop and report after each)
============================================================

Do not skip ahead. After completing each phase, STOP, summarize what you built 
and how to verify it, and wait for confirmation before starting the next phase.

PHASE 0 — Setup
- Confirm/create folder structure: Scripts/Navigation, Scripts/MissionPlanner, 
  Scripts/Drone, Scripts/Environment, Scenes
- Locate and port Stage 1 navigation code per Part 2 above
- Checkpoint: DisasterGrid.cs and NavigationEngine.cs exist and compile

PHASE 1 — Static Scene & Data Model
- Build scene: ground plane, placeholder box obstacles, marked hospital and 
  charging-station locations
- Implement Patient and Drone data classes exactly as specified in Part 1
- Spawn 6 drone placeholders and 3-4 patient markers
- Checkpoint: scene loads with correct object counts, nothing needs to move yet

PHASE 2 — Single-Drone Navigation
- Wire NavigationEngine.FindSafePath to one drone, one hardcoded patient, no 
  Mission Planner yet
- Checkpoint: one drone follows a sensible, obstacle-clearing route

PHASE 3 — Multi-Drone Local Avoidance
- Layer ComputeAvoidanceVelocity on top for 2-3 simultaneously moving drones on 
  crossing paths
- Checkpoint: multiple drones navigate at once with no collisions

PHASE 4 — Mission Planner: Filtering Only
- Implement live DroneList and Patient priority queue
- Implement Step 2 hard filters only (no scoring yet)
- Checkpoint: manually verify a low-battery or busy drone gets correctly excluded

PHASE 5 — Mission Planner: Scoring & Dispatch (Review 2 target)
- Implement full scoring (Part 1) and SelectBestDrone -> DispatchDrone -> 
  NavigationEngine handoff
- Run full autonomous scenario: earthquake trigger -> patients queued -> drones 
  scored and dispatched -> navigate -> deliver -> return to Idle, with NO manual 
  triggering required
- Checkpoint: this is the Review 2 demo — press play, everything runs autonomously

PHASE 6 — Dynamic Events (Review 3 target starts here)
- Implement all four event handlers from Part 1, each re-invoking existing 
  Mission Planner / NavigationEngine functions only
- Test each independently via a debug trigger, then together in one scenario
- Checkpoint: all four events triggerable live and visibly handled correctly

PHASE 7 — Logging & Metrics
- Implement all 8 metrics and the event feed log per Part 3
- Checkpoint: a completed run produces a correct CSV + readable event log

PHASE 8 — Web Dashboard
- Unity writes fleet/queue/alert state to a JSON file every ~200-500ms
- Simple HTML/JS page polls that file, renders: Fleet Status table, 2D map 
  (canvas dots mirroring positions), Alerts feed
- Add Assignment Explainability panel last (shows the 3 score factors + winning 
  score for the most recent dispatch)
- Post-mission Analytics view renders Phase 7's data as a table/chart
- Checkpoint: dashboard reflects a live Unity run in a browser

PHASE 9 — Baseline & Polish
- Add a toggle: nearest-idle-drone-only mode (bypasses scoring) for baseline 
  comparison runs
- Run identical scenarios in both modes, confirm both log correctly
- Checkpoint: ready for full demo rehearsal

============================================================
PART 5 — WORKING STYLE
============================================================

-Commit to git after every checkpoint passes, and push to GitHub too, with a clear message naming the 
  phase completed. Initialize git now if not already present.
- Never touch or modify the Stage 1 project/scene if it exists in this repo — 
  Stage 2 code is new/separate, ported logic only, source files untouched.
- If Unity project structure, existing assets, or naming conventions are unclear, 
  ask before creating conflicting/duplicate structures — check via MCP first.
- Prefer porting/adapting existing working code over rewriting from scratch.
- After each phase, tell me explicitly how to manually verify it worked (what to 
  press play and look for), not just "done."

START with Phase 0 now: inspect the current project via MCP, report what already 
exists, then proceed.
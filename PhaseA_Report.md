# ECEN 723 Spring 2026 — Project Phase A Report

**Team ID:** _(fill in)_
**Group:** i-group & v-group
**Members:** _(fill in)_
**Date:** April 3, 2026

---

## 1. Code Structure

The implementation is organized into **three files**, cleanly separating shared infrastructure from each group's logic:

| File | Role |
|---|---|
| `common_model.py` | Shared data structures, road graph, and transition validators |
| `i_group_phaseA.py` | Infrastructure simulator — traffic lights and crossing arbitration |
| `v_group_phaseA.py` | Vehicle simulator — movement, routing, and tour completion |

Each road between two nodes is modeled as **two independent directed segments** (e.g., `I00_to_I01` EAST and `I01_to_I00` WEST), with **slot 0** at the entry and **slot 29** at the intersection approach.

---

## 2. Shared Data Model (`common_model.py`)

### Key Data Classes

| Class | Purpose |
|---|---|
| `VehicleState` | Position (`segment` + `slot`), visit flags (B/C/D), crossing request and intent |
| `IntersectionLightState` | Which direction (if any) currently holds the green light |
| `CrossingRequest` | A vehicle declaring its intent to cross and desired next segment |
| `CrossingGrant` | Infrastructure's approval or denial of a specific crossing request |
| `Segment` | A directed road segment with `from_node`, `to_node`, `direction`, and `length_slots` |

### Transition Validators

Three shared helper functions enforce movement legality across both groups:

- **`is_valid_crossing_transition`** — outgoing segment must originate at the node where the incoming segment terminates
- **`is_u_turn_transition`** — outgoing segment must not return to the node the vehicle came from
- **`is_right_turn_transition`** — outgoing direction must not be a right turn of the incoming direction (EAST→SOUTH, SOUTH→WEST, WEST→NORTH, NORTH→EAST)

---

## 3. i-group: Infrastructure Simulator

### 3.1 Architecture

The i-group is built around two classes:

- **`IntersectionController`** — one instance per intersection; owns the light state and a per-direction **starvation counter** to prevent indefinite waiting
- **`InfrastructureSimulator`** — the top-level coordinator; drives the full per-step pipeline and accumulates a `SafetyReport`

### 3.2 Algorithm

Each call to `step(vehicles)` runs the following pipeline:

**Step 1 — Identify waiting vehicles**
`get_incoming_waiting_vehicles()` collects vehicles that are at **slot 29**, have `request_crossing = True`, and whose segment ends at an intersection.

**Step 2 — Update traffic lights**
For each intersection, `count_waiting_by_direction()` groups waiting vehicles by direction. `select_green_direction()` picks the winner using:

```
score(direction) = waiting_count(direction) + starvation_counter(direction)
```

Non-winning directions with waiting vehicles increment their starvation counter; the winner resets to 0.

**Step 3 — Validate and build crossing requests**
`build_crossing_requests()` validates each vehicle's request — correct slot, valid transition, no U-turn, no right turn — and constructs a `CrossingRequest` per passing vehicle.

**Step 4 — Grant one crossing per intersection**
`select_one_grant()` filters valid requests to those matching the **green direction** and issues exactly **one grant**, breaking ties by `car_id` (alphabetical).

**Step 5 — Compute congestion map**
Counts vehicles with `stopped = True` on segments feeding each intersection.

**Step 6 — Safety check**
`check_safety()` verifies all active constraints and populates the `SafetyReport`.

### 3.3 Output (per step)

```python
{
    "time_step": int,
    "lights": {"I00": "east" | "north" | "south" | "west" | None, ...},
    "crossing_grants": [{"intersection_id": str, "car_id": str, "granted": bool}, ...],
    "congestion_map": {"I00": int, ...},
    "safety_report": {
        "collisions": int,
        "red_light_violations": int,
        "simultaneous_green_violations": int,
        "invalid_grant_violations": int,
        "wrong_direction_violations": int,
        "u_turn_violations": int,
        "right_turn_violations": int,
    }
}
```

### 3.4 Safety Properties Verified

| Property | Mechanism |
|---|---|
| No collisions | `detect_collisions()` — scans all (segment, slot) pairs for duplicates |
| No red-light violations | `check_safety()` — every grant must align with the green direction |
| At most 1 green per intersection | Checked across all controllers in `check_safety()` |
| At most 1 crossing per step | `select_one_grant()` — returns at most one grant per intersection |
| No U-turns | `validate_request()` — calls `is_u_turn_transition()` |
| No right turns | `validate_request()` — calls `is_right_turn_transition()` |
| No wrong-direction transitions | `validate_request()` — calls `is_valid_crossing_transition()` |

---

## 4. v-group: Vehicle Simulator

### 4.1 Architecture

The v-group is built around two classes:

- **`RoutePlanner`** — stateless; selects the best next segment given current position, target node, and congestion map
- **`VehicleSimulator`** — maintains all vehicle states; processes one time step per call and tracks tour completions and safety violations

### 4.2 Algorithm

Each call to `step(i_group_output)` runs in **two phases**:

**Phase 1 — `prepare_requests(congestion_map)`**

For each vehicle:
1. **Blocking check** (`is_front_blocked()`): if the nearest car ahead in the same segment is within **15 slots** (0.5 mile), the vehicle stays and clears its request.
2. **Advance inside segment**: if not at slot 29, increment slot by 1.
3. **At a terminal** (A/B/C/D): record the visit and immediately enter the next outgoing segment from `RoutePlanner`.
4. **At slot 29 toward an intersection**: call `build_crossing_request()` — pick a next segment via `RoutePlanner` and set `request_crossing = True`.

After all vehicles are processed, a **collision check** is run.

**Phase 2 — `apply_i_group_output(lights, crossing_grants)`**

For each vehicle with a pending crossing request:
1. Check for a matching grant in `crossing_grants`.
2. **If granted**: validate transition (no wrong direction, no U-turn, no right turn), verify light consistency, and move the vehicle to slot 0 of the new segment.
3. **If not granted**: vehicle stops at slot 29.

After crossings are applied, terminal visits are recorded, completed tours are detected and the vehicle is reset to start a new tour.

**Route Planning**

`choose_next_segment()` filters candidates to those passing all three validators, then scores each:

```
cost = manhattan_distance(next_node → target_node) × 2
     + congestion_penalty(next_node)
     + base_cost (10)
```

The **lowest-cost valid candidate** is selected.

### 4.3 Output (per step)

```python
{
    "time_step": int,
    "vehicles": {"car_id": {full vehicle snapshot}, ...},
    "vehicles_for_i_group": {"car_id": {snapshot}, ...},
    "stats": {
        "completed_tours": int,
        "red_light_violations": int,
        "collisions": int,
        "illegal_direction_violations": int,
        "u_turn_violations": int,
        "right_turn_violations": int,
    }
}
```

### 4.4 Safety Properties Verified

| Property | Mechanism |
|---|---|
| No collisions | `check_collision()` — scans all (segment, slot) pairs for duplicates |
| No red-light violations | `apply_intersection_result()` — checks light direction vs grant |
| No U-turns | `apply_intersection_result()` — calls `is_u_turn_transition()` |
| No right turns | `apply_intersection_result()` — calls `is_right_turn_transition()` |
| No wrong-direction transitions | `apply_intersection_result()` — calls `is_valid_crossing_transition()` |

---

## 5. i-group / v-group Integration Protocol

The two modules exchange the following data each time step:

| Direction | Payload | Description |
|---|---|---|
| v-group → i-group | `Dict[str, VehicleState]` | All vehicle positions, stopping status, crossing requests, and desired next segments |
| i-group → v-group | `lights` | Green direction (or `None`) per intersection |
| i-group → v-group | `crossing_grants` | One grant per intersection — identifies the specific car permitted to cross |
| i-group → v-group | `congestion_map` | Stopped vehicle count per intersection, used by the route planner |

**Why grants are necessary:** the `lights` signal only specifies which *direction* is green — it does not resolve which of multiple waiting cars from that direction actually crosses. The grant makes that decision explicit and allows both groups to independently verify compliance.

---

## 6. Design Discussion

This section documents the rationale behind key implementation decisions that are not immediately obvious from the specification alone.

### Two directed segments per road

Each bidirectional road is modeled as two independent directed segments rather than a single bidirectional object. This keeps the data model simple and uniform — every segment has exactly one direction, one origin node, and one destination node. Both groups operate on the same segment map without needing to resolve directionality at runtime, and all transition validators work without special-casing.

### Starvation counter in light selection

The specification requires that collisions and red-light violations be minimized, which implies traffic must flow. A purely queue-length-based light policy can permanently favor a high-traffic direction and starve a low-traffic one. The starvation counter adds a time-based bias: a direction that has been waiting without a green light accumulates weight each step, ensuring it will eventually be selected even if its queue is shorter. This is a deliberate design choice to balance throughput with fairness.

### Crossing grants alongside light signals

The `lights` output tells vehicles which direction is currently green, but it does not specify which individual vehicle may cross. When multiple vehicles from the same direction are queued at the same intersection, the light alone is insufficient — all of them would attempt to cross simultaneously, violating the at-most-one-crossing-per-step constraint. The `crossing_grants` signal resolves this by naming exactly one car per intersection per step. This also enables both groups to independently verify safety: the i-group checks that it only granted legal crossings, and the v-group checks that it only accepted a crossing when a grant was received.

### Two-phase vehicle step

The v-group separates each time step into two phases: first, vehicles advance and build crossing requests (`prepare_requests`); then, i-group output is applied to resolve intersections (`apply_i_group_output`). This mirrors the real-world asynchrony between vehicle decision-making and infrastructure response, and it ensures the snapshot sent to the i-group reflects vehicle positions before any crossing occurs — keeping the two groups' views of state consistent.

### 15-slot line-of-sight limit for blocking

The specification states that a vehicle can observe another vehicle ahead only within 0.5 mile and only if no third vehicle is between them. At 1/30 mile per slot, 0.5 mile corresponds to 15 slots. The blocking check therefore finds the nearest car ahead in the same segment and only halts the vehicle if that car is within 15 slots. A car more than 15 slots ahead is outside the observable range and does not impede movement.

### Alphabetical tie-breaking for grants

When multiple vehicles from the green direction are waiting at the same intersection, the i-group must deterministically select one. Alphabetical ordering by `car_id` is used because it is stable, reproducible across runs, and requires no additional state. This ensures that repeated executions with identical inputs produce identical outputs, which is important for verification and debugging.

### Right-turn prohibition in the route planner

Rather than detecting and penalizing right turns after the fact, the route planner filters them out during candidate selection. A right-turn segment is never even considered as a routing option. This approach enforces the constraint proactively and keeps the safety checker's role as a secondary audit rather than a primary enforcement mechanism. Both groups still track right-turn violations independently for cross-validation purposes.

### Congestion-aware routing heuristic

The route planner uses a Manhattan distance estimate over the intersection grid combined with a congestion penalty from the i-group's `congestion_map`. A full shortest-path search (e.g., Dijkstra) is deferred to Phase B integration. The heuristic is sufficient for Phase A because the grid is small (3x3 intersections), and the congestion penalty provides enough signal to avoid obviously blocked paths.

---

## 7. Test Results

### 6.1 i-group Test

**Setup:** Three vehicles placed across the grid:
- `car_1` — segment `A_to_I00`, slot 29, eastbound, requesting crossing to `I00_to_I01`
- `car_2` — segment `I10_to_I11`, slot 29, eastbound, requesting crossing to `I11_to_I12`
- `car_3` — segment `I01_to_I11`, slot 12, mid-segment, no crossing request

**Output (Step 1):**
```python
{
  'time_step': 1,
  'lights': {
    'I00': 'east',    # car_1 waiting → green east
    'I11': 'east',    # car_2 waiting → green east
    'I01': None, 'I02': None, 'I10': None,
    'I12': None, 'I20': None, 'I21': None, 'I22': None
  },
  'crossing_grants': [
    {'intersection_id': 'I00', 'car_id': 'car_1', 'granted': True},
    {'intersection_id': 'I11', 'car_id': 'car_2', 'granted': True}
  ],
  'congestion_map': {'I00': 1, 'I11': 1, ...all others 0},
  'safety_report': {
    'collisions': 0, 'red_light_violations': 0,
    'simultaneous_green_violations': 0, 'invalid_grant_violations': 0,
    'wrong_direction_violations': 0, 'u_turn_violations': 0,
    'right_turn_violations': 0
  }
}
```

**What this confirms:**
- Lights set to `east` exactly where eastbound vehicles are waiting
- Exactly one grant issued per intersection
- `car_3` at slot 12 correctly excluded — not at slot 29, no crossing request
- All safety counters zero — no violations

---

### 6.2 v-group Test

**Setup:** Two vehicles (`car_1`, `car_2`) both starting at `A_to_I00`, slot 0.

**Step 1 (no i-group input):**
```python
'vehicles': {
    'car_1': {'current_slot': 1, 'stopped': False, ...},
    'car_2': {'current_slot': 0, 'stopped': True, ...}
}
'stats': {'collisions': 0, 'red_light_violations': 0, ...all zeros}
```

**Step 2 (i-group input: I00=east, grant issued to car_1):**
```python
'vehicles': {
    'car_1': {'current_slot': 2, 'stopped': False, ...},
    'car_2': {'current_slot': 0, 'stopped': True, ...}
}
'stats': {'collisions': 0, 'red_light_violations': 0, ...all zeros}
```

**What this confirms:**
- `car_1` advances slot 0 → 1 → 2 correctly across two steps
- `car_2` stays stopped at slot 0 — correctly blocked by `car_1` ahead within 15 slots
- No violations across either step

---

## 7. Source Code

Source code is included in the submitted repository:

- `common_model.py` — shared data model and validators
- `i_group_phaseA.py` — infrastructure simulator
- `v_group_phaseA.py` — vehicle simulator

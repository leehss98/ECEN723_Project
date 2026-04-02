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

This section documents the rationale behind key implementation decisions through the lens of how the two groups negotiated their interface and resolved design trade-offs.

### Road representation

**i-group:** We need to know the direction a vehicle is approaching from to decide which light to set. A single bidirectional segment object would force us to resolve direction at runtime for every vehicle lookup.

**v-group:** Agreed. If every segment has exactly one direction baked in, the route planner can filter candidates by direction without any extra logic, and the transition validators stay simple.

**Decision:** Each road is modeled as two independent directed segments. Both groups operate on the same static segment map with no ambiguity.

---

### Why crossing grants are needed alongside light signals

**v-group:** We receive the `lights` output and know which direction is green. Can we just let any vehicle in that direction cross?

**i-group:** No — if two vehicles are both at slot 29 on the same green direction, you have no way to decide which one goes. Both would attempt to cross in the same step, which is a collision.

**v-group:** So the grant names the specific car. That also means we can verify independently: if we ever execute a crossing without a grant, we know something went wrong on our side.

**i-group:** Exactly. And if a grant goes to a vehicle that was not in the green direction, we flag that on our side. Both groups audit the same event from their own perspective.

**Decision:** The i-group issues one explicit grant per intersection per step. Both groups track violations independently; their counts must match.

---

### Starvation counter in light selection

**v-group:** If a low-traffic direction never gets a green light, vehicles heading that way will block forever. That hurts throughput and creates deadlocks.

**i-group:** A pure queue-length policy would always favor the busiest direction. We added a starvation counter that accumulates each step a direction is skipped. It biases the selection score toward directions that have been waiting longer, even if their queue is smaller.

**Decision:** Light selection scores each direction as `waiting_count + starvation_counter`. The counter resets to zero when a direction is granted and increments otherwise.

---

### Two-phase vehicle step

**i-group:** We need a consistent snapshot of all vehicle positions before we compute lights and grants. If vehicles are still moving when we read them, our decisions will be based on stale or partially-updated state.

**v-group:** We handle this by splitting each step into two phases. In Phase 1, vehicles advance within their segments and submit crossing requests — that snapshot goes to you. In Phase 2, we apply your response to resolve intersections. You never see a vehicle mid-crossing.

**Decision:** `prepare_requests()` runs first and produces the snapshot for the i-group. `apply_i_group_output()` runs second and consumes the i-group response. The two phases are never interleaved.

---

### 15-slot line-of-sight limit for blocking

**i-group:** The spec says a vehicle can see another car within 0.5 mile with no car in between. That is 15 slots. Beyond that, it is outside the vehicle's observable range.

**v-group:** The original blocking check stopped a vehicle if any car was ahead in the segment, regardless of distance. That was too conservative — a car 25 slots ahead should not block movement.

**Decision:** `is_front_blocked()` finds the nearest car ahead in the same segment and only blocks if the gap is 15 slots or fewer.

---

### Alphabetical tie-breaking for grants

**v-group:** When multiple cars are waiting from the same green direction, which one do you pick?

**i-group:** We sort by `car_id` alphabetically and take the first. It is deterministic, requires no additional state, and makes repeated runs with the same input produce the same output — which matters for verification.

**Decision:** Grants are awarded to the lexicographically first eligible `car_id`.

---

### Right-turn prohibition

**v-group:** We enforce this at the routing stage — the route planner never selects a right-turn segment as a candidate. The constraint is applied before a request is even formed.

**i-group:** We enforce it again during request validation and in the safety check. If a right-turn request somehow arrives, we reject it and log a violation.

**Decision:** Right turns are blocked proactively in the route planner and audited defensively in both groups' safety checks.

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

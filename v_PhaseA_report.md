# ECEN 723 Spring 2026: Project Phase A Report (v-group)

**Team ID:** _17_
**Group:** v-group
**Members:** _Vishnu Rajagopal, Stephen Muttathil_
**Date:** April 3, 2026

---

## 1. Code Structure

The implementation is organized into three files, cleanly separating shared state from each group's logic:

| File | Role |
|---|---|
| `common_model.py` | Shared data structures, road graph, and transition predicates |
| `i_group_phaseA.py` | Infrastructure simulator: light arbitration and intersection access control |
| `v_group_phaseA.py` | Vehicle simulator: movement, routing, and tour completion |

Each road between two nodes is modeled as two independent directed segments (e.g., `I00_to_I01` EAST and `I01_to_I00` WEST), with slot 0 at the entry and slot 29 at the intersection approach. The system advances in discrete time steps, where each step corresponds to a synchronized state transition across all vehicles and intersections.

---

## 2. Shared Data Model (`common_model.py`)

### Key Data Classes

| Class | Purpose |
|---|---|
| `VehicleState` | Encodes the full state of a vehicle: position (`segment` + `slot`), visit flags (B/C/D), and pending crossing intent |
| `IntersectionLightState` | Records which direction (if any) currently holds the green light at an intersection |
| `CrossingRequest` | A vehicle's wait on the intersection semaphore; declares intent to enter the critical section |
| `CrossingGrant` | The infrastructure's signal; releases the semaphore for exactly one vehicle per intersection per step |
| `Segment` | A directed road segment with `from_node`, `to_node`, `direction`, and `length_slots` |

### Transition Predicates

Three shared predicates enforce movement legality and are evaluated by both groups independently:

- **`is_valid_crossing_transition`**: outgoing segment must originate at the node where the incoming segment terminates; violations indicate an illegal state transition
- **`is_u_turn_transition`**: outgoing segment must not return to the node the vehicle came from; enforces the no-U-turn invariant
- **`is_right_turn_transition`**: outgoing direction must not be a right turn of the incoming direction (EAST to SOUTH, SOUTH to WEST, WEST to NORTH, NORTH to EAST); enforces the no-right-turn invariant

---

## 3. v-group: Vehicle Simulator

### 3.1 Architecture

The v-group is built around two classes:

- **`RoutePlanner`**: a stateless decision procedure; selects the next segment given current position, target node, and the congestion map
- **`VehicleSimulator`**: maintains the global vehicle state; advances the system by one time step per call and independently tracks all invariant violations

### 3.2 Algorithm

Each call to `step(i_group_output)` executes in two atomic phases, mirroring a two-phase synchronization protocol:

**Phase 1: `prepare_requests(congestion_map)`**

For each vehicle:

1. **Blocking predicate** (`is_front_blocked()`): if the nearest vehicle ahead in the same segment is within 15 slots (0.5 mile), the process blocks and clears its pending request.
2. **Internal state transition**: if not at slot 29, advance one slot.
3. **Terminal node**: record the visit and immediately execute the next segment transition chosen by the decision procedure.
4. **Intersection approach** (slot 29): invoke `build_crossing_request()`; the decision procedure selects a next segment and the vehicle issues a wait on the intersection semaphore by setting `request_crossing = True`.

A collision check (mutual exclusion audit) is run after all vehicles are processed.

**Phase 2: `apply_i_group_output(lights, crossing_grants)`**

For each vehicle holding a pending semaphore wait:

1. Check whether the i-group has issued a signal (grant) for this vehicle.
2. **If signaled**: evaluate all three transition predicates; if all pass, execute the crossing; the vehicle transitions to slot 0 of the new segment.
3. **If not signaled**: the vehicle remains blocked at slot 29, awaiting the next arbitration cycle.

After crossings are resolved, terminal visits are recorded, completed tours are detected, and any vehicle that has satisfied the liveness condition (visited all of B, C, D, and returned to A) is reset to begin a new tour.

**Decision Procedure (Route Planning)**

`choose_next_segment()` filters all candidate segments through the three transition predicates, then ranks the remainder by:

```
cost = manhattan_distance(next_node to target_node) x 2
     + congestion_penalty(next_node)
     + base_cost (10)
```

The lowest-cost admissible candidate is selected. This heuristic approximates a shortest-path search and is sufficient for Phase A; a complete decision procedure is planned for Phase B.

### 3.3 Output (per step)

```python
{
    "time_step": int,
    "vehicles": {"car_id": {full state snapshot}, ...},
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

### 3.4 Safety Invariants Verified

| Invariant | Mechanism |
|---|---|
| Mutual exclusion: no two vehicles share a (segment, slot) | `check_collision()` audits all position pairs after each phase |
| No red-light violation | `apply_intersection_result()` checks the guard condition against the received grant |
| No U-turn state transition | `apply_intersection_result()` evaluates `is_u_turn_transition()` |
| No right-turn state transition | `apply_intersection_result()` evaluates `is_right_turn_transition()` |
| No illegal segment transition | `apply_intersection_result()` evaluates `is_valid_crossing_transition()` |

---

## 4. i-group / v-group Integration Protocol

The two modules exchange the following signals each time step:

| Direction | Signal | Semantics |
|---|---|---|
| v-group to i-group | `Dict[str, VehicleState]` | Full system state snapshot: vehicle positions, blocking status, semaphore wait requests |
| i-group to v-group | `lights` | Active guard condition: the direction whose semaphore wait may proceed |
| i-group to v-group | `crossing_grants` | Semaphore signal: names the one vehicle per intersection that may execute its crossing |
| i-group to v-group | `congestion_map` | Load metric: stopped vehicle count per intersection, consumed by the decision procedure |

The protocol guarantees that each step is processed against a consistent global snapshot: the v-group freezes vehicle state before sending it to the i-group, and no crossing is executed until the i-group's semaphore signals are received and applied.

---

## 5. Design Discussion

This section documents the rationale behind key implementation decisions through the lens of how the two groups negotiated their shared protocol and resolved design trade-offs.

### Road representation

**i-group:** We need to evaluate the guard condition based on the direction a vehicle is approaching from. A single bidirectional segment object would require direction resolution at runtime on every state read.

**v-group:** Agreed. If every segment encodes exactly one direction, the decision procedure can filter candidates statically and the transition predicates remain simple boolean functions.

**Decision:** Each road is represented as two independent directed segments. Both groups share a single static segment map with no runtime ambiguity.

---

### Intersection as a critical section: the semaphore model

**v-group:** We receive the `lights` output and know which direction satisfies the guard condition. Can we allow any vehicle in that direction to execute the crossing?

**i-group:** No; if two vehicles are both blocked at slot 29 on the same green direction, both satisfy the guard condition simultaneously. Without an explicit arbitration signal, both would attempt to enter the critical section in the same step, violating mutual exclusion and producing a collision.

**v-group:** So each intersection is effectively a critical section, and the `crossing_grant` is a binary semaphore signal; it names the one process that acquires the resource for this step.

**i-group:** Exactly. The `lights` output is the guard condition; the grant is the semaphore signal. A vehicle issues a wait by setting `request_crossing = True`; the i-group responds with a signal for exactly one vehicle per intersection. Both groups then independently verify that no invariant was violated; the i-group checks that only the granted vehicle executed the crossing; the v-group checks that it only crossed when it held a valid signal.

**Decision:** Each intersection is modeled as a binary semaphore. The i-group controls the semaphore and issues at most one signal per step. Both groups audit the same events independently; their violation counts must match.

---

### Starvation-freedom in light selection

**v-group:** A purely queue-length-based arbitration policy can permanently favor high-traffic directions. Vehicles on low-traffic approaches would never acquire the semaphore; the system would exhibit starvation.

**i-group:** We address this with a starvation counter per direction. Each step a direction is denied the semaphore, its counter increments. The selection function scores each direction as `waiting_count + starvation_counter`, so a direction that has waited long enough will eventually outrank a busier one. This guarantees the starvation-freedom property: every waiting vehicle is eventually granted access.

**Decision:** The scoring function combines queue length with accumulated wait time. The counter resets on grant and increments otherwise, enforcing fairness without requiring a full scheduling proof at this phase.

---

### Two-phase synchronization protocol

**i-group:** We require a consistent global snapshot of vehicle positions before computing lights and grants. If vehicles were still transitioning when we read state, our guard condition evaluation would be based on a partially-updated system state.

**v-group:** We enforce this with a two-phase step. Phase 1 advances all vehicles within their segments and freezes the state into a snapshot; this is what we send to you. Phase 2 applies your semaphore signals to resolve intersection crossings. The two phases are never interleaved; you always observe a stable state.

**Decision:** `prepare_requests()` produces a consistent snapshot before any semaphore signal is consumed. `apply_i_group_output()` applies signals only after the snapshot has been transmitted. This mirrors a two-phase commit protocol for shared state.

---

### 15-slot observability limit

**i-group:** The specification constrains a vehicle's observable state: it can detect another vehicle ahead only within 0.5 mile (15 slots) and only if no third vehicle is between them.

**v-group:** The original blocking predicate was over-conservative; it treated any vehicle ahead in the same segment as blocking, regardless of distance. A vehicle 25 slots ahead is outside the observable range and should not affect the blocking decision.

**Decision:** `is_front_blocked()` identifies the nearest vehicle ahead in the same segment and evaluates the predicate only if the gap is at most 15 slots. This correctly models the bounded observability constraint from the specification.

---

### Deterministic arbitration for semaphore signals

**v-group:** When multiple vehicles satisfy the guard condition at the same intersection, the arbitration must be deterministic to allow reproducible verification runs.

**i-group:** We order candidates lexicographically by `car_id` and select the first. This requires no additional state, produces identical outputs for identical inputs, and makes counterexample traces fully reproducible; a property that will be important in Phase C verification.

**Decision:** Semaphore signals are awarded to the lexicographically first eligible `car_id`.

---

### Right-turn prohibition: proactive vs. defensive enforcement

**v-group:** The no-right-turn constraint is enforced proactively in the decision procedure; a right-turn segment is never admitted as a candidate. The invariant is upheld before a semaphore wait is even issued.

**i-group:** We apply a second, defensive check during request validation and in the safety audit. If a right-turn request arrives, due to a bug or an unanticipated code path, it is rejected and logged as a violation. This layered enforcement follows a defense-in-depth approach: the v-group prevents violations by construction; the i-group detects them if they occur.

**Decision:** Right-turn transitions are excluded from the decision procedure and independently audited by both groups' safety checkers.

---

## 6. Test Results

Two test scenarios exercise v-group behavior in isolation, followed by an end-to-end integration test. All outputs shown are actual program output.

---

### 6.1 v-group Standalone Test

**Setup:** Two vehicles starting at `A_to_I00`, slot 0, to exercise the blocking predicate and the two-phase protocol in isolation (no i-group input).

**Actual output:**

```
Step 1 (no i-group signals):
  car_1: seg=A_to_I00  slot=1  stopped=False  req=False
  car_2: seg=A_to_I00  slot=0  stopped=True   req=False
  stats: {completed_tours:0, red_light_violations:0, collisions:0,
          illegal_direction_violations:0, u_turn_violations:0, right_turn_violations:0}

Step 2 (i-group signal: I00=east, car_1 granted):
  car_1: seg=A_to_I00  slot=2  stopped=False  req=False
  car_2: seg=A_to_I00  slot=0  stopped=True   req=False
  stats: all zeros
```

**Observations:**

- `car_1` advances: slot 0 to 1 to 2 across two steps
- `car_2` blocks at slot 0 because `car_1` is within 15 slots ahead in the same segment; the blocking predicate fires
- The grant for `car_1` at I00 in Step 2 does not produce a crossing because `car_1` has not yet reached slot 29; the signal is correctly ignored

---

### 6.2 End-to-End Integration Test

**Setup:** Both simulators running together in a closed loop for 50 steps. The v-group snapshot is forwarded to the i-group each step and the i-group lights and grants are fed back to the v-group. Two vehicles start at `A_to_I00`, slot 0.

**Selected step trace (actual output):**

```
Step 1:
  car_1: seg=A_to_I00   slot=1   stopped=False  req=False
  car_2: seg=A_to_I00   slot=0   stopped=True   req=False
  lights: {}    grants: []

Step 29:
  car_1: seg=A_to_I00   slot=29  stopped=False  req=False
  car_2: seg=A_to_I00   slot=14  stopped=False  req=False
  lights: {}    grants: []
  (car_2 is now free; car_1 is exactly 15 slots ahead, at the visibility boundary)

Step 30:  [car_1 issues semaphore wait; i-group responds with grant]
  car_1: seg=A_to_I00   slot=29  stopped=True   req=True   next=I00_to_I01
  car_2: seg=A_to_I00   slot=14  stopped=True   req=False
  lights: {'I00': 'east'}
  grants: [{'intersection_id': 'I00', 'car_id': 'car_1', 'granted': True}]

Step 31:  [car_1 executes crossing; state transition to new segment]
  car_1: seg=I00_to_I01  slot=0   stopped=False  req=False
  car_2: seg=A_to_I00    slot=14  stopped=True   req=False
  lights: {'I00': 'east'}
  grants: [{'intersection_id': 'I00', 'car_id': 'car_1', 'granted': True}]

Step 48:  [car_2 has also crossed; both vehicles on I00_to_I01]
  car_1: seg=I00_to_I01  slot=17  stopped=False  req=False
  car_2: seg=I00_to_I01  slot=0   stopped=False  req=False
  lights: {'I00': 'east'}
  grants: [{'intersection_id': 'I00', 'car_id': 'car_2', 'granted': True}]

All violation counters: zero across both groups for all 50 steps
```

**Observations:**

- `car_1` reaches slot 29 after 29 steps (one slot per step, unblocked once `car_2` falls 15 slots behind)
- At step 30, the crossing request fires; the two-phase protocol ensures Phase 1 state is frozen before Phase 2 applies the grant
- At step 31, `car_1` transitions atomically to slot 0 of `I00_to_I01`; one step per crossing, exactly as required
- `car_2` follows the same path 17 steps later; its blocking predicate fires and releases correctly as the gap widens
- Both groups independently count zero violations across all 50 steps; their safety audit results agree

---

## 7. Source Code

Source code is included in the submitted repository:

- `common_model.py`: shared data model, road graph, and transition predicates
- `i_group_phaseA.py`: infrastructure simulator with semaphore-based intersection control
- `v_group_phaseA.py`: vehicle simulator with two-phase synchronization protocol

# ECEN 723 Spring 2026 — Project Phase A Report

**Team ID:** _(fill in)_  
**Group:** i-group & v-group  
**Members:** _(fill in)_  
**Date:** April 3, 2026

---

## 1. Code Structure

The implementation is split into three files:

- **`common_model.py`** — shared data structures and road graph used by both groups
- **`i_group_phaseA.py`** — infrastructure simulator (traffic lights, crossing arbitration)
- **`v_group_phaseA.py`** — vehicle simulator (movement, routing, tour tracking)

Each road between two nodes is modeled as two independent directed segments (e.g., `I00_to_I01` EAST and `I01_to_I00` WEST), with slot 0 at the entry and slot 29 at the intersection approach.

---

## 2. Shared Data Model (`common_model.py`)

### Key Data Classes

| Class | Fields | Purpose |
|---|---|---|
| `Node` | `node_id`, `node_type` | Intersection or terminal |
| `Segment` | `segment_id`, `from_node`, `to_node`, `direction`, `length_slots` | Directed road segment |
| `VehicleState` | `car_id`, `current_segment`, `current_slot`, `visited_B/C/D`, `current_target`, `stopped`, `request_crossing`, `desired_next_segment` | Complete vehicle state |
| `IntersectionLightState` | `intersection_id`, `green_direction` | Current light at one intersection |
| `CrossingRequest` | `car_id`, `intersection_id`, `incoming_segment`, `outgoing_segment` | Vehicle requests to cross |
| `CrossingGrant` | `intersection_id`, `car_id`, `granted` | Infrastructure approves/denies crossing |

### Helper Functions

- `is_valid_crossing_transition(segments, incoming, outgoing)` — checks that the outgoing segment starts at the same node the incoming segment ends at.
- `is_u_turn_transition(segments, incoming, outgoing)` — checks that the outgoing segment does not return to the node the vehicle came from.

---

## 3. i-group: Infrastructure Simulator

### 3.1 Architecture

The i-group is structured around two classes:

- **`IntersectionController`** — one per intersection; holds the current light state and a per-direction starvation counter.
- **`InfrastructureSimulator`** — top-level coordinator; holds all controllers, processes vehicle states each step, and produces lights, grants, congestion data, and safety reports.

### 3.2 Algorithm

Each call to `step(vehicles)` executes the following pipeline:

**Step 1 — Identify waiting vehicles**  
`get_incoming_waiting_vehicles()` scans all vehicle states. A vehicle is considered waiting at intersection X if:
- Its current segment ends at X (an intersection node),
- It is at slot 29 (last slot, intersection approach),
- `request_crossing = True`.

**Step 2 — Update traffic lights**  
For each intersection, `count_waiting_by_direction()` groups waiting vehicles by their incoming direction (N/S/E/W). Then `IntersectionController.select_green_direction()` selects the direction with the highest score:

```
score(direction) = waiting_count(direction) + starvation_counter(direction)
```

The winning direction gets the green light. All other directions with waiting vehicles increment their starvation counter; the winning direction resets to 0. This prevents indefinite starvation of less-busy directions.

**Step 3 — Build and validate crossing requests**  
`build_crossing_requests()` constructs a `CrossingRequest` for each vehicle that passes validation:
- Must be at slot 29 of a segment ending at an intersection.
- `desired_next_segment` must be a valid continuation (passes `is_valid_crossing_transition`).
- Must not be a U-turn.

**Step 4 — Grant one crossing per intersection**  
`select_one_grant()` filters requests to those matching the current green direction at each intersection and grants exactly one. Ties are broken deterministically by `car_id` (alphabetical order), ensuring at most 1 car crosses per intersection per step.

**Step 5 — Compute congestion map**  
Counts stopped vehicles (`vehicle.stopped = True`) on segments feeding each intersection.

**Step 6 — Safety verification**  
`check_safety()` verifies:
- At most one green direction per intersection.
- Every granted vehicle came from the green direction.
- No granted vehicle performs a wrong-direction or U-turn transition.
- No two vehicles share the same (segment, slot) — collision detection.

### 3.3 Output Format

```python
{
    "time_step": int,
    "lights": {"I00": "east" | "north" | "south" | "west" | None, ...},
    "crossing_grants": [
        {"intersection_id": str, "car_id": str, "granted": bool}, ...
    ],
    "congestion_map": {"I00": int, ...},
    "safety_report": {
        "collisions": int,
        "red_light_violations": int,
        "simultaneous_green_violations": int,
        "invalid_grant_violations": int,
        "wrong_direction_violations": int,
        "u_turn_violations": int,
    }
}
```

### 3.4 Safety Properties Verified by i-group

| Property | How verified |
|---|---|
| No collisions | `detect_collisions()` scans all (segment, slot) pairs for duplicates |
| No red-light violations | `check_safety()` verifies every grant aligns with green direction |
| At most 1 green per intersection | Checked in `check_safety()` across all controllers |
| At most 1 crossing per step | `select_one_grant()` returns at most one grant per intersection |
| No U-turns | `validate_request()` calls `is_u_turn_transition()` |
| No wrong-direction transitions | `validate_request()` calls `is_valid_crossing_transition()` |

---

## 4. v-group: Vehicle Simulator

### 4.1 Architecture

The v-group is structured around two classes:

- **`RoutePlanner`** — stateless helper; selects the best next segment for a vehicle given its current position, target, and congestion map.
- **`VehicleSimulator`** — maintains all vehicle states and processes one time step per call; tracks tour completions and safety violation counts.

### 4.2 Algorithm

Each call to `step(i_group_output)` has two phases:

**Phase 1 — `prepare_requests(congestion_map)`**

For each vehicle:
1. **Blocking check** (`is_front_blocked()`): if another vehicle occupies any slot ahead in the same segment, the vehicle stays and clears its crossing request.
2. **Advance inside segment**: if not at slot 29, increment slot by 1.
3. **At a terminal node** (A/B/C/D): record the visit and immediately enter the next outgoing segment chosen by `RoutePlanner`.
4. **At an intersection approach** (slot 29): call `build_crossing_request()` to pick a next segment via `RoutePlanner` and set `request_crossing = True`.

After all vehicles are updated, a collision check is run.

**Phase 2 — `apply_i_group_output(lights, crossing_grants)`**

For each vehicle with `request_crossing = True` heading to an intersection:
1. Check if the vehicle has a grant in `crossing_grants`.
2. If granted: validate the transition (no wrong direction, no U-turn); check light direction consistency; move vehicle to slot 0 of the new segment.
3. If not granted: vehicle stays stopped at slot 29.

After crossings are applied, visits to terminals are recorded and completed tours are detected and reset.

**Route Planning (`RoutePlanner`)**

`choose_next_segment()` filters candidates using `is_valid_crossing_transition` and `is_u_turn_transition`, then scores each by `estimate_cost()`:

```
cost = grid_manhattan_distance(next_node, target_node) × 2
     + congestion_penalty(next_node)
     + base_cost (10)
```

The lowest-cost valid candidate is selected.

### 4.3 Output Format

```python
{
    "time_step": int,
    "vehicles": {
        "car_id": {
            "current_segment": str,
            "current_slot": int,
            "visited_B": bool, "visited_C": bool, "visited_D": bool,
            "current_target": str,
            "stopped": bool,
            "request_crossing": bool,
            "desired_next_segment": str | None,
        }, ...
    },
    "vehicles_for_i_group": { ... },   # snapshot sent to i-group
    "stats": {
        "completed_tours": int,
        "red_light_violations": int,
        "collisions": int,
        "illegal_direction_violations": int,
        "u_turn_violations": int,
    }
}
```

### 4.4 Safety Properties Verified by v-group

| Property | How verified |
|---|---|
| No collisions | `check_collision()` scans all (segment, slot) pairs for duplicates |
| No red-light violations | `apply_intersection_result()` checks light direction vs grant |
| No U-turns | `apply_intersection_result()` calls `is_u_turn_transition()` |
| No wrong-direction transitions | `apply_intersection_result()` calls `is_valid_crossing_transition()` |

---

## 5. i-group / v-group Integration Protocol

The two modules exchange the following data each time step:

| Direction | Data | Description |
|---|---|---|
| v-group → i-group | `Dict[str, VehicleState]` | All vehicle positions, stopping status, crossing requests, desired next segments |
| i-group → v-group | `lights` | Green direction (or None) per intersection |
| i-group → v-group | `crossing_grants` | One grant per intersection indicating which car may cross |
| i-group → v-group | `congestion_map` | Stopped vehicle count per intersection |

The grant signal is necessary because multiple vehicles may be waiting at the same intersection from the same (green) direction. The grant disambiguates exactly one car to cross, enforcing the at-most-1-crossing-per-step rule from both sides independently.

---

## 6. Test Results

### 6.1 i-group Test

Three vehicles placed at various positions:
- `car_1`: segment `A_to_I00`, slot 29, heading EAST toward I00, requesting crossing to `I00_to_I01`
- `car_2`: segment `I10_to_I11`, slot 29, heading EAST toward I11, requesting crossing to `I11_to_I12`
- `car_3`: segment `I01_to_I11`, slot 12, mid-segment, not requesting crossing

**Output (Step 1):**
```python
{
  'time_step': 1,
  'lights': {
    'I00': 'east',   # car_1 waiting eastbound → green east
    'I01': None,
    'I02': None,
    'I10': None,
    'I11': 'east',   # car_2 waiting eastbound → green east
    'I12': None,
    'I20': None,
    'I21': None,
    'I22': None
  },
  'crossing_grants': [
    {'intersection_id': 'I00', 'car_id': 'car_1', 'granted': True},
    {'intersection_id': 'I11', 'car_id': 'car_2', 'granted': True}
  ],
  'congestion_map': {
    'I00': 1, 'I01': 0, 'I02': 0,
    'I10': 0, 'I11': 1, 'I12': 0,
    'I20': 0, 'I21': 0, 'I22': 0
  },
  'safety_report': {
    'collisions': 0,
    'red_light_violations': 0,
    'simultaneous_green_violations': 0,
    'invalid_grant_violations': 0,
    'wrong_direction_violations': 0,
    'u_turn_violations': 0
  }
}
```

**Observations:**
- Lights set correctly to `east` at intersections with eastbound waiting vehicles (I00, I11); all others `None`.
- Grants issued to exactly one car per intersection.
- All safety counters zero — no violations.
- car_3 at slot 12 correctly excluded (not at slot 29, not requesting crossing).

### 6.2 v-group Test

Two vehicles (`car_1`, `car_2`) both starting at `A_to_I00`, slot 0.

**Step 1 (no i-group input):**
```python
{
  'time_step': 1,
  'vehicles': {
    'car_1': {'current_segment': 'A_to_I00', 'current_slot': 1, 'stopped': False, ...},
    'car_2': {'current_segment': 'A_to_I00', 'current_slot': 0, 'stopped': True, ...}
  },
  'stats': {'completed_tours': 0, 'red_light_violations': 0, 'collisions': 0,
            'illegal_direction_violations': 0, 'u_turn_violations': 0}
}
```

**Step 2 (with i-group output: I00=east granted to car_1):**
```python
{
  'time_step': 2,
  'vehicles': {
    'car_1': {'current_segment': 'A_to_I00', 'current_slot': 2, 'stopped': False, ...},
    'car_2': {'current_segment': 'A_to_I00', 'current_slot': 0, 'stopped': True, ...}
  },
  'stats': {'completed_tours': 0, 'red_light_violations': 0, 'collisions': 0,
            'illegal_direction_violations': 0, 'u_turn_violations': 0}
}
```

**Observations:**
- `car_1` advances from slot 0 → 1 → 2 correctly.
- `car_2` correctly stays stopped at slot 0 because `car_1` is blocking ahead in the same segment.
- No violations across either step.
- The front-blocking rule is working: `car_2` cannot advance while `car_1` occupies a slot ahead.

---

## 7. Source Code

### `common_model.py`

```python
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Dict, Optional


class NodeType(Enum):
    INTERSECTION = "intersection"
    TERMINAL = "terminal"


class Direction(Enum):
    NORTH = "north"
    SOUTH = "south"
    EAST = "east"
    WEST = "west"


@dataclass(frozen=True)
class Node:
    node_id: str
    node_type: NodeType


@dataclass(frozen=True)
class Segment:
    segment_id: str
    from_node: str
    to_node: str
    direction: Direction
    length_slots: int = 30


@dataclass
class VehicleState:
    car_id: str
    current_segment: str
    current_slot: int
    visited_B: bool
    visited_C: bool
    visited_D: bool
    current_target: str
    stopped: bool
    request_crossing: bool = False
    desired_next_segment: Optional[str] = None


@dataclass
class IntersectionLightState:
    intersection_id: str
    green_direction: Optional[Direction] = None


@dataclass
class CrossingRequest:
    car_id: str
    intersection_id: str
    incoming_segment: str
    outgoing_segment: str


@dataclass
class CrossingGrant:
    intersection_id: str
    car_id: str
    granted: bool


@dataclass
class SimulationState:
    time_step: int
    vehicles: Dict[str, VehicleState]
    lights: Dict[str, IntersectionLightState]


NODES: Dict[str, Node] = {
    "I00": Node("I00", NodeType.INTERSECTION),
    "I01": Node("I01", NodeType.INTERSECTION),
    "I02": Node("I02", NodeType.INTERSECTION),
    "I10": Node("I10", NodeType.INTERSECTION),
    "I11": Node("I11", NodeType.INTERSECTION),
    "I12": Node("I12", NodeType.INTERSECTION),
    "I20": Node("I20", NodeType.INTERSECTION),
    "I21": Node("I21", NodeType.INTERSECTION),
    "I22": Node("I22", NodeType.INTERSECTION),
    "A": Node("A", NodeType.TERMINAL),
    "B": Node("B", NodeType.TERMINAL),
    "C": Node("C", NodeType.TERMINAL),
    "D": Node("D", NodeType.TERMINAL),
}

SEGMENTS: Dict[str, Segment] = {
    "I00_to_I01": Segment("I00_to_I01", "I00", "I01", Direction.EAST),
    "I01_to_I00": Segment("I01_to_I00", "I01", "I00", Direction.WEST),
    "I01_to_I02": Segment("I01_to_I02", "I01", "I02", Direction.EAST),
    "I02_to_I01": Segment("I02_to_I01", "I02", "I01", Direction.WEST),
    "I10_to_I11": Segment("I10_to_I11", "I10", "I11", Direction.EAST),
    "I11_to_I10": Segment("I11_to_I10", "I11", "I10", Direction.WEST),
    "I11_to_I12": Segment("I11_to_I12", "I11", "I12", Direction.EAST),
    "I12_to_I11": Segment("I12_to_I11", "I12", "I11", Direction.WEST),
    "I20_to_I21": Segment("I20_to_I21", "I20", "I21", Direction.EAST),
    "I21_to_I20": Segment("I21_to_I20", "I21", "I20", Direction.WEST),
    "I21_to_I22": Segment("I21_to_I22", "I21", "I22", Direction.EAST),
    "I22_to_I21": Segment("I22_to_I21", "I22", "I21", Direction.WEST),
    "I00_to_I10": Segment("I00_to_I10", "I00", "I10", Direction.SOUTH),
    "I10_to_I00": Segment("I10_to_I00", "I10", "I00", Direction.NORTH),
    "I10_to_I20": Segment("I10_to_I20", "I10", "I20", Direction.SOUTH),
    "I20_to_I10": Segment("I20_to_I10", "I20", "I10", Direction.NORTH),
    "I01_to_I11": Segment("I01_to_I11", "I01", "I11", Direction.SOUTH),
    "I11_to_I01": Segment("I11_to_I01", "I11", "I01", Direction.NORTH),
    "I11_to_I21": Segment("I11_to_I21", "I11", "I21", Direction.SOUTH),
    "I21_to_I11": Segment("I21_to_I11", "I21", "I11", Direction.NORTH),
    "I02_to_I12": Segment("I02_to_I12", "I02", "I12", Direction.SOUTH),
    "I12_to_I02": Segment("I12_to_I02", "I12", "I02", Direction.NORTH),
    "I12_to_I22": Segment("I12_to_I22", "I12", "I22", Direction.SOUTH),
    "I22_to_I12": Segment("I22_to_I12", "I22", "I12", Direction.NORTH),
    "A_to_I00": Segment("A_to_I00", "A", "I00", Direction.EAST),
    "I00_to_A": Segment("I00_to_A", "I00", "A", Direction.WEST),
    "B_to_I02": Segment("B_to_I02", "B", "I02", Direction.SOUTH),
    "I02_to_B": Segment("I02_to_B", "I02", "B", Direction.NORTH),
    "C_to_I22": Segment("C_to_I22", "C", "I22", Direction.WEST),
    "I22_to_C": Segment("I22_to_C", "I22", "C", Direction.EAST),
    "D_to_I20": Segment("D_to_I20", "D", "I20", Direction.NORTH),
    "I20_to_D": Segment("I20_to_D", "I20", "D", Direction.SOUTH),
}


def build_nodes() -> Dict[str, Node]:
    return dict(NODES)


def build_segments() -> Dict[str, Segment]:
    return dict(SEGMENTS)


def is_valid_crossing_transition(
    segments: Dict[str, Segment],
    incoming_segment_id: str,
    outgoing_segment_id: str,
) -> bool:
    incoming = segments.get(incoming_segment_id)
    outgoing = segments.get(outgoing_segment_id)
    if incoming is None or outgoing is None:
        return False
    return incoming.to_node == outgoing.from_node


def is_u_turn_transition(
    segments: Dict[str, Segment],
    incoming_segment_id: str,
    outgoing_segment_id: str,
) -> bool:
    if not is_valid_crossing_transition(segments, incoming_segment_id, outgoing_segment_id):
        return False
    incoming = segments[incoming_segment_id]
    outgoing = segments[outgoing_segment_id]
    if not incoming.to_node.startswith("I"):
        return False
    return outgoing.to_node == incoming.from_node
```

### `i_group_phaseA.py`

_(see repository file)_

### `v_group_phaseA.py`

_(see repository file)_

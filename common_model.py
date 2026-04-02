from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Dict, Optional


class NodeType(Enum):
    """Classify each node as either an internal intersection or an external terminal."""
    INTERSECTION = "intersection"
    TERMINAL = "terminal"


class Direction(Enum):
    """Represent the travel direction assigned to each directed road segment."""
    NORTH = "north"
    SOUTH = "south"
    EAST = "east"
    WEST = "west"


@dataclass(frozen=True)
class Node:
    """Store the identifier and role of a node in the shared road graph."""
    node_id: str
    node_type: NodeType


@dataclass(frozen=True)
class Segment:
    """Describe a directed road segment between two nodes and its slot length."""
    segment_id: str
    from_node: str
    to_node: str
    direction: Direction
    length_slots: int = 30


@dataclass
class VehicleState:
    """Capture the vehicle position, visit progress, and pending crossing intent for one car."""
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
    """Record which direction currently has the green light at an intersection."""
    intersection_id: str
    green_direction: Optional[Direction] = None


@dataclass
class CrossingRequest:
    """Represent a vehicle request to move through an intersection into a specific outgoing segment."""
    car_id: str
    intersection_id: str
    incoming_segment: str
    outgoing_segment: str


@dataclass
class CrossingGrant:
    """Represent the infrastructure decision that approves or denies a vehicle crossing request."""
    intersection_id: str
    car_id: str
    granted: bool


@dataclass
class SimulationState:
    """Bundle the top-level simulation state shared across vehicles, lights, and time."""
    time_step: int
    vehicles: Dict[str, VehicleState]
    lights: Dict[str, IntersectionLightState]


# Keep the static map in one place so the i-group and v-group share the same node and segment IDs.
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
    """Return a shallow copy of the shared node map so callers can read it safely."""
    return dict(NODES)


def build_segments() -> Dict[str, Segment]:
    """Return a shallow copy of the shared segment map used by both simulation groups."""
    return dict(SEGMENTS)


def is_valid_crossing_transition(
    segments: Dict[str, Segment],
    incoming_segment_id: str,
    outgoing_segment_id: str,
) -> bool:
    """Treat a crossing as valid only when the incoming destination matches the outgoing origin."""
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
    """Treat a transition as a U-turn when it immediately returns to the node the vehicle came from."""
    if not is_valid_crossing_transition(segments, incoming_segment_id, outgoing_segment_id):
        return False

    incoming = segments[incoming_segment_id]
    outgoing = segments[outgoing_segment_id]
    if not incoming.to_node.startswith("I"):
        return False
    return outgoing.to_node == incoming.from_node


# Maps each direction to the direction that would be a right turn from it.
_RIGHT_TURN_OF: Dict[Direction, Direction] = {
    Direction.NORTH: Direction.EAST,
    Direction.EAST:  Direction.SOUTH,
    Direction.SOUTH: Direction.WEST,
    Direction.WEST:  Direction.NORTH,
}


def is_right_turn_transition(
    segments: Dict[str, Segment],
    incoming_segment_id: str,
    outgoing_segment_id: str,
) -> bool:
    """Return True when the outgoing direction is a right turn relative to the incoming direction."""
    if not is_valid_crossing_transition(segments, incoming_segment_id, outgoing_segment_id):
        return False
    incoming = segments[incoming_segment_id]
    outgoing = segments[outgoing_segment_id]
    return outgoing.direction == _RIGHT_TURN_OF[incoming.direction]

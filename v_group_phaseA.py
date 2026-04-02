from __future__ import annotations

from typing import Dict, List, Optional, Set

from common_model import (
    Segment,
    VehicleState,
    build_nodes,
    build_segments,
    is_right_turn_transition,
    is_u_turn_transition,
    is_valid_crossing_transition,
)


class RoutePlanner:
    """
    A simple Phase A route planner.
    It does not solve a full shortest path yet and instead picks the next segment from remaining targets and local congestion.
    """

    def __init__(self, segments: Dict[str, Segment]) -> None:
        self.segments = segments
        self.outgoing_by_node: Dict[str, List[str]] = {}
        for seg_id, seg in segments.items():
            self.outgoing_by_node.setdefault(seg.from_node, []).append(seg_id)

    def get_next_target(self, vehicle: VehicleState) -> str:
        remaining = []
        if not vehicle.visited_B:
            remaining.append("B")
        if not vehicle.visited_C:
            remaining.append("C")
        if not vehicle.visited_D:
            remaining.append("D")

        if remaining:
            return remaining[0]

        return "A"

    def choose_next_segment(
        self,
        current_segment: str,
        current_node: str,
        target_node: str,
        congestion_map: Dict[str, int],
    ) -> Optional[str]:
        """
        Choose the outgoing segment that appears to move the vehicle closer to its target.
        Phase A keeps this heuristic cheap by using node-ID distance and congestion only.
        """
        candidates = self.outgoing_by_node.get(current_node, [])
        if not candidates:
            return None

        best_seg = None
        best_score = float("inf")

        for seg_id in candidates:
            if not is_valid_crossing_transition(self.segments, current_segment, seg_id):
                continue
            if is_u_turn_transition(self.segments, current_segment, seg_id):
                continue
            if is_right_turn_transition(self.segments, current_segment, seg_id):
                continue

            seg = self.segments[seg_id]
            score = self.estimate_cost(seg.to_node, target_node, congestion_map)
            if score < best_score:
                best_score = score
                best_seg = seg_id

        return best_seg

    def estimate_cost(
        self,
        node: str,
        target: str,
        congestion_map: Dict[str, int],
    ) -> int:
        """
        A rough cost function used only in Phase A.
        It combines a grid-distance estimate with a congestion penalty instead of a full shortest-path search.
        """
        if node == target:
            return 0

        score = 10

        if node.startswith("I") and target.startswith("I"):
            score += abs(int(node[1]) - int(target[1])) * 2
            score += abs(int(node[2]) - int(target[2])) * 2
        elif node.startswith("I") and target in {"A", "B", "C", "D"}:
            terminal_anchor = {
                "A": "I00",
                "B": "I02",
                "C": "I22",
                "D": "I20",
            }[target]
            score += abs(int(node[1]) - int(terminal_anchor[1])) * 2
            score += abs(int(node[2]) - int(terminal_anchor[2])) * 2
        elif node in {"A", "B", "C", "D"}:
            score += 3

        score += congestion_map.get(node, 0)
        return score


class VehicleSimulator:
    def __init__(self) -> None:
        self.nodes = build_nodes()
        self.segments = build_segments()
        self.route_planner = RoutePlanner(self.segments)
        self.vehicles: Dict[str, VehicleState] = {}
        self.time_step = 0
        self.completed_tours = 0
        self.red_light_violations = 0
        self.collisions = 0
        self.illegal_direction_violations = 0
        self.u_turn_violations = 0
        self.right_turn_violations = 0

    def add_vehicle(self, car_id: str) -> None:
        self.vehicles[car_id] = VehicleState(
            car_id=car_id,
            current_segment="A_to_I00",
            current_slot=0,
            visited_B=False,
            visited_C=False,
            visited_D=False,
            current_target="B",
            stopped=False,
            request_crossing=False,
            desired_next_segment=None,
        )

    def mark_visit_if_needed(self, vehicle: VehicleState) -> None:
        seg = self.segments[vehicle.current_segment]
        node = seg.to_node if vehicle.current_slot == seg.length_slots - 1 else None

        if node == "B":
            vehicle.visited_B = True
        elif node == "C":
            vehicle.visited_C = True
        elif node == "D":
            vehicle.visited_D = True

        vehicle.current_target = self.route_planner.get_next_target(vehicle)

    # Spec: a car can see up to 0.5 mile = 15 slots ahead with no third car in between.
    VISIBILITY_SLOTS = 15

    def is_front_blocked(self, vehicle: VehicleState) -> bool:
        """
        Block only if the nearest car ahead in the same segment is within VISIBILITY_SLOTS slots.
        The spec limits visibility to 0.5 mile (15 slots) with no third car in between,
        so a car more than 15 slots away is invisible and does not block movement.
        """
        nearest_slot = None
        for other in self.vehicles.values():
            if other.car_id == vehicle.car_id:
                continue
            if other.current_segment == vehicle.current_segment and other.current_slot > vehicle.current_slot:
                if nearest_slot is None or other.current_slot < nearest_slot:
                    nearest_slot = other.current_slot

        if nearest_slot is None:
            return False
        return (nearest_slot - vehicle.current_slot) <= self.VISIBILITY_SLOTS

    def build_crossing_request(
        self,
        vehicle: VehicleState,
        congestion_map: Dict[str, int],
    ) -> None:
        seg = self.segments[vehicle.current_segment]
        current_node = seg.to_node
        target = vehicle.current_target

        next_seg = self.route_planner.choose_next_segment(
            current_segment=vehicle.current_segment,
            current_node=current_node,
            target_node=target,
            congestion_map=congestion_map,
        )

        vehicle.request_crossing = True
        vehicle.desired_next_segment = next_seg

    def advance_from_terminal(
        self,
        vehicle: VehicleState,
        congestion_map: Dict[str, int],
    ) -> None:
        seg = self.segments[vehicle.current_segment]
        next_seg = self.route_planner.choose_next_segment(
            current_segment=vehicle.current_segment,
            current_node=seg.to_node,
            target_node=vehicle.current_target,
            congestion_map=congestion_map,
        )

        if next_seg is None:
            self.stay(vehicle)
            vehicle.request_crossing = False
            vehicle.desired_next_segment = None
            return

        if not is_valid_crossing_transition(self.segments, vehicle.current_segment, next_seg):
            self.illegal_direction_violations += 1
            self.stay(vehicle)
            vehicle.request_crossing = False
            vehicle.desired_next_segment = None
            return

        vehicle.current_segment = next_seg
        vehicle.current_slot = 0
        vehicle.stopped = False
        vehicle.request_crossing = False
        vehicle.desired_next_segment = None

    def move_inside_segment(self, vehicle: VehicleState) -> None:
        if vehicle.current_slot < self.segments[vehicle.current_segment].length_slots - 1:
            vehicle.current_slot += 1
            vehicle.stopped = False
            vehicle.request_crossing = False
            vehicle.desired_next_segment = None

    def stay(self, vehicle: VehicleState) -> None:
        vehicle.stopped = True

    def apply_intersection_result(
        self,
        vehicle: VehicleState,
        lights: Dict[str, Optional[str]],
        crossing_grants: List[Dict[str, object]],
    ) -> None:
        seg = self.segments[vehicle.current_segment]
        intersection_id = seg.to_node

        granted = False
        for g in crossing_grants:
            if g["car_id"] == vehicle.car_id and g["intersection_id"] == intersection_id and g["granted"]:
                granted = True
                break

        green_dir = lights.get(intersection_id)

        if granted and vehicle.desired_next_segment is not None:
            if not is_valid_crossing_transition(
                self.segments,
                vehicle.current_segment,
                vehicle.desired_next_segment,
            ):
                self.illegal_direction_violations += 1
                vehicle.stopped = True
                return

            if is_u_turn_transition(
                self.segments,
                vehicle.current_segment,
                vehicle.desired_next_segment,
            ):
                self.u_turn_violations += 1
                vehicle.stopped = True
                return

            current_direction = seg.direction.value
            if green_dir is None or green_dir != current_direction:
                # Record the mismatch for validation, but still apply the received grant to the simulator state.
                self.red_light_violations += 1

            vehicle.current_segment = vehicle.desired_next_segment
            vehicle.current_slot = 0
            vehicle.stopped = False
            vehicle.request_crossing = False
            vehicle.desired_next_segment = None
            return

        vehicle.stopped = True

    def check_collision(self) -> None:
        occupied: Set[tuple[str, int]] = set()
        for vehicle in self.vehicles.values():
            loc = (vehicle.current_segment, vehicle.current_slot)
            if loc in occupied:
                self.collisions += 1
            occupied.add(loc)

    def build_vehicle_snapshot(self) -> Dict[str, Dict[str, object]]:
        return {
            car_id: {
                "current_segment": v.current_segment,
                "current_slot": v.current_slot,
                "visited_B": v.visited_B,
                "visited_C": v.visited_C,
                "visited_D": v.visited_D,
                "current_target": v.current_target,
                "stopped": v.stopped,
                "request_crossing": v.request_crossing,
                "desired_next_segment": v.desired_next_segment,
            }
            for car_id, v in self.vehicles.items()
        }

    def prepare_requests(
        self,
        congestion_map: Dict[str, int],
    ) -> Dict[str, Dict[str, object]]:
        """
        Build the snapshot that will be sent to the i-group.
        Segment-internal movement happens here, while actual intersection crossing is applied after the i-group response arrives.
        """
        for vehicle in self.vehicles.values():
            vehicle.current_target = self.route_planner.get_next_target(vehicle)

            if self.is_front_blocked(vehicle):
                self.stay(vehicle)
                vehicle.request_crossing = False
                vehicle.desired_next_segment = None
                continue

            seg = self.segments[vehicle.current_segment]

            if vehicle.current_slot < seg.length_slots - 1:
                self.move_inside_segment(vehicle)
            elif seg.to_node in {"A", "B", "C", "D"}:
                self.mark_visit_if_needed(vehicle)
                self.advance_from_terminal(vehicle, congestion_map)
            else:
                self.build_crossing_request(vehicle, congestion_map)

        self.check_collision()
        return self.build_vehicle_snapshot()

    def apply_i_group_output(
        self,
        lights: Dict[str, Optional[str]],
        crossing_grants: List[Dict[str, object]],
    ) -> None:
        for vehicle in self.vehicles.values():
            seg = self.segments[vehicle.current_segment]

            if vehicle.request_crossing and seg.to_node.startswith("I"):
                self.apply_intersection_result(vehicle, lights, crossing_grants)

            self.mark_visit_if_needed(vehicle)

            if vehicle.visited_B and vehicle.visited_C and vehicle.visited_D:
                end_seg = self.segments[vehicle.current_segment]
                if end_seg.to_node == "A" and vehicle.current_slot == end_seg.length_slots - 1:
                    self.completed_tours += 1
                    # Restart completed tours from the same initial state.
                    vehicle.current_segment = "A_to_I00"
                    vehicle.current_slot = 0
                    vehicle.visited_B = False
                    vehicle.visited_C = False
                    vehicle.visited_D = False
                    vehicle.current_target = "B"
                    vehicle.stopped = False
                    vehicle.request_crossing = False
                    vehicle.desired_next_segment = None

        self.check_collision()

    def step(
        self,
        i_group_output: Optional[Dict[str, object]] = None,
    ) -> Dict[str, object]:
        """
        Perform both request preparation and i-group result application for one step.
        The returned vehicles value is the actual vehicle state at the end of the step,
        while vehicles_for_i_group is the snapshot prepared for the i-group.
        """
        self.time_step += 1

        if i_group_output is None:
            i_group_output = {
                "lights": {},
                "crossing_grants": [],
                "congestion_map": {},
            }

        congestion_map = i_group_output.get("congestion_map", {})
        lights = i_group_output.get("lights", {})
        crossing_grants = i_group_output.get("crossing_grants", [])

        vehicles_for_i_group = self.prepare_requests(congestion_map)
        self.apply_i_group_output(lights, crossing_grants)
        current_vehicle_states = self.build_vehicle_snapshot()

        return {
            "time_step": self.time_step,
            "vehicles": current_vehicle_states,
            "vehicles_for_i_group": vehicles_for_i_group,
            "stats": {
                "completed_tours": self.completed_tours,
                "red_light_violations": self.red_light_violations,
                "collisions": self.collisions,
                "illegal_direction_violations": self.illegal_direction_violations,
                "u_turn_violations": self.u_turn_violations,
            },
        }


if __name__ == "__main__":
    v_sim = VehicleSimulator()
    v_sim.add_vehicle("car_1")
    v_sim.add_vehicle("car_2")

    # Assume the first step runs before any i-group response is available.
    out1 = v_sim.step()
    print("Step 1")
    print(out1)

    # Later steps can inject lights and grants computed by the i-group.
    i_group_output = {
        "lights": {
            "I00": "east",
            "I11": "north",
        },
        "crossing_grants": [
            {
                "intersection_id": "I00",
                "car_id": "car_1",
                "granted": True,
            }
        ],
        "congestion_map": {
            "I00": 1,
            "I11": 2,
        },
    }

    out2 = v_sim.step(i_group_output)
    print("Step 2")
    print(out2)

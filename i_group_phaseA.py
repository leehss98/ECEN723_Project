from __future__ import annotations

import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from common_model import (
    CrossingGrant,
    CrossingRequest,
    Direction,
    IntersectionLightState,
    NodeType,
    VehicleState,
    build_nodes,
    build_segments,
    is_right_turn_transition,
    is_u_turn_transition,
    is_valid_crossing_transition,
)


@dataclass
class SafetyReport:
    collisions: int = 0
    red_light_violations: int = 0
    simultaneous_green_violations: int = 0
    invalid_grant_violations: int = 0
    wrong_direction_violations: int = 0
    u_turn_violations: int = 0
    right_turn_violations: int = 0


@dataclass
class IntersectionController:
    intersection_id: str
    light_state: IntersectionLightState
    starvation_counter: Dict[Direction, int] = field(default_factory=dict)

    def __post_init__(self) -> None:
        for d in Direction:
            self.starvation_counter.setdefault(d, 0)

    def select_green_direction(
        self,
        waiting_by_direction: Dict[Direction, int],
    ) -> Optional[Direction]:
        """
        Score each direction with both queue length and a starvation counter.
        This gives long-waiting directions a simple bias so they do not get ignored indefinitely.
        """
        best_dir = None
        best_score = -1

        for d in Direction:
            score = waiting_by_direction.get(d, 0) + self.starvation_counter.get(d, 0)
            if score > best_score and waiting_by_direction.get(d, 0) > 0:
                best_score = score
                best_dir = d

        return best_dir

    def update_light(
        self,
        waiting_by_direction: Dict[Direction, int],
    ) -> None:
        chosen = self.select_green_direction(waiting_by_direction)
        self.light_state.green_direction = chosen

        for d in Direction:
            if d == chosen:
                self.starvation_counter[d] = 0
            else:
                if waiting_by_direction.get(d, 0) > 0:
                    self.starvation_counter[d] += 1
                else:
                    self.starvation_counter[d] = 0


class InfrastructureSimulator:
    def __init__(self) -> None:
        self.nodes = build_nodes()
        self.segments = build_segments()

        self.intersection_ids = [
            node_id
            for node_id, node in self.nodes.items()
            if node.node_type == NodeType.INTERSECTION
        ]

        self.controllers: Dict[str, IntersectionController] = {
            iid: IntersectionController(
                intersection_id=iid,
                light_state=IntersectionLightState(intersection_id=iid, green_direction=None),
            )
            for iid in self.intersection_ids
        }

        self.time_step = 0
        self.safety_report = SafetyReport()

    def get_incoming_waiting_vehicles(
        self,
        vehicles: Dict[str, VehicleState],
    ) -> Dict[str, List[VehicleState]]:
        """
        A vehicle joins an intersection queue only when three conditions hold:
        it is at the last slot, it has requested crossing, and its current segment ends at an intersection.
        """
        waiting: Dict[str, List[VehicleState]] = {iid: [] for iid in self.intersection_ids}

        for vehicle in vehicles.values():
            if vehicle.current_segment not in self.segments:
                continue

            seg = self.segments[vehicle.current_segment]
            if seg.to_node in self.intersection_ids:
                if vehicle.current_slot == seg.length_slots - 1 and vehicle.request_crossing:
                    waiting[seg.to_node].append(vehicle)

        return waiting

    def count_waiting_by_direction(
        self,
        intersection_id: str,
        waiting_vehicles: List[VehicleState],
    ) -> Dict[Direction, int]:
        counts = {d: 0 for d in Direction}
        for vehicle in waiting_vehicles:
            seg = self.segments[vehicle.current_segment]
            counts[seg.direction] += 1
        return counts

    def validate_request(
        self,
        vehicle: VehicleState,
    ) -> Tuple[bool, bool]:
        # Reject any request whose position or segment transition violates the shared road model.
        if vehicle.current_segment not in self.segments:
            self.safety_report.wrong_direction_violations += 1
            return False, False
        if vehicle.desired_next_segment is None or vehicle.desired_next_segment not in self.segments:
            self.safety_report.wrong_direction_violations += 1
            return False, False

        incoming_seg = self.segments[vehicle.current_segment]
        if vehicle.current_slot != incoming_seg.length_slots - 1:
            self.safety_report.wrong_direction_violations += 1
            return False, False

        if incoming_seg.to_node not in self.intersection_ids:
            self.safety_report.wrong_direction_violations += 1
            return False, False

        if not is_valid_crossing_transition(
            self.segments,
            vehicle.current_segment,
            vehicle.desired_next_segment,
        ):
            self.safety_report.wrong_direction_violations += 1
            return False, False

        if is_u_turn_transition(
            self.segments,
            vehicle.current_segment,
            vehicle.desired_next_segment,
        ):
            self.safety_report.u_turn_violations += 1
            return False, True

        if is_right_turn_transition(
            self.segments,
            vehicle.current_segment,
            vehicle.desired_next_segment,
        ):
            self.safety_report.right_turn_violations += 1
            return False, False

        return True, False

    def build_crossing_requests(
        self,
        vehicles: Dict[str, VehicleState],
    ) -> Dict[str, List[CrossingRequest]]:
        requests: Dict[str, List[CrossingRequest]] = {iid: [] for iid in self.intersection_ids}

        for vehicle in vehicles.values():
            if not vehicle.request_crossing:
                continue
            is_valid, _ = self.validate_request(vehicle)
            if not is_valid:
                continue

            incoming_seg = self.segments[vehicle.current_segment]
            intersection_id = incoming_seg.to_node

            if intersection_id not in self.intersection_ids:
                continue

            requests[intersection_id].append(
                CrossingRequest(
                    car_id=vehicle.car_id,
                    intersection_id=intersection_id,
                    incoming_segment=vehicle.current_segment,
                    outgoing_segment=vehicle.desired_next_segment,
                )
            )

        return requests

    def select_one_grant(
        self,
        intersection_id: str,
        requests: List[CrossingRequest],
        vehicles: Dict[str, VehicleState],
    ) -> Optional[CrossingGrant]:
        """
        Allow at most one crossing per intersection in each time step.
        Only requests arriving from the current green direction are eligible.
        """
        green_dir = self.controllers[intersection_id].light_state.green_direction
        if green_dir is None:
            return None

        eligible: List[CrossingRequest] = []
        for req in requests:
            incoming_seg = self.segments[req.incoming_segment]
            if incoming_seg.direction == green_dir:
                eligible.append(req)

        if not eligible:
            return None

        # Break ties in a stable way so repeated runs produce the same grant.
        eligible.sort(key=lambda r: r.car_id)
        chosen = eligible[0]

        return CrossingGrant(
            intersection_id=intersection_id,
            car_id=chosen.car_id,
            granted=True,
        )

    def compute_congestion_map(
        self,
        vehicles: Dict[str, VehicleState],
    ) -> Dict[str, int]:
        """
        Approximate congestion by counting stopped vehicles on segments that feed each intersection.
        Phase A uses stopped-car counts instead of a more detailed queue model.
        """
        congestion = {iid: 0 for iid in self.intersection_ids}

        for vehicle in vehicles.values():
            if vehicle.current_segment not in self.segments:
                continue
            seg = self.segments[vehicle.current_segment]
            if seg.to_node in self.intersection_ids and vehicle.stopped:
                congestion[seg.to_node] += 1

        return congestion

    def detect_collisions(
        self,
        vehicles: Dict[str, VehicleState],
    ) -> None:
        occupied: Dict[Tuple[str, int], str] = {}
        for vehicle in vehicles.values():
            if vehicle.current_segment not in self.segments:
                continue

            loc = (vehicle.current_segment, vehicle.current_slot)
            if loc in occupied:
                self.safety_report.collisions += 1
            else:
                occupied[loc] = vehicle.car_id

    def check_safety(
        self,
        vehicles: Dict[str, VehicleState],
        grants: List[CrossingGrant],
        requests_by_intersection: Dict[str, List[CrossingRequest]],
    ) -> None:
        """
        Verify that each granted vehicle came from the green direction and requested a legal transition.
        Collision detection is handled separately by scanning occupied vehicle positions afterward.
        """
        for iid, controller in self.controllers.items():
            green = controller.light_state.green_direction
            green_count = 1 if green is not None else 0
            if green_count > 1:
                self.safety_report.simultaneous_green_violations += 1

        grant_map: Dict[Tuple[str, str], CrossingGrant] = {
            (g.intersection_id, g.car_id): g for g in grants if g.granted
        }

        for iid, reqs in requests_by_intersection.items():
            green_dir = self.controllers[iid].light_state.green_direction
            for req in reqs:
                key = (iid, req.car_id)
                if key in grant_map:
                    incoming_seg = self.segments[req.incoming_segment]
                    if green_dir is None or incoming_seg.direction != green_dir:
                        self.safety_report.invalid_grant_violations += 1
                        self.safety_report.red_light_violations += 1

                    if not is_valid_crossing_transition(
                        self.segments,
                        req.incoming_segment,
                        req.outgoing_segment,
                    ):
                        self.safety_report.invalid_grant_violations += 1
                        self.safety_report.wrong_direction_violations += 1

                    if is_u_turn_transition(
                        self.segments,
                        req.incoming_segment,
                        req.outgoing_segment,
                    ):
                        self.safety_report.invalid_grant_violations += 1
                        self.safety_report.u_turn_violations += 1

                    if is_right_turn_transition(
                        self.segments,
                        req.incoming_segment,
                        req.outgoing_segment,
                    ):
                        self.safety_report.invalid_grant_violations += 1
                        self.safety_report.right_turn_violations += 1

        self.detect_collisions(vehicles)

    def step(
        self,
        vehicles: Dict[str, VehicleState],
    ) -> Dict[str, object]:
        """
        Consume the vehicle state from the v-group and compute lights, grants, congestion, and safety data.
        """
        self.time_step += 1

        waiting = self.get_incoming_waiting_vehicles(vehicles)

        for iid in self.intersection_ids:
            waiting_by_dir = self.count_waiting_by_direction(iid, waiting[iid])
            self.controllers[iid].update_light(waiting_by_dir)

        requests_by_intersection = self.build_crossing_requests(vehicles)

        grants: List[CrossingGrant] = []
        for iid in self.intersection_ids:
            grant = self.select_one_grant(iid, requests_by_intersection[iid], vehicles)
            if grant is not None:
                grants.append(grant)

        congestion = self.compute_congestion_map(vehicles)

        self.check_safety(vehicles, grants, requests_by_intersection)

        lights_out = {
            iid: controller.light_state.green_direction.value
            if controller.light_state.green_direction is not None else None
            for iid, controller in self.controllers.items()
        }

        grants_out = [
            {
                "intersection_id": g.intersection_id,
                "car_id": g.car_id,
                "granted": g.granted,
            }
            for g in grants
        ]

        return {
            "time_step": self.time_step,
            "lights": lights_out,
            "crossing_grants": grants_out,
            "congestion_map": congestion,
            "safety_report": {
                "collisions": self.safety_report.collisions,
                "red_light_violations": self.safety_report.red_light_violations,
                "simultaneous_green_violations": self.safety_report.simultaneous_green_violations,
                "invalid_grant_violations": self.safety_report.invalid_grant_violations,
                "wrong_direction_violations": self.safety_report.wrong_direction_violations,
                "u_turn_violations": self.safety_report.u_turn_violations,
                "right_turn_violations": self.safety_report.right_turn_violations,
            },
        }


if __name__ == "__main__":
    sim = InfrastructureSimulator()

    vehicles = {
        "car_1": VehicleState(
            car_id="car_1",
            current_segment="A_to_I00",
            current_slot=29,
            visited_B=False,
            visited_C=False,
            visited_D=False,
            current_target="B",
            stopped=True,
            request_crossing=True,
            desired_next_segment="I00_to_I01",
        ),
        "car_2": VehicleState(
            car_id="car_2",
            current_segment="I10_to_I11",
            current_slot=29,
            visited_B=False,
            visited_C=False,
            visited_D=False,
            current_target="C",
            stopped=True,
            request_crossing=True,
            desired_next_segment="I11_to_I12",
        ),
        "car_3": VehicleState(
            car_id="car_3",
            current_segment="I01_to_I11",
            current_slot=12,
            visited_B=True,
            visited_C=False,
            visited_D=False,
            current_target="D",
            stopped=False,
            request_crossing=False,
            desired_next_segment=None,
        ),
    }

    result = sim.step(vehicles)
    print(result)

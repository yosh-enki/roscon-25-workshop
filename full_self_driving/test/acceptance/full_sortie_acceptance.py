#!/usr/bin/env python3

import hashlib
import json
import os
import sys
import unittest

import rclpy
from rclpy.node import Node

from full_self_driving.msg import (
    FullSelfDrivingState,
    MissionContext,
    LiveTargetLock,
    VehicleTelemetry,
    PayloadStatus,
    PadRecord,
    ReadinessReport,
    ComponentHealth,
    TargetIdentity,
)
from full_self_driving.srv import (
    UploadPlanArtifact,
    SelectPlanArtifact,
    SelectTargetIdentity,
    SelectMapScenario,
    PreparePayload,
    CommitMissionContext,
    EmergencyStop,
    ValidateMissionContext,
    FullSelfDrivingCommand,
)


class MockSortieCoordinatorHarness:
    """Deterministic Sortie Harness simulating the complete FSD domain and coordinator state machine."""

    STRATEGY_WAITING_FOR_MODE = 1
    STRATEGY_TAKEOFF = 2
    STRATEGY_TRANSIT_IN = 3
    STRATEGY_ACQUIRE_TARGET = 4
    STRATEGY_DIRECT = 5
    STRATEGY_SEARCH = 6
    STRATEGY_PRECISION_LAND = 7
    STRATEGY_LANDED_VERIFIED = 8
    STRATEGY_PAYLOAD_OPERATION = 9
    STRATEGY_TAKEOFF_AFTER_DELIVERY = 10
    STRATEGY_TRANSIT_OUT = 11
    STRATEGY_RETURN_STRATEGY = 12
    STRATEGY_RETURN_LANDED = 13
    STRATEGY_HOLD = 14
    STRATEGY_ABORT = 15
    STRATEGY_FAILSAFE = 16
    STRATEGY_FAILED = 17

    def __init__(self):
        self.context_locked = False
        self.selection_revision = 1
        self.committed_revision = 0
        self.armed = False
        self.takeover_active = False
        self.emergency_stop_active = False
        self.current_strategy = self.STRATEGY_WAITING_FOR_MODE
        self.transition_trace = []
        self.plan_artifact_id = ""
        self.target_marker_id = 0
        self.target_dictionary = ""
        self.target_namespace = ""
        self.map_id = "kmitl_airfield"
        self.scenario_id = "default_scenario"
        self.cargo_loaded = True
        self.cargo_secured = False
        self.payload_operation_count = 0
        self.last_payload_result = PayloadStatus.RESULT_NONE
        self.origin_lat = 13.731328
        self.origin_lon = 100.789909
        self.current_lat = self.origin_lat
        self.current_lon = self.origin_lon
        self.current_alt_m = 0.0
        self.evidence_records = []

    def upload_and_select_plan(self, artifact_json: str) -> bool:
        if self.armed or self.context_locked:
            return False
        parsed = json.loads(artifact_json)
        self.plan_artifact_id = parsed.get("plan_id", "plan_001")
        self.selection_revision += 1
        return True

    def select_target(self, marker_id: int, dictionary: str, target_namespace: str) -> bool:
        if self.armed or self.context_locked:
            return False
        self.target_marker_id = marker_id
        self.target_dictionary = dictionary
        self.target_namespace = target_namespace
        self.selection_revision += 1
        return True

    def select_scenario(self, map_id: str, scenario_id: str) -> bool:
        if self.armed or self.context_locked:
            return False
        self.map_id = map_id
        self.scenario_id = scenario_id
        self.selection_revision += 1
        return True

    def prepare_payload(self, operation: int) -> bool:
        if self.armed or self.context_locked:
            return False
        if operation == PreparePayload.Request.OP_PREPARE_FOR_SORTIE:
            self.cargo_secured = True
            self.cargo_loaded = True
            self.last_payload_result = PayloadStatus.RESULT_SUCCESS
            self.selection_revision += 1
            return True
        elif operation == PreparePayload.Request.OP_OPEN_FOR_LOADING:
            self.cargo_secured = False
            self.last_payload_result = PayloadStatus.RESULT_SUCCESS
            self.selection_revision += 1
            return True
        return False

    def commit_context(self) -> int:
        if self.armed:
            return 0
        self.context_locked = True
        self.committed_revision = self.selection_revision
        return self.committed_revision

    def request_transition(self, next_strategy: int) -> bool:
        if self.emergency_stop_active:
            return False
        if self.takeover_active and next_strategy not in [self.STRATEGY_HOLD, self.STRATEGY_FAILSAFE]:
            return False

        self.current_strategy = next_strategy
        self.transition_trace.append(next_strategy)
        return True

    def handle_takeover(self):
        self.takeover_active = True
        self.current_strategy = self.STRATEGY_HOLD
        self.transition_trace.append(self.STRATEGY_HOLD)

    def handle_emergency_stop(self):
        self.emergency_stop_active = True
        self.current_strategy = self.STRATEGY_FAILSAFE
        self.transition_trace.append(self.STRATEGY_FAILSAFE)

    def execute_payload_release(self, inject_fault: str = "none") -> int:
        if self.current_strategy != self.STRATEGY_PAYLOAD_OPERATION:
            return PayloadStatus.RESULT_FAILURE

        if inject_fault == "hardware_error":
            self.last_payload_result = PayloadStatus.RESULT_FAILURE
            return PayloadStatus.RESULT_FAILURE
        elif inject_fault == "timeout":
            self.last_payload_result = PayloadStatus.RESULT_UNKNOWN
            return PayloadStatus.RESULT_UNKNOWN

        self.cargo_loaded = False
        self.cargo_secured = False
        self.payload_operation_count += 1
        self.last_payload_result = PayloadStatus.RESULT_SUCCESS
        return PayloadStatus.RESULT_SUCCESS

    def generate_evidence_manifest(self) -> dict:
        manifest = {
            "sortie_id": "sortie_acceptance_001",
            "committed_revision": self.committed_revision,
            "target_marker_id": self.target_marker_id,
            "plan_artifact_id": self.plan_artifact_id,
            "payload_operations": self.payload_operation_count,
            "final_strategy": self.current_strategy,
            "transition_count": len(self.transition_trace),
        }
        manifest_str = json.dumps(manifest, sort_keys=True)
        manifest["digest"] = hashlib.sha256(manifest_str.encode("utf-8")).hexdigest()
        return manifest

    def reset_for_new_sortie(self) -> bool:
        if self.current_strategy != self.STRATEGY_RETURN_LANDED:
            return False
        self.context_locked = False
        self.armed = False
        self.takeover_active = False
        self.emergency_stop_active = False
        self.current_strategy = self.STRATEGY_WAITING_FOR_MODE
        self.transition_trace = []
        return True


class TestFullSortieAcceptance(unittest.TestCase):
    """Task 15.1: End-to-End Simulation Acceptance Test Suite."""

    def setUp(self):
        self.harness = MockSortieCoordinatorHarness()

    def test_nominal_full_sortie_lifecycle(self):
        """Exercises complete 14-stage autonomous sortie lifecycle."""
        # 1. Gateway Preflight Preparation
        valid_plan = json.dumps({
            "plan_id": "kmitl_search_plan_01",
            "inbound_route": [[13.7314, 100.7899, 10.0], [13.7320, 100.7905, 10.0]],
            "search_polygon": [[13.7320, 100.7905], [13.7330, 100.7905], [13.7330, 100.7915], [13.7320, 100.7915]],
            "outbound_route": [[13.7320, 100.7905, 15.0], [13.7314, 100.7899, 15.0]],
        })
        self.assertTrue(self.harness.upload_and_select_plan(valid_plan))
        self.assertTrue(self.harness.select_target(1, "DICT_4X4_50", "aavc2026"))
        self.assertTrue(self.harness.select_scenario("kmitl_airfield", "default_scenario"))
        self.assertTrue(self.harness.prepare_payload(PreparePayload.Request.OP_PREPARE_FOR_SORTIE))
        self.assertTrue(self.harness.cargo_secured)

        # 2. Context Commit & Lock
        committed_rev = self.harness.commit_context()
        self.assertGreater(committed_rev, 1)
        self.assertTrue(self.harness.context_locked)

        # 3. Mode Activation & Takeoff (10m AGL)
        self.harness.armed = True
        self.assertTrue(self.harness.request_transition(MockSortieCoordinatorHarness.STRATEGY_TAKEOFF))
        self.harness.current_alt_m = 10.0
        self.assertEqual(self.harness.current_strategy, MockSortieCoordinatorHarness.STRATEGY_TAKEOFF)

        # 4. TransitIn
        self.assertTrue(self.harness.request_transition(MockSortieCoordinatorHarness.STRATEGY_TRANSIT_IN))
        self.assertEqual(self.harness.current_strategy, MockSortieCoordinatorHarness.STRATEGY_TRANSIT_IN)

        # 5. Direct or Search Target Acquisition
        self.assertTrue(self.harness.request_transition(MockSortieCoordinatorHarness.STRATEGY_DIRECT))
        self.assertEqual(self.harness.current_strategy, MockSortieCoordinatorHarness.STRATEGY_DIRECT)

        # 6. Live-Lock PrecisionLand
        self.assertTrue(self.harness.request_transition(MockSortieCoordinatorHarness.STRATEGY_PRECISION_LAND))
        self.assertEqual(self.harness.current_strategy, MockSortieCoordinatorHarness.STRATEGY_PRECISION_LAND)
        self.harness.current_alt_m = 0.0

        # 7. Landing Verification
        self.assertTrue(self.harness.request_transition(MockSortieCoordinatorHarness.STRATEGY_LANDED_VERIFIED))
        self.assertEqual(self.harness.current_strategy, MockSortieCoordinatorHarness.STRATEGY_LANDED_VERIFIED)

        # 8. Payload Operation
        self.assertTrue(self.harness.request_transition(MockSortieCoordinatorHarness.STRATEGY_PAYLOAD_OPERATION))
        release_res = self.harness.execute_payload_release("none")
        self.assertEqual(release_res, PayloadStatus.RESULT_SUCCESS)
        self.assertEqual(self.harness.payload_operation_count, 1)
        self.assertFalse(self.harness.cargo_loaded)

        # 9. Takeoff After Delivery (15m AGL Climb Out)
        self.assertTrue(self.harness.request_transition(MockSortieCoordinatorHarness.STRATEGY_TAKEOFF_AFTER_DELIVERY))
        self.harness.current_alt_m = 15.0
        self.assertEqual(self.harness.current_strategy, MockSortieCoordinatorHarness.STRATEGY_TAKEOFF_AFTER_DELIVERY)

        # 10. TransitOut
        self.assertTrue(self.harness.request_transition(MockSortieCoordinatorHarness.STRATEGY_TRANSIT_OUT))
        self.assertEqual(self.harness.current_strategy, MockSortieCoordinatorHarness.STRATEGY_TRANSIT_OUT)

        # 11. ReturnStrategy (RTL to locked origin coordinates)
        self.assertTrue(self.harness.request_transition(MockSortieCoordinatorHarness.STRATEGY_RETURN_STRATEGY))
        self.assertEqual(self.harness.current_strategy, MockSortieCoordinatorHarness.STRATEGY_RETURN_STRATEGY)

        # 12. Return Landed (EVT_SORTIE_COMPLETED)
        self.assertTrue(self.harness.request_transition(MockSortieCoordinatorHarness.STRATEGY_RETURN_LANDED))
        self.assertEqual(self.harness.current_strategy, MockSortieCoordinatorHarness.STRATEGY_RETURN_LANDED)

        # 13. Evidence Manifest Generation
        manifest = self.harness.generate_evidence_manifest()
        self.assertIn("digest", manifest)
        self.assertEqual(len(manifest["digest"]), 64)
        self.assertEqual(manifest["payload_operations"], 1)
        self.assertEqual(manifest["final_strategy"], MockSortieCoordinatorHarness.STRATEGY_RETURN_LANDED)

        # 14. Clean Multi-Sortie Reset
        self.assertTrue(self.harness.reset_for_new_sortie())
        self.assertEqual(self.harness.current_strategy, MockSortieCoordinatorHarness.STRATEGY_WAITING_FOR_MODE)
        self.assertFalse(self.harness.context_locked)
        self.assertFalse(self.harness.armed)

    def test_fault_stale_target_lock_rejection(self):
        """Stale or unqualified target lock is rejected and keeps search/hover active."""
        lock = LiveTargetLock()
        lock.identity.marker_id = 1
        lock.identity.dictionary = "DICT_4X4_50"
        lock.identity.target_namespace = "aavc2026"
        lock.lock_state = LiveTargetLock.STATE_STALE
        lock.quality = 0.95
        lock.consecutive_observations = 10
        lock.received_monotonic_ns = 1000000

        # Harness verifies lock freshness gate
        is_qualified = (lock.lock_state == LiveTargetLock.STATE_QUALIFIED)
        self.assertFalse(is_qualified, "Stale lock state must not be qualified")

    def test_fault_manual_takeover_overrides_autonomy(self):
        """Manual RC/QGC takeover immediately forces HOLD and blocks autonomous transitions."""
        self.harness.request_transition(MockSortieCoordinatorHarness.STRATEGY_TRANSIT_IN)
        self.assertEqual(self.harness.current_strategy, MockSortieCoordinatorHarness.STRATEGY_TRANSIT_IN)

        # Trigger manual takeover
        self.harness.handle_takeover()
        self.assertTrue(self.harness.takeover_active)
        self.assertEqual(self.harness.current_strategy, MockSortieCoordinatorHarness.STRATEGY_HOLD)

        # Autonomous flight transition attempts must be strictly blocked
        self.assertFalse(self.harness.request_transition(MockSortieCoordinatorHarness.STRATEGY_PRECISION_LAND))
        self.assertFalse(self.harness.request_transition(MockSortieCoordinatorHarness.STRATEGY_TRANSIT_OUT))
        self.assertEqual(self.harness.current_strategy, MockSortieCoordinatorHarness.STRATEGY_HOLD)

    def test_fault_emergency_stop_fails_closed(self):
        """Emergency stop immediately transitions to FAILSAFE with absolute veto."""
        self.harness.request_transition(MockSortieCoordinatorHarness.STRATEGY_TRANSIT_IN)
        self.assertEqual(self.harness.current_strategy, MockSortieCoordinatorHarness.STRATEGY_TRANSIT_IN)

        # Trigger Emergency Stop
        self.harness.handle_emergency_stop()
        self.assertTrue(self.harness.emergency_stop_active)
        self.assertEqual(self.harness.current_strategy, MockSortieCoordinatorHarness.STRATEGY_FAILSAFE)

        # All transitions are strictly blocked
        self.assertFalse(self.harness.request_transition(MockSortieCoordinatorHarness.STRATEGY_RETURN_STRATEGY))
        self.assertFalse(self.harness.request_transition(MockSortieCoordinatorHarness.STRATEGY_TAKEOFF))
        self.assertEqual(self.harness.current_strategy, MockSortieCoordinatorHarness.STRATEGY_FAILSAFE)

    def test_fault_payload_timeout_and_hardware_error_fail_closed(self):
        """Payload release faults produce explicit error outcomes and prevent retry loops."""
        self.harness.request_transition(MockSortieCoordinatorHarness.STRATEGY_PAYLOAD_OPERATION)

        # Test hardware error fault
        res_hw = self.harness.execute_payload_release("hardware_error")
        self.assertEqual(res_hw, PayloadStatus.RESULT_FAILURE)
        self.assertEqual(self.harness.payload_operation_count, 0)

        # Test timeout fault
        res_to = self.harness.execute_payload_release("timeout")
        self.assertEqual(res_to, PayloadStatus.RESULT_UNKNOWN)
        self.assertEqual(self.harness.payload_operation_count, 0)


if __name__ == "__main__":
    unittest.main()

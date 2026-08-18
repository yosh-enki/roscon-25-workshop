#!/usr/bin/env python3
"""
Security Policy Enforcement & SROS2 Enclave Verification Test Suite.
Validates cryptographic certificate chains, least-privilege topic allowlists,
OMG DDS Security governance encryption rules, and rogue enclave rejection.
"""

import datetime
import os
import subprocess
import xml.etree.ElementTree as ET
import pytest

PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
KEYSTORE_DIR = os.path.join(PACKAGE_ROOT, "config", "security", "sample_keystore")

AUTONOMY_ENCLAVES = [
    "flight_runtime",
    "perception",
    "pad_registry",
    "gateway",
    "evidence",
]


@pytest.fixture(scope="module")
def keystore_path():
    assert os.path.isdir(KEYSTORE_DIR), f"Keystore directory does not exist: {KEYSTORE_DIR}"
    return KEYSTORE_DIR


class TestSecurityPolicyEnforcement:
    """Automated verification of SROS2 PKI, Access Controls, and DDS Security profiles."""

    def test_root_ca_exists_and_valid(self, keystore_path):
        ca_cert = os.path.join(keystore_path, "public", "ca.cert.pem")
        ca_key = os.path.join(keystore_path, "private", "ca.key.pem")

        assert os.path.isfile(ca_cert), f"Missing Root CA cert: {ca_cert}"
        assert os.path.isfile(ca_key), f"Missing Root CA key: {ca_key}"

        # Verify CA certificate properties using openssl
        res = subprocess.run(
            ["openssl", "x509", "-in", ca_cert, "-noout", "-subject", "-issuer"],
            capture_output=True,
            text=True,
            check=True,
        )
        assert "CN = FullSelfDriving_Root_CA" in res.stdout
        assert "O = FullSelfDriving" in res.stdout

    @pytest.mark.parametrize("enclave", AUTONOMY_ENCLAVES)
    def test_enclave_artifacts_exist(self, keystore_path, enclave):
        enc_dir = os.path.join(keystore_path, "enclaves", "full_self_driving", enclave)
        assert os.path.isdir(enc_dir), f"Missing enclave directory: {enc_dir}"

        required_files = [
            "cert.pem",
            "key.pem",
            "identity_ca.cert.pem",
            "permissions_ca.cert.pem",
            "governance.xml",
            "governance.p7s",
            "permissions.xml",
            "permissions.p7s",
        ]
        for req_file in required_files:
            file_path = os.path.join(enc_dir, req_file)
            assert os.path.isfile(file_path), f"Enclave {enclave} missing required file: {req_file}"

    @pytest.mark.parametrize("enclave", AUTONOMY_ENCLAVES)
    def test_certificate_chain_and_subject(self, keystore_path, enclave):
        ca_cert = os.path.join(keystore_path, "public", "ca.cert.pem")
        enc_dir = os.path.join(keystore_path, "enclaves", "full_self_driving", enclave)
        node_cert = os.path.join(enc_dir, "cert.pem")

        # 1. Verify certificate chain against Root CA
        verify_res = subprocess.run(
            ["openssl", "verify", "-CAfile", ca_cert, node_cert],
            capture_output=True,
            text=True,
        )
        assert verify_res.returncode == 0, f"Certificate verification failed for {enclave}: {verify_res.stderr}"
        assert "OK" in verify_res.stdout

        # 2. Verify Subject Common Name
        subj_res = subprocess.run(
            ["openssl", "x509", "-in", node_cert, "-noout", "-subject"],
            capture_output=True,
            text=True,
            check=True,
        )
        expected_cn = f"CN = /full_self_driving/{enclave}"
        assert expected_cn in subj_res.stdout, f"Expected {expected_cn} in subject, got: {subj_res.stdout}"

    @pytest.mark.parametrize("enclave", AUTONOMY_ENCLAVES)
    def test_pkcs7_policy_signatures(self, keystore_path, enclave):
        ca_cert = os.path.join(keystore_path, "public", "ca.cert.pem")
        enc_dir = os.path.join(keystore_path, "enclaves", "full_self_driving", enclave)
        gov_p7s = os.path.join(enc_dir, "governance.p7s")
        perm_p7s = os.path.join(enc_dir, "permissions.p7s")

        # Verify governance.p7s signature
        gov_res = subprocess.run(
            ["openssl", "smime", "-verify", "-inform", "PEM", "-in", gov_p7s, "-CAfile", ca_cert],
            capture_output=True,
            text=True,
        )
        assert gov_res.returncode == 0, f"governance.p7s signature invalid for {enclave}: {gov_res.stderr}"

        # Verify permissions.p7s signature
        perm_res = subprocess.run(
            ["openssl", "smime", "-verify", "-inform", "PEM", "-in", perm_p7s, "-CAfile", ca_cert],
            capture_output=True,
            text=True,
        )
        assert perm_res.returncode == 0, f"permissions.p7s signature invalid for {enclave}: {perm_res.stderr}"

    @pytest.mark.parametrize("enclave", AUTONOMY_ENCLAVES)
    def test_governance_xml_security_enforcement(self, keystore_path, enclave):
        enc_dir = os.path.join(keystore_path, "enclaves", "full_self_driving", enclave)
        gov_xml = os.path.join(enc_dir, "governance.xml")

        tree = ET.parse(gov_xml)
        root = tree.getroot()

        rule = root.find(".//domain_rule")
        assert rule is not None, f"domain_rule missing in {gov_xml}"

        unauth = rule.find("allow_unauthenticated_participants")
        assert unauth is not None and unauth.text.strip().lower() == "false"

        join_ctrl = rule.find("enable_join_access_control")
        assert join_ctrl is not None and join_ctrl.text.strip().lower() == "true"

        disc_prot = rule.find("discovery_protection_kind")
        assert disc_prot is not None and disc_prot.text.strip().upper() == "ENCRYPT"

        data_prot = rule.find(".//data_protection_kind")
        assert data_prot is not None and data_prot.text.strip().upper() == "ENCRYPT"

    @pytest.mark.parametrize("enclave", AUTONOMY_ENCLAVES)
    def test_permissions_xml_least_privilege(self, keystore_path, enclave):
        enc_dir = os.path.join(keystore_path, "enclaves", "full_self_driving", enclave)
        perm_xml = os.path.join(enc_dir, "permissions.xml")

        tree = ET.parse(perm_xml)
        root = tree.getroot()

        grant = root.find(".//grant")
        assert grant is not None, f"Grant missing in {perm_xml}"
        assert grant.get("name") == f"/full_self_driving/{enclave}"

        default_tag = grant.find("default")
        assert default_tag is not None and default_tag.text.strip().upper() == "DENY"

        pub_topics = [t.text.strip() for t in grant.findall(".//publish/topics/topic")]
        sub_topics = [t.text.strip() for t in grant.findall(".//subscribe/topics/topic")]

        # Specific isolation assertions:
        if enclave == "perception":
            # Perception MUST NOT be allowed to publish flight state or execute payload
            assert "rt/full_self_driving/state" not in pub_topics
            assert "rq/full_self_driving/prepare_payloadRequest" not in pub_topics
            # Perception MUST publish live target lock
            assert "rt/full_self_driving/live_target_lock" in pub_topics

        elif enclave == "flight_runtime":
            # Flight runtime MUST publish state, telemetry, and readiness
            assert "rt/full_self_driving/state" in pub_topics
            assert "rt/full_self_driving/readiness" in pub_topics
            assert "rt/full_self_driving/telemetry" in pub_topics
            # Flight runtime MUST NOT publish raw camera streams
            assert "rt/camera" not in pub_topics
            assert "rt/camera_info" not in pub_topics

        elif enclave == "gateway":
            # Gateway cannot directly publish authoritative flight state
            assert "rt/full_self_driving/state" not in pub_topics
            # Gateway CAN publish target selection and service requests as client
            assert "rt/full_self_driving/target_selection" in pub_topics
            assert "rq/full_self_driving/prepare_payloadRequest" in pub_topics

    def test_rogue_node_rejection_policy(self, keystore_path):
        """Simulate a rogue node without an enclave or grant attempting access."""
        rogue_enclave = "rogue_attacker_node"
        enc_dir = os.path.join(keystore_path, "enclaves", "full_self_driving", rogue_enclave)
        # Rogue enclave should not exist in verified keystore
        assert not os.path.exists(enc_dir)

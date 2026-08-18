#!/usr/bin/env python3
"""
Automated SROS2 PKI & Keystore Generator for Full Self-Driving Stack.
Generates Root CA, Node Certificates (X.509), OMG DDS Security Governance,
and Granular Least-Privilege Permissions signed as PKCS#7 (.p7s) artifacts.
"""

import argparse
import datetime
import os
import subprocess
import sys
from pathlib import Path


# Least-privilege access rules per enclave
ENCLAVE_POLICIES = {
    "/full_self_driving/flight_runtime": {
        "publish": [
            "rt/full_self_driving/state",
            "rt/full_self_driving/readiness",
            "rt/full_self_driving/safety",
            "rt/full_self_driving/telemetry",
            "rt/full_self_driving/plan/working_status",
            "rt/full_self_driving/payload/status",
            "rt/rosout",
            "rt/parameter_events",
            # Service responses
            "rr/full_self_driving/emergency_stopReply",
            "rr/full_self_driving/prepare_payloadReply",
            "rr/full_self_driving/fsd_flight_runtime/*",
        ],
        "subscribe": [
            "rt/full_self_driving/live_target_lock",
            "rt/clock",
            "rt/tf",
            "rt/tf_static",
            "rt/fmu/out/*",
            # Service requests
            "rq/full_self_driving/emergency_stopRequest",
            "rq/full_self_driving/prepare_payloadRequest",
            "rq/full_self_driving/fsd_flight_runtime/*",
        ],
    },
    "/full_self_driving/perception": {
        "publish": [
            "rt/full_self_driving/all_id_observations",
            "rt/full_self_driving/annotated_image",
            "rt/full_self_driving/live_target_lock",
            "rt/full_self_driving/health/perception",
            "rt/rosout",
            "rt/parameter_events",
            "rr/full_self_driving/fsd_perception/*",
        ],
        "subscribe": [
            "rt/camera",
            "rt/camera_info",
            "rt/full_self_driving/target_selection",
            "rt/clock",
            "rt/tf",
            "rt/tf_static",
            "rq/full_self_driving/fsd_perception/*",
        ],
    },
    "/full_self_driving/pad_registry": {
        "publish": [
            "rt/full_self_driving/pad_registry/snapshot",
            "rt/full_self_driving/pad_registry/status",
            "rt/full_self_driving/health/pad_registry",
            "rt/rosout",
            "rt/parameter_events",
            "rr/full_self_driving/fsd_pad_registry/*",
        ],
        "subscribe": [
            "rt/full_self_driving/all_id_observations",
            "rt/clock",
            "rq/full_self_driving/fsd_pad_registry/*",
        ],
    },
    "/full_self_driving/evidence": {
        "publish": [
            "rt/full_self_driving/health/evidence",
            "rt/rosout",
            "rt/parameter_events",
            "rr/full_self_driving/fsd_evidence/*",
        ],
        "subscribe": [
            "rt/full_self_driving/state",
            "rt/full_self_driving/payload/status",
            "rt/full_self_driving/safety",
            "rt/clock",
            "rq/full_self_driving/fsd_evidence/*",
        ],
    },
    "/full_self_driving/gateway": {
        "publish": [
            "rt/full_self_driving/health/gateway",
            "rt/full_self_driving/target_selection",
            "rt/rosout",
            "rt/parameter_events",
            # Service requests (as client)
            "rq/full_self_driving/emergency_stopRequest",
            "rq/full_self_driving/prepare_payloadRequest",
            "rr/full_self_driving/fsd_gateway/*",
        ],
        "subscribe": [
            "rt/full_self_driving/state",
            "rt/full_self_driving/telemetry",
            "rt/full_self_driving/readiness",
            "rt/full_self_driving/payload/status",
            "rt/full_self_driving/plan/working_status",
            "rt/full_self_driving/pad_registry/snapshot",
            "rt/clock",
            # Service responses (as client)
            "rr/full_self_driving/emergency_stopReply",
            "rr/full_self_driving/prepare_payloadReply",
            "rq/full_self_driving/fsd_gateway/*",
        ],
    },
}


def create_governance_xml(filepath: Path):
    content = """<?xml version="1.0" encoding="UTF-8"?>
<dds xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
     xsi:noNamespaceSchemaLocation="omg_shared_ca_governance.xsd">
  <domain_access_rules>
    <domain_rule>
      <domains>
        <id_range>
          <min>0</min>
          <max>230</max>
        </id_range>
      </domains>
      <allow_unauthenticated_participants>false</allow_unauthenticated_participants>
      <enable_join_access_control>true</enable_join_access_control>
      <discovery_protection_kind>ENCRYPT</discovery_protection_kind>
      <liveliness_protection_kind>ENCRYPT</liveliness_protection_kind>
      <rtps_protection_kind>ENCRYPT</rtps_protection_kind>
      <topic_access_rules>
        <topic_rule>
          <topic_expression>*</topic_expression>
          <enable_discovery_protection>true</enable_discovery_protection>
          <enable_read_access_control>true</enable_read_access_control>
          <enable_write_access_control>true</enable_write_access_control>
          <metadata_protection_kind>ENCRYPT</metadata_protection_kind>
          <data_protection_kind>ENCRYPT</data_protection_kind>
        </topic_rule>
      </topic_access_rules>
    </domain_rule>
  </domain_access_rules>
</dds>
"""
    with open(filepath, "w", encoding="utf-8") as f:
        f.write(content)


def create_permissions_xml(filepath: Path, enclave_name: str, policy: dict):
    not_before = datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%S")
    not_after = (datetime.datetime.now(datetime.timezone.utc) + datetime.timedelta(days=3650)).strftime("%Y-%m-%dT%H:%M:%S")

    pub_topics = "\n".join([f"            <topic>{t}</topic>" for t in policy.get("publish", [])])
    sub_topics = "\n".join([f"            <topic>{t}</topic>" for t in policy.get("subscribe", [])])

    content = f"""<?xml version="1.0" encoding="UTF-8"?>
<dds xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
     xsi:noNamespaceSchemaLocation="omg_shared_ca_permissions.xsd">
  <permissions>
    <grant name="{enclave_name}">
      <subject_name>CN={enclave_name}</subject_name>
      <validity>
        <not_before>{not_before}</not_before>
        <not_after>{not_after}</not_after>
      </validity>
      <allow_rule>
        <domains>
          <id>0</id>
        </domains>
        <publish>
          <topics>
{pub_topics}
          </topics>
        </publish>
        <subscribe>
          <topics>
{sub_topics}
          </topics>
        </subscribe>
      </allow_rule>
      <default>DENY</default>
    </grant>
  </permissions>
</dds>
"""
    with open(filepath, "w", encoding="utf-8") as f:
        f.write(content)


def run_cmd(cmd: list):
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode != 0:
        raise RuntimeError(f"Command failed: {' '.join(cmd)}\nStderr: {res.stderr}")
    return res.stdout


def sign_p7s(xml_file: Path, p7s_file: Path, ca_cert: Path, ca_key: Path):
    cmd = [
        "openssl", "smime", "-sign",
        "-in", str(xml_file),
        "-out", str(p7s_file),
        "-signer", str(ca_cert),
        "-inkey", str(ca_key),
        "-outform", "PEM",
        "-nodetach",
    ]
    run_cmd(cmd)


def generate_keystore(keystore_dir: Path):
    keystore_dir = keystore_dir.resolve()
    pub_dir = keystore_dir / "public"
    priv_dir = keystore_dir / "private"
    enclaves_dir = keystore_dir / "enclaves"

    pub_dir.mkdir(parents=True, exist_ok=True)
    priv_dir.mkdir(parents=True, exist_ok=True)
    enclaves_dir.mkdir(parents=True, exist_ok=True)

    ca_key = priv_dir / "ca.key.pem"
    ca_cert = pub_dir / "ca.cert.pem"

    # 1. Generate Root CA
    if not ca_key.exists() or not ca_cert.exists():
        print(f"[INFO] Generating Root CA in {keystore_dir}...")
        run_cmd([
            "openssl", "req", "-x509", "-newkey", "rsa:2048",
            "-nodes", "-keyout", str(ca_key),
            "-out", str(ca_cert),
            "-days", "3650",
            "-subj", "/CN=FullSelfDriving_Root_CA/O=FullSelfDriving/C=US",
        ])
        os.chmod(ca_key, 0o600)

    # 2. Generate Enclaves for each autonomy node
    for enclave_name, policy in ENCLAVE_POLICIES.items():
        rel_enclave = enclave_name.strip("/")
        enc_dir = enclaves_dir / rel_enclave
        enc_dir.mkdir(parents=True, exist_ok=True)

        node_key = enc_dir / "key.pem"
        node_csr = enc_dir / "req.csr"
        node_cert = enc_dir / "cert.pem"
        node_ca_cert = enc_dir / "identity_ca.cert.pem"
        node_perm_ca = enc_dir / "permissions_ca.cert.pem"

        gov_xml = enc_dir / "governance.xml"
        gov_p7s = enc_dir / "governance.p7s"
        perm_xml = enc_dir / "permissions.xml"
        perm_p7s = enc_dir / "permissions.p7s"

        # Copy identity and permissions CA certificates
        with open(ca_cert, "rb") as src, open(node_ca_cert, "wb") as dst:
            dst.write(src.read())
        with open(ca_cert, "rb") as src, open(node_perm_ca, "wb") as dst:
            dst.write(src.read())

        # Generate Node Key & Certificate
        escaped_enclave = enclave_name.replace("/", "\\/")
        if not node_key.exists() or not node_cert.exists():
            print(f"[INFO] Provisioning enclave identity: {enclave_name}")
            run_cmd([
                "openssl", "req", "-new", "-newkey", "rsa:2048",
                "-nodes", "-keyout", str(node_key),
                "-out", str(node_csr),
                "-subj", f"/CN={escaped_enclave}/O=FullSelfDriving/C=US",
            ])
            run_cmd([
                "openssl", "x509", "-req",
                "-in", str(node_csr),
                "-CA", str(ca_cert),
                "-CAkey", str(ca_key),
                "-CAcreateserial",
                "-out", str(node_cert),
                "-days", "3650",
            ])
            if node_csr.exists():
                node_csr.unlink()
            os.chmod(node_key, 0o600)

        # Generate Governance & Permissions XML
        create_governance_xml(gov_xml)
        create_permissions_xml(perm_xml, enclave_name, policy)

        # Sign XML into PKCS#7 .p7s bundles
        sign_p7s(gov_xml, gov_p7s, ca_cert, ca_key)
        sign_p7s(perm_xml, perm_p7s, ca_cert, ca_key)

    print(f"[SUCCESS] SROS2 Keystore successfully generated at: {keystore_dir}")


def main():
    parser = argparse.ArgumentParser(description="Generate SROS2 Keystore for Full Self-Driving nodes")
    parser.add_argument(
        "--keystore-dir",
        type=str,
        default=os.path.join(os.path.dirname(__file__), "..", "config", "security", "sample_keystore"),
        help="Target keystore directory",
    )
    args = parser.parse_args()
    generate_keystore(Path(args.keystore_dir))


if __name__ == "__main__":
    main()

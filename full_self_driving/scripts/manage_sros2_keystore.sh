#!/bin/bash
# ==============================================================================
# SROS2 Keystore Management Utility for Full Self-Driving Stack
# Provides commands to generate, verify, inspect, and rotate keystores.
# ==============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
DEFAULT_KEYSTORE="${PACKAGE_DIR}/config/security/sample_keystore"

usage() {
    echo "Usage: $0 {generate|verify|inspect|rotate|clean} [KEYSTORE_DIR]"
    echo ""
    echo "Commands:"
    echo "  generate [DIR]  Generate Root CA and all autonomy node enclaves"
    echo "  verify   [DIR]  Verify certificate validity, signatures, and permissions"
    echo "  inspect  [DIR]  Print certificate details and permissions summary"
    echo "  rotate   [DIR]  Rotate all node certificates preserving Root CA"
    echo "  clean    [DIR]  Remove keystore artifacts"
    exit 1
}

CMD="${1:-}"
KEYSTORE_DIR="${2:-${DEFAULT_KEYSTORE}}"

if [ -z "${CMD}" ]; then
    usage
fi

case "${CMD}" in
    generate)
        echo "[INFO] Generating SROS2 Keystore at: ${KEYSTORE_DIR}"
        python3 "${SCRIPT_DIR}/generate_sros2_keystore.py" --keystore-dir "${KEYSTORE_DIR}"
        ;;

    verify)
        echo "[INFO] Verifying SROS2 Keystore at: ${KEYSTORE_DIR}"
        CA_CERT="${KEYSTORE_DIR}/public/ca.cert.pem"
        if [ ! -f "${CA_CERT}" ]; then
            echo "[ERROR] CA certificate not found: ${CA_CERT}"
            exit 1
        fi
        
        # Verify CA cert
        openssl x509 -in "${CA_CERT}" -noout -subject -dates
        
        # Verify each node certificate against CA
        for enc in flight_runtime perception pad_registry gateway evidence; do
            CERT="${KEYSTORE_DIR}/enclaves/full_self_driving/${enc}/cert.pem"
            PERM_P7S="${KEYSTORE_DIR}/enclaves/full_self_driving/${enc}/permissions.p7s"
            GOV_P7S="${KEYSTORE_DIR}/enclaves/full_self_driving/${enc}/governance.p7s"
            
            if [ -f "${CERT}" ]; then
                echo -n "[OK] Node /full_self_driving/${enc} cert: "
                openssl verify -CAfile "${CA_CERT}" "${CERT}"
            else
                echo "[FAIL] Missing cert for enclave: ${enc}"
                exit 1
            fi

            # Verify PKCS#7 signatures
            if [ -f "${PERM_P7S}" ]; then
                openssl smime -verify -inform PEM -in "${PERM_P7S}" -CAfile "${CA_CERT}" >/dev/null 2>&1 && \
                    echo "     -> permissions.p7s signature verified." || (echo "     -> permissions.p7s signature INVALID!" && exit 1)
            fi
            if [ -f "${GOV_P7S}" ]; then
                openssl smime -verify -inform PEM -in "${GOV_P7S}" -CAfile "${CA_CERT}" >/dev/null 2>&1 && \
                    echo "     -> governance.p7s signature verified." || (echo "     -> governance.p7s signature INVALID!" && exit 1)
            fi
        done
        echo "[SUCCESS] All certificates and signed policies verified successfully."
        ;;

    inspect)
        echo "[INFO] Inspecting SROS2 Keystore: ${KEYSTORE_DIR}"
        for enc in flight_runtime perception pad_registry gateway evidence; do
            CERT="${KEYSTORE_DIR}/enclaves/full_self_driving/${enc}/cert.pem"
            PERM_XML="${KEYSTORE_DIR}/enclaves/full_self_driving/${enc}/permissions.xml"
            if [ -f "${CERT}" ]; then
                echo "=================================================="
                echo "Enclave: /full_self_driving/${enc}"
                openssl x509 -in "${CERT}" -noout -subject -issuer -dates
                if [ -f "${PERM_XML}" ]; then
                    echo "Permissions rules:"
                    grep -E "<(topic|publish|subscribe|default)>" "${PERM_XML}" | sed 's/^[ \t]*/  /'
                fi
            fi
        done
        ;;

    rotate)
        echo "[INFO] Rotating node certificates in: ${KEYSTORE_DIR}"
        # Keep CA, remove node enclaves
        rm -rf "${KEYSTORE_DIR}/enclaves"
        python3 "${SCRIPT_DIR}/generate_sros2_keystore.py" --keystore-dir "${KEYSTORE_DIR}"
        echo "[SUCCESS] Enclave certificates rotated successfully."
        ;;

    clean)
        echo "[INFO] Cleaning keystore directory: ${KEYSTORE_DIR}"
        rm -rf "${KEYSTORE_DIR}"
        echo "[SUCCESS] Keystore removed."
        ;;

    *)
        usage
        ;;
esac

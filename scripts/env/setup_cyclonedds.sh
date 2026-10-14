#!/usr/bin/env bash
# ==============================================================================
# TuinaDex v2.0 - Eclipse CycloneDDS Cross-Distro Setup Script
# Standardizes DDS RMW implementation and domain across Ubuntu 26.04 and 22.04
# Specification: docs/superpowers/specs/2026-10-14-environment-management-architecture-design.md
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_XML_PATH="${SCRIPT_DIR}/cyclonedds.xml"
ROS_DOMAIN_ID_TARGET=42
RMW_TARGET="rmw_cyclonedds_cpp"

generate_xml() {
    local target_file="$1"
    mkdir -p "$(dirname "${target_file}")"
    cat <<EOF > "${target_file}"
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain Id="${ROS_DOMAIN_ID_TARGET}">
        <General>
            <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
            <AllowMulticast>true</AllowMulticast>
            <MaxMessageSize>65500B</MaxMessageSize>
        </General>
        <Discovery>
            <ParticipantIndex>auto</ParticipantIndex>
        </Discovery>
        <Internal>
            <Watermarks>
                <WhcHigh>500kB</WhcHigh>
            </Watermarks>
        </Internal>
    </Domain>
</CycloneDDS>
EOF
    echo "[TD_v2.0-DDS] CycloneDDS XML generated at: ${target_file}"
}

# Mode 1: Argument handling for generation only
if [ "$1" = "--generate-xml" ]; then
    if [ -n "$2" ]; then
        generate_xml "$2"
    else
        generate_xml "${DEFAULT_XML_PATH}"
    fi
    exit 0
fi

# Ensure default XML exists
if [ ! -f "${DEFAULT_XML_PATH}" ]; then
    generate_xml "${DEFAULT_XML_PATH}"
fi

# Mode 2: Install into ~/.bashrc
if [ "$1" = "--install" ]; then
    BASHRC="${HOME}/.bashrc"
    MARKER="# >>> TuinaDex v2.0 CycloneDDS Baseline >>>"
    if grep -q "${MARKER}" "${BASHRC}" 2>/dev/null; then
        echo "[TD_v2.0-DDS] CycloneDDS configuration already exists in ${BASHRC}."
    else
        cat <<EOF >> "${BASHRC}"

${MARKER}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID_TARGET}
export RMW_IMPLEMENTATION=${RMW_TARGET}
export CYCLONEDDS_URI="file://${DEFAULT_XML_PATH}"
# <<< TuinaDex v2.0 CycloneDDS Baseline <<<
EOF
        echo "[TD_v2.0-DDS] Successfully installed TuinaDex DDS configuration into ${BASHRC}."
    fi
    exit 0
fi

# Mode 3: Direct export for current shell
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID_TARGET}
export RMW_IMPLEMENTATION=${RMW_TARGET}
export CYCLONEDDS_URI="file://${DEFAULT_XML_PATH}"

echo "[TD_v2.0-DDS] Environment active for current shell:"
echo "  - ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
echo "  - RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION}"
echo "  - CYCLONEDDS_URI: ${CYCLONEDDS_URI}"

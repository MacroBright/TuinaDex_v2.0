"""
TuinaDex v2.0 - Environment Management & Health Check Test Suite
Verifies that all environment scripts, udev rules, service units, and Docker configurations
comply with SPEC-ENV-2026-10-14.
"""

import os
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path
import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent


def test_task1_scripts_exist_and_executable():
    """Verify that Task 1 setup and diagnostic scripts exist and are executable."""
    setup_script = REPO_ROOT / "scripts" / "env" / "setup_cyclonedds.sh"
    check_script = REPO_ROOT / "scripts" / "env" / "check_env.sh"

    assert setup_script.exists(), "scripts/env/setup_cyclonedds.sh does not exist"
    assert os.access(setup_script, os.X_OK), "scripts/env/setup_cyclonedds.sh is not executable"

    assert check_script.exists(), "scripts/env/check_env.sh does not exist"
    assert os.access(check_script, os.X_OK), "scripts/env/check_env.sh is not executable"


def test_task1_bash_syntax():
    """Ensure bash scripts have valid syntax via `bash -n`."""
    scripts = [
        REPO_ROOT / "scripts" / "env" / "setup_cyclonedds.sh",
        REPO_ROOT / "scripts" / "env" / "check_env.sh",
    ]
    for script in scripts:
        if script.exists():
            res = subprocess.run(["bash", "-n", str(script)], capture_output=True, text=True)
            assert res.returncode == 0, f"Syntax error in {script}: {res.stderr}"


def test_cyclonedds_xml_validity(tmp_path):
    """Verify setup_cyclonedds.sh generates valid CycloneDDS XML."""
    setup_script = REPO_ROOT / "scripts" / "env" / "setup_cyclonedds.sh"
    if not setup_script.exists():
        pytest.skip("setup_cyclonedds.sh not yet created")

    # Run setup script pointing output to tmp_path
    xml_out = tmp_path / "cyclonedds.xml"
    res = subprocess.run(
        ["bash", str(setup_script), "--generate-xml", str(xml_out)],
        capture_output=True,
        text=True,
    )
    assert res.returncode == 0, f"Script failed: {res.stderr}"
    assert xml_out.exists(), "cyclonedds.xml was not generated"

    # Parse XML to verify well-formedness
    tree = ET.parse(xml_out)
    root = tree.getroot()
    assert "CycloneDDS" in root.tag
    domain = root.find(".//{*}Domain")
    if domain is None:
        domain = root.find("Domain")
    assert domain is not None, "Domain element not found in CycloneDDS XML"
    domain_id = domain.attrib.get("Id") or domain.find(".//{*}Id")
    if domain_id is not None:
        val = domain_id if isinstance(domain_id, str) else domain_id.text
        assert val == "42" or val == "any"

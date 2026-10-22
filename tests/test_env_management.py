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


def test_task2_dev_pc_rules_and_init():
    """Verify Task 2 udev rules and dev PC init script."""
    rules_file = REPO_ROOT / "scripts" / "env" / "99-tuinadex-devices.rules"
    init_script = REPO_ROOT / "scripts" / "env" / "init_dev_pc.sh"

    assert rules_file.exists(), "scripts/env/99-tuinadex-devices.rules does not exist"
    rules_content = rules_file.read_text()
    assert "tuinadex_hand" in rules_content, "Missing tuinadex_hand symlink rule"
    assert "0666" in rules_content, "Missing 0666 permissions in udev rules"

    assert init_script.exists(), "scripts/env/init_dev_pc.sh does not exist"
    assert os.access(init_script, os.X_OK), "scripts/env/init_dev_pc.sh is not executable"
    res = subprocess.run(["bash", "-n", str(init_script)], capture_output=True, text=True)
    assert res.returncode == 0, f"Syntax error in init_dev_pc.sh: {res.stderr}"


def test_task3_orangepi_service_and_init():
    """Verify Task 3 systemd CAN service and Orange Pi init script."""
    service_file = REPO_ROOT / "scripts" / "env" / "tuinadex-can.service"
    init_script = REPO_ROOT / "scripts" / "env" / "init_orangepi.sh"

    assert service_file.exists(), "scripts/env/tuinadex-can.service does not exist"
    service_content = service_file.read_text()
    assert "[Unit]" in service_content, "Missing [Unit] section in systemd service"
    assert "[Service]" in service_content, "Missing [Service] section in systemd service"
    assert "bitrate 1000000" in service_content, "Missing 1Mbps bitrate configuration"

    assert init_script.exists(), "scripts/env/init_orangepi.sh does not exist"
    assert os.access(init_script, os.X_OK), "scripts/env/init_orangepi.sh is not executable"
    res = subprocess.run(["bash", "-n", str(init_script)], capture_output=True, text=True)
    assert res.returncode == 0, f"Syntax error in init_orangepi.sh: {res.stderr}"


def test_task4_data_docker_and_script():
    """Verify Task 4 data cleaning Dockerfile and wrapper script."""
    dockerfile = REPO_ROOT / "docker" / "data.Dockerfile"
    script = REPO_ROOT / "scripts" / "run_data_convert.sh"

    assert dockerfile.exists(), "docker/data.Dockerfile does not exist"
    df_content = dockerfile.read_text()
    assert "FROM " in df_content, "Missing FROM line in data.Dockerfile"
    assert "lerobot" in df_content.lower(), "Missing lerobot dependency in data.Dockerfile"
    assert "WORKDIR" in df_content, "Missing WORKDIR in data.Dockerfile"

    assert script.exists(), "scripts/run_data_convert.sh does not exist"
    assert os.access(script, os.X_OK), "scripts/run_data_convert.sh is not executable"
    res = subprocess.run(["bash", "-n", str(script)], capture_output=True, text=True)
    assert res.returncode == 0, f"Syntax error in run_data_convert.sh: {res.stderr}"


def test_task5_vla_cloud_docker_and_script():
    """Verify Task 5 cloud VLA Dockerfile and launch script."""
    dockerfile = REPO_ROOT / "docker" / "vla_cloud.Dockerfile"
    script = REPO_ROOT / "scripts" / "launch_vla_cloud.sh"

    assert dockerfile.exists(), "docker/vla_cloud.Dockerfile does not exist"
    df_content = dockerfile.read_text()
    assert "nvidia/cuda" in df_content, "Missing nvidia/cuda base in vla_cloud.Dockerfile"
    assert "grpc" in df_content.lower(), "Missing grpc dependency in vla_cloud.Dockerfile"
    assert "EXPOSE" in df_content, "Missing EXPOSE directive in vla_cloud.Dockerfile"

    assert script.exists(), "scripts/launch_vla_cloud.sh does not exist"
    assert os.access(script, os.X_OK), "scripts/launch_vla_cloud.sh is not executable"
    res = subprocess.run(["bash", "-n", str(script)], capture_output=True, text=True)
    assert res.returncode == 0, f"Syntax error in launch_vla_cloud.sh: {res.stderr}"


def test_task6_verify_all_script():
    """Verify Task 6 holistic verification script."""
    verify_script = REPO_ROOT / "scripts" / "verify_all_envs.sh"

    assert verify_script.exists(), "scripts/verify_all_envs.sh does not exist"
    assert os.access(verify_script, os.X_OK), "scripts/verify_all_envs.sh is not executable"
    res = subprocess.run(["bash", "-n", str(verify_script)], capture_output=True, text=True)
    assert res.returncode == 0, f"Syntax error in verify_all_envs.sh: {res.stderr}"


def test_task7_consolidated_conda_environments_and_activation():
    """Verify Task 7 consolidated conda environment specs and activation script."""
    activator = REPO_ROOT / "scripts" / "activate_tuinadex.sh"
    control_yaml = REPO_ROOT / "scripts" / "env" / "tuinadex-control.yml"
    vla_yaml = REPO_ROOT / "scripts" / "env" / "tuinadex-vla.yml"

    # 1. Activator existence, permissions, and bash syntax
    assert activator.exists(), "scripts/activate_tuinadex.sh does not exist"
    assert os.access(activator, os.X_OK), "scripts/activate_tuinadex.sh is not executable"
    res = subprocess.run(["bash", "-n", str(activator)], capture_output=True, text=True)
    assert res.returncode == 0, f"Syntax error in activate_tuinadex.sh: {res.stderr}"

    activator_content = activator.read_text()
    assert "tuinadex-control" in activator_content
    assert "tuinadex-vla" in activator_content
    assert "setup_cyclonedds.sh" in activator_content

    # 2. YAML environment specs
    assert control_yaml.exists(), "scripts/env/tuinadex-control.yml does not exist"
    control_text = control_yaml.read_text()
    assert "name: tuinadex-control" in control_text
    assert "python=3.10" in control_text

    assert vla_yaml.exists(), "scripts/env/tuinadex-vla.yml does not exist"
    vla_text = vla_yaml.read_text()
    assert "name: tuinadex-vla" in vla_text
    assert "python=3.10" in vla_text


def test_task8_environment_change_gate_rules():
    """Verify that the Environment Change Gate is codified in AGENTS.md and SESSION_STATE.md."""
    agents_file = REPO_ROOT / "AGENTS.md"
    session_file = REPO_ROOT / ".agent" / "SESSION_STATE.md"

    assert agents_file.exists(), "AGENTS.md does not exist"
    agents_content = agents_file.read_text()
    assert "环境管理与依赖变更门禁" in agents_content
    assert "verify_all_envs.sh" in agents_content
    assert "ENVIRONMENT_MANAGEMENT_SPEC.md" in agents_content

    assert session_file.exists(), ".agent/SESSION_STATE.md does not exist"
    session_content = session_file.read_text()
    assert "不变量 24" in session_content
    assert "verify_all_envs.sh" in session_content








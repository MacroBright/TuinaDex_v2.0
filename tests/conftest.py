"""conftest.py — pytest configuration and fixtures."""
import os

# Isolate ROS 2 domain during testing to prevent crosstalk with running daemon/GUI nodes
os.environ.setdefault("ROS_DOMAIN_ID", "42")


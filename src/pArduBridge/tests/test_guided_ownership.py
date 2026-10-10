#!/usr/bin/env python3
"""Isolated ownership/ACK lease regressions; no MOOS, FC, or flight commands."""

import os
import subprocess
import tempfile
import unittest
from pathlib import Path


class GuidedOwnershipTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory(prefix="guided-ownership-")
        cls.addClassCleanup(cls.directory.cleanup)
        cls.binary = Path(cls.directory.name) / "test_guided_ownership"
        subprocess.run(
            [
                os.environ.get("CXX", "c++"),
                "-std=c++17",
                "-Wall",
                "-Wextra",
                "-Werror",
                str(Path(__file__).with_suffix(".cpp")),
                "-o",
                str(cls.binary),
            ],
            check=True,
            timeout=30,
        )

    def check_case(self, name):
        subprocess.run([str(self.binary), name], check=True, timeout=5)

    def test_native_classification(self):
        self.check_case("classification")

    def test_warning_grace_and_retraction(self):
        self.check_case("timing")

    def test_ack_lease_expiry_and_revocation(self):
        self.check_case("leases")


if __name__ == "__main__":
    unittest.main()

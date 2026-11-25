"""The firmware app must use the same drone mass as the Python model (the post-fault margin is tiny)."""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from ftqc.params import QuadParams  # noqa: E402

APP_CONFIG = os.path.join(os.path.dirname(__file__), "..", "firmware", "app_ftc", "app-config")


def read_app_config(path=APP_CONFIG):
    with open(path) as fh:
        lines = [ln.strip() for ln in fh if ln.strip() and not ln.startswith("#")]
    return dict(ln.split("=", 1) for ln in lines)


class TestFirmwareConfig(unittest.TestCase):
    def test_firmware_mass_override_is_enabled(self):
        self.assertEqual(read_app_config().get("CONFIG_MODIFY_CF_MASS"), "y")

    def test_firmware_mass_matches_python_model(self):
        mass_mg = int(read_app_config()["CONFIG_MODIFIED_CF_MASS"])
        self.assertEqual(mass_mg, round(QuadParams().mass * 1e6))


if __name__ == "__main__":
    unittest.main()

"""Patch conditional_experimental_mode.py lightboost lookup.

Replaces the line that reads tuned_boost = self.LIGHT_BOOSTS[1] with
tuned_boost = 1.0.  The 1.2 multiplier false-triggered Conditional
Experimental Mode on stop signs / traffic lights at highway speed.
Confirmed fixed on-road 2026-08-03.  LIGHT_BOOSTS is intentionally left
unchanged because it is used elsewhere.
"""

import os
import sys

CEM_FILE = "/data/openpilot/starpilot/controls/lib/conditional_experimental_mode.py"
OLD_LIGHTBOOST = """      tuned_boost = self.LIGHT_BOOSTS[1]
"""
NEW_LIGHTBOOST = """      tuned_boost = 1.0  # CEM_LIGHTBOOST_PATCHED: was LIGHT_BOOSTS[1]=1.2, false-triggered CEM stop/light on highway
"""


def main():
    if not os.path.isfile(CEM_FILE):
        print(f"ERROR: {CEM_FILE} missing")
        return 1

    with open(CEM_FILE) as f:
        content = f.read()

    if "CEM_LIGHTBOOST_PATCHED" in content:
        print("conditional_experimental_mode.py already patched")
        return 0

    if OLD_LIGHTBOOST not in content:
        print("ERROR: expected tuned_boost assignment not found in conditional_experimental_mode.py; patch not applied")
        return 1

    content = content.replace(OLD_LIGHTBOOST, NEW_LIGHTBOOST, 1)

    with open(CEM_FILE, "w") as f:
        f.write(content)

    print("conditional_experimental_mode.py patched for CEM lightboost")
    return 0



if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
# TSS-P Camry (sDSU LONG_FILTER via patch_sdsu_long.py): admit the car to
# Conditional Experimental Mode (CEM) startup.
#
# feature_runtime.py: upstream feature_enabled(params, cp, 'conditional', env)
#   admits only the Ioniq 6 (ioniq6_long_eligible) and GM long cars
#   (gm_long_supported); both require not pcmCruise, so on this pcmCruise=True
#   Camry the gate is False and every caller (selfdrived, plannerd, radard,
#   ui) declines to construct the CEM owners at all. With the marker file
#   /data/ops_patches/camry_cem present (os.path.isfile, read once at import)
#   the 'conditional' feature additionally admits a brand == 'toyota' CarParams
#   passing experimental_available() (openpilotLongitudinalControl True, not
#   passive/dashcamOnly/notCar; pcmCruise is not consulted there or anywhere
#   downstream in runtime_settings/policy/host/consumer/plannerd/radard).
#   Absent marker = upstream admission exactly. curve / slc / profile / aol
#   admission is unchanged.
#   The marker is import-time state: it takes effect when the consumers next
#   start (manager restarts plannerd/selfdrived/radard each drive). No params
#   are written; CEM still needs its usual runtime authority (ready settings
#   verdict, SafeMode off, fresh CAN/model, enabled + longActive).
import os
import sys

STARPILOT = "/data/openpilot/openpilot/starpilot"

EDITS = {
  "feature_runtime.py": [
    ("""from collections.abc import Mapping
from dataclasses import replace""",
     """import os
from collections.abc import Mapping
from dataclasses import replace"""),
    ("""from openpilot.starpilot.conditional_mode.preferences import PreferenceError, decode_preferences""",
     """from openpilot.starpilot.conditional_mode.preferences import PreferenceError, decode_preferences
from openpilot.starpilot.conditional_mode.runtime_settings import experimental_available"""),
    ("""from openpilot.starpilot.speed_limits.selection import Source""",
     """from openpilot.starpilot.speed_limits.selection import Source


# /data/ops_patches/camry_cem (read once at import) also admits a TSS-P Toyota
# with openpilot longitudinal (the sDSU LONG_FILTER Camry from patch_sdsu_long)
# to the conditional-mode startup gate upstream reserves for the Ioniq 6 and
# GM long cars. Absent marker = upstream admission exactly.
CAMRY_CEM = os.path.isfile("/data/ops_patches/camry_cem")


def _toyota_cem_capable(cp) -> bool:
  # experimental_available: openpilotLongitudinalControl, not passive/dashcamOnly/notCar
  return bool(CAMRY_CEM and getattr(cp, 'brand', None) == 'toyota' and experimental_available(cp))"""),
    ("""  capable = cp is not None and (ioniq6_long_eligible(cp) or
                                (feature in ('conditional', 'curve', 'slc', 'profile') and gm_long_supported(cp)) or
                                (feature in ('slc', 'vision') and gm_display_supported(cp)))""",
     """  capable = cp is not None and (ioniq6_long_eligible(cp) or
                                (feature == 'conditional' and _toyota_cem_capable(cp)) or
                                (feature in ('conditional', 'curve', 'slc', 'profile') and gm_long_supported(cp)) or
                                (feature in ('slc', 'vision') and gm_display_supported(cp)))"""),
  ],
}


def main():
  contents = {}
  for name in EDITS:
    path = os.path.join(STARPILOT, name)
    if not os.path.isfile(path):
      print(f"ERROR: {path} missing")
      return 1
    with open(path) as f:
      contents[name] = f.read()

  # validate every anchor before writing anything
  pending = {}
  for name, edits in EDITS.items():
    content = contents[name]
    todo = []
    for old, new in edits:
      if new in content:
        continue
      if content.count(old) != 1:
        print(f"ERROR: anchor found {content.count(old)}x (expected 1) in {name}; patch not applied")
        print(f"  anchor: {old.splitlines()[0]!r}")
        return 1
      todo.append((old, new))
    pending[name] = todo

  if not any(pending.values()):
    print("camry cem already applied")
    return 0

  for name, todo in pending.items():
    content = contents[name]
    for old, new in todo:
      content = content.replace(old, new, 1)
    with open(os.path.join(STARPILOT, name), "w") as f:
      f.write(content)
    print(f"patched {name} ({len(todo)} edits)")

  return 0


if __name__ == "__main__":
  sys.exit(main())

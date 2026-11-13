#!/usr/bin/env python3
# Allow Conditional Experimental Mode on TSS-P Camry
#
# feature_runtime.py: upstream feature_enabled(params, cp, 'conditional', env)
#   routes capability through starpilot/feature_capabilities.py
#   (longitudinal_supported: openpilotLongitudinalControl True, not
#   passive/dashcamOnly/notCar, GM additionally validated by its native
#   safety-word configuration). The 'conditional' feature also admits a
#   brand == 'toyota' CarParams passing experimental_available()
#   (openpilotLongitudinalControl True, not passive/dashcamOnly/notCar;
#   pcmCruise is not consulted there or anywhere downstream in
#   runtime_settings/policy/host/consumer/plannerd/radard).
#   curve / slc / profile / aol admission is unchanged.
#   No params are written; CEM still needs its usual runtime authority (ready
#   settings verdict, SafeMode off, fresh CAN/model, enabled + longActive).
import os
import sys

STARPILOT = "/data/openpilot/openpilot/starpilot"

EDITS = {
  "feature_runtime.py": [
    ("""from openpilot.starpilot.conditional_mode.preferences import PreferenceError, decode_preferences""",
     """from openpilot.starpilot.conditional_mode.preferences import PreferenceError, decode_preferences
from openpilot.starpilot.conditional_mode.runtime_settings import experimental_available"""),
    ("""from openpilot.starpilot.speed_limits.selection import Source""",
     """from openpilot.starpilot.speed_limits.selection import Source


# Admit a Toyota with openpilot longitudinal to the conditional-mode
# startup gate. feature_capabilities.longitudinal_supported does not.
def _toyota_cem_capable(cp) -> bool:
  # experimental_available: openpilotLongitudinalControl, not passive/dashcamOnly/notCar
  return bool(getattr(cp, 'brand', None) == 'toyota' and experimental_available(cp))"""),
    ("""  capable = display_supported(cp) if feature in ('slc', 'vision') else longitudinal_supported(cp)""",
     """  capable = display_supported(cp) if feature in ('slc', 'vision') else longitudinal_supported(cp)
  capable = capable or (feature == 'conditional' and _toyota_cem_capable(cp))"""),
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

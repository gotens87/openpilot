#!/usr/bin/env python3
# /data/ops_patches/patch_sdsu_long.py
# TSS-P Camry: enable openpilot longitudinal when sDSU / radar filter hardware
# is present (0x2FF or 0x2AA on bus 0).
#
# interface.py: detect the hardware and mark LONG_FILTER. With the Camry's
#   default EPS factor of 73 this lands safetyParam on exactly 4169, the value
#   panda matches to select the filter TX set, and enables openpilot
#   longitudinal. RAISED_ACCEL_LIMIT, the hybrid actuator delay, the SDSU
#   parser entry and the SDSU distance button all follow from LONG_FILTER.
# carcontroller.py: run the TSS-P standstill latch for every LONG_FILTER car,
#   not just the Prius. Without it RELEASE_STANDSTILL stays 1 on this hybrid
#   and the PCM never latches standstill-hold. The marker file
#   /data/ops_patches/camry_sng (os.path.isfile, checked once at import)
#   restores the old SNG hack: the latch is never set and always cleared, so
#   the car auto-resumes from standstill behind a lead. No marker = stock
#   latch behaviour. The auto-hold / brake_hold path is untouched.
# carstate.py: keep LDA_ON_MESSAGE off non-Prius TSS-P cars; there it pulses
#   with the stock LDA toggle, it is not a button.
# carstate.py: match the comma UI's displayed speed to the Camry dash, which
#   over-reads true speed by ~4% (57 vs 59.5 kph set speed) while stock
#   vEgoCluster only adds 1.5%, leaving the screen ~2.5 km/h low. The marker
#   file /data/ops_patches/camry_cluster_speed is read once at import into
#   CAMRY_CLUSTER_RATIO: missing file = None (stock 1.015 for everyone),
#   present = float(contents), with empty/unparseable/outside 1.0..1.10
#   falling back to 1.04. Applied only on the LONG_FILTER path; vEgoCluster is
#   display-only here since pcmCruise is true.
import os
import sys

TOYOTA = "/data/openpilot/opendbc_repo/opendbc/car/toyota"

EDITS = {
  "interface.py": [
    ("""    stop_and_go = bool(ret.flags & ToyotaFlags.TSS2)""",
     """    # sDSU / radar filter hardware on the TSS-P Camry announces itself on bus 0
    camry_sdsu = candidate == CAR.TOYOTA_CAMRY and (0x2FF in fingerprint[0] or 0x2AA in fingerprint[0])
    if camry_sdsu:
      # sDSU / radar filter hardware needs the Toyota safety long-filter TX set
      ret.flags |= ToyotaFlags.LONG_FILTER.value
      ret.safetyConfigs[0].safetyParam |= ToyotaSafetyFlags.LONG_FILTER.value

    stop_and_go = bool(ret.flags & ToyotaFlags.TSS2)"""),
    ("""    ret.openpilotLongitudinalControl = ((bool(ret.flags & ToyotaFlags.TSS2) and not (ret.flags & ToyotaFlags.RADAR_ACC)) or
                                        bool(ret.flags & ToyotaFlags.DISABLE_RADAR.value))""",
     """    ret.openpilotLongitudinalControl = (camry_sdsu or
                                        (bool(ret.flags & ToyotaFlags.TSS2) and not (ret.flags & ToyotaFlags.RADAR_ACC)) or
                                        bool(ret.flags & ToyotaFlags.DISABLE_RADAR.value))"""),
  ],
  "carcontroller.py": [
    ("""import math
import numpy as np""",
     """import math
import os
import numpy as np"""),
    ("""VisualAlert = structs.CarControl.HUDControl.VisualAlert""",
     """VisualAlert = structs.CarControl.HUDControl.VisualAlert

# /data/ops_patches/camry_sng (read once at import) restores the old SNG-hack
# behaviour on the TSS-P long path: never latch standstill, always clear it,
# so the car auto-resumes from standstill behind a lead
CAMRY_SNG_HACK = os.path.isfile("/data/ops_patches/camry_sng")"""),
    ("""      if self.prius_longitudinal:
        if CS.out.standstill and not self.last_standstill:""",
     """      # TSS-P PCMs latch a standstill request that must be explicitly released to move again
      if self.CP.flags & ToyotaFlags.LONG_FILTER:
        if CS.out.standstill and not self.last_standstill:"""),
    ("""        if CS.out.standstill and not self.last_standstill:
          self.standstill_req = True""",
     """        if CS.out.standstill and not self.last_standstill and not CAMRY_SNG_HACK:
          self.standstill_req = True"""),
    ("""        if CS.pcm_acc_status != 8 or CC.cruiseControl.resume:
          self.standstill_req = False""",
     """        if CS.pcm_acc_status != 8 or CC.cruiseControl.resume or CAMRY_SNG_HACK:
          self.standstill_req = False"""),
    ("""        if not long_active and CS.out.standstill:
          self.standstill_req = True""",
     """        # camry_sng never latches standstill, so the inactive-at-stop re-latch must not either
        if not long_active and CS.out.standstill and not CAMRY_SNG_HACK:
          self.standstill_req = True"""),
  ],
  "carstate.py": [
    ("""import copy
import math""",
     """import copy
import math
import os"""),
    ("""SteerControlType = structs.CarParams.SteerControlType""",
     """SteerControlType = structs.CarParams.SteerControlType


def _camry_cluster_ratio():
  if not os.path.isfile("/data/ops_patches/camry_cluster_speed"):
    return None
  try:
    with open("/data/ops_patches/camry_cluster_speed") as f:
      ratio = float(f.read().strip())
  except (OSError, ValueError):
    return 1.04
  return ratio if 1.0 <= ratio <= 1.10 else 1.04


# /data/ops_patches/camry_cluster_speed (read once at import) scales the UI
# cluster speed to match the Camry dash's ~4% over-read; None = stock 1.015
CAMRY_CLUSTER_RATIO = _camry_cluster_ratio()"""),
    ("""    ret.vEgoCluster = ret.vEgo * 1.015  # minimum of all the cars""",
     """    # on the sDSU long path, the marker-controlled ratio matches the UI speed to the Camry dash
    cluster_ratio = CAMRY_CLUSTER_RATIO if (self.CP.flags & ToyotaFlags.LONG_FILTER and CAMRY_CLUSTER_RATIO is not None) else 1.015
    ret.vEgoCluster = ret.vEgo * cluster_ratio  # minimum of all the cars"""),
    ("""    if self.CP.flags & (ToyotaFlags.TSS2 | ToyotaFlags.LONG_FILTER) or self.CP.carFingerprint == CAR.TOYOTA_PRIUS_RETROFIT:
      # lkas button is wired to the camera""",
     """    # on TSS-P, LDA_ON_MESSAGE pulses with the stock LDA toggle, so only treat it as a button on the Prius
    if (self.CP.flags & ToyotaFlags.TSS2 or self.CP.carFingerprint == CAR.TOYOTA_PRIUS_RETROFIT or
        (self.CP.flags & ToyotaFlags.LONG_FILTER and self.CP.carFingerprint == CAR.TOYOTA_PRIUS)):"""),
  ],
}


def main():
  contents = {}
  for name in EDITS:
    path = os.path.join(TOYOTA, name)
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
    print("sdsu long already applied")
    return 0

  for name, todo in pending.items():
    content = contents[name]
    for old, new in todo:
      content = content.replace(old, new, 1)
    with open(os.path.join(TOYOTA, name), "w") as f:
      f.write(content)
    print(f"patched {name} ({len(todo)} edits)")

  return 0


if __name__ == "__main__":
  sys.exit(main())

#!/usr/bin/env python3
# comma 3: a restart requested while engaged waits, then reboots on disengage.
#
# manager.py leaves its main loop and reboots as soon as DoReboot is set.
# DoReboot is CLEAR_ON_MANAGER_START, so manager_init drops it before the
# loop on the boot that reboot starts, and the request cannot loop.
# IsEngaged is the persistent bool hardwared already writes from
# selfdriveState.enabled. While both are set, manager keeps running and
# takes the existing shutdown path on the first pass where IsEngaged is
# false. Callers sudo reboot themselves when it is already false.
# Dead-fan reboot stays HARDWARE.reboot("fan_dead") in hardwared.
import os
import sys

TREE = "/data/openpilot/openpilot"

EDITS = {
  "system/manager/manager.py": [
    ("""    # Exit main loop when uninstall/shutdown/reboot is needed
    shutdown = False
    for param in ("DoUninstall", "DoShutdown", "DoReboot"):
      if params.get_bool(param):
        shutdown = True""",
     """    # Exit main loop when uninstall/shutdown/reboot is needed.
    # DoReboot while IsEngaged waits for disengage; the key is cleared on manager start, so it cannot loop.
    shutdown = False
    for param in ("DoUninstall", "DoShutdown", "DoReboot"):
      if params.get_bool(param):
        if param == "DoReboot" and params.get_bool("IsEngaged"):
          continue
        shutdown = True"""),
  ],
}


def main():
  contents = {}
  for name in EDITS:
    path = os.path.join(TREE, name)
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
    print("reboot on disengage already applied")
    return 0

  for name, todo in pending.items():
    content = contents[name]
    for old, new in todo:
      content = content.replace(old, new, 1)
    with open(os.path.join(TREE, name), "w") as f:
      f.write(content)
    print(f"patched {name} ({len(todo)} edits)")

  return 0


if __name__ == "__main__":
  sys.exit(main())

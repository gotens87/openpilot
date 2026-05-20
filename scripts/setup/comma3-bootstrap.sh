#!/usr/bin/bash
# comma3-bootstrap: Recreates the custom /data/ environment after an AGNOS reflash.
# Idempotent — safe to re-run at any time to self-heal missing hooks.
# Vault: personal/vehicles/openpilot/comma3-device-setup.md Part 6.

set -euo pipefail

echo "==> Starting comma3 bootstrap..."

# 1. Comma egress block (banned-from-comma-servers device)
echo "--> Configuring /data/comma-block.sh egress block..."
cat > /data/comma-block.sh <<'EOF'
#!/usr/bin/bash
# Block all egress to comma.ai Azure subnet. Device is banned, want zero exfil.
# Konik.ai (Cloudflare 2606:4700:...) is on different IP space — unaffected.
# AGNOS uses iptables-legacy (nf_tables fallback fails). Vault: comma3-device-setup.md Part 2.
COMMA_CIDR=20.188.93.0/24
sudo iptables-legacy -C OUTPUT -d $COMMA_CIDR -j DROP 2>/dev/null || sudo iptables-legacy -I OUTPUT -d $COMMA_CIDR -j DROP
EOF
chmod +x /data/comma-block.sh

# 2. F2FS-tools offline cache (OTA-resilient persistence)
echo "--> Ensuring f2fs-tools .deb is cached at /data/..."
if ! ls /data/f2fs-tools_*.deb >/dev/null 2>&1; then
  echo "    Downloading f2fs-tools .deb (requires internet)..."
  (cd /data && sudo apt-get update >/dev/null && sudo apt-get download f2fs-tools >/dev/null)
else
  echo "    Cache present."
fi

# 3. NVMe filesystem check (non-destructive by default)
echo "--> Checking NVMe filesystem type..."
NVME_DEV="/dev/nvme0n1"
NVME_FS=$(sudo blkid -s TYPE -o value "$NVME_DEV" 2>/dev/null || true)

if [ "$NVME_FS" = "ext4" ] || [ -z "$NVME_FS" ]; then
  if [ "${1:-}" = "--reformat-nvme" ]; then
    echo "    [!] Reformatting NVMe to f2fs (DESTRUCTIVE — wipes all rlogs)..."
    sudo umount /data/media 2>/dev/null || true
    sudo /sbin/mkfs.f2fs -f "$NVME_DEV"
    sudo mount -t f2fs -o rw,noatime,nosuid,nodev,discard,fsync_mode=strict "$NVME_DEV" /data/media
    sudo mkdir -p /data/media/0
    sudo chown -R comma:comma /data/media/0
  else
    echo "    [!] WARNING: $NVME_DEV is '${NVME_FS:-unformatted}' (not f2fs)."
    echo "    Run 'scripts/setup/comma3-bootstrap.sh --reformat-nvme' to format to f2fs (WIPES ALL RLOGS)."
  fi
else
  echo "    NVMe is $NVME_FS (expected: f2fs)."
fi

# 4. openpilot boot hook (/data/continue.sh)
echo "--> Writing /data/continue.sh boot hook..."
cat > /data/continue.sh <<'EOF'
#!/usr/bin/bash
# F2FS tools auto-install (offline, OTA-resilient — vault Part 4)
if ! command -v fsck.f2fs >/dev/null 2>&1; then
  logger -t f2fs-persistence "F2FS tools missing (likely AGNOS OTA). Installing from /data/ cache..."
  DEBIAN_FRONTEND=noninteractive dpkg -i /data/f2fs-tools_*.deb >/dev/null 2>&1
fi

# F2FS failsafe mount for /data/media (NVMe). Vault Part 3.
if ! mountpoint -q /data/media; then
  sudo mount -t f2fs -o rw,noatime,nosuid,nodev,discard,fsync_mode=strict /dev/nvme0n1 /data/media || true
fi

# F2FS durability tuning: remount lfs-mode + noatime (Gemini 2026-05-19, vault Part 5).
sudo mount -o remount,noatime,mode=lfs /data/media

# F2FS durability stopgap: sync /data/media every 10s (loggerd has no explicit fsync,
# cold-shutdown without this loses 20+ minutes of in-flight log data).
( while true; do sync -f /data/media; sleep 10; done ) &

# Comma egress block. Vault Part 2.
/data/comma-block.sh

cd /data/openpilot
exec ./launch_openpilot.sh
EOF
chmod +x /data/continue.sh

# 5. Restore load-bearing StarPilot params
echo "--> Restoring required device params..."
export PYTHONPATH="/data/openpilot"
/usr/local/venv/bin/python3 <<'EOF'
import sys
sys.path.insert(0, "/data/openpilot")
from openpilot.common.params import Params
p = Params()

# Longitudinal / TSS-P Patch F tunings
p.put("AggressiveFollow", "0.5")
p.put("AggressiveFollowHigh", "0.4")
p.put("StopDistance", "2.0")
p.put_bool("AlphaLongitudinalEnabled", True)

# Personalities
p.put_bool("CustomPersonalities", True)
p.put("LongitudinalPersonality", "0")  # aggressive
p.put_bool("AggressivePersonalityProfile", True)

# Belt-and-suspenders telemetry disable (paired with iptables block)
p.put_bool("NoUploads", True)

print("    Params written.")
EOF

# 6. Execute the egress block immediately (no reboot required)
echo "--> Applying egress block to running iptables..."
/data/comma-block.sh

echo "==> Bootstrap complete. Device is hardened and f2fs persistence is primed."

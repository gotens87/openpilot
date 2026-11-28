"""Tailscale node restore. Connect work runs outside the HTTP request."""
from __future__ import annotations

import json
from pathlib import Path
import re
import subprocess
import tarfile
import threading
import time

ROOT = Path('/data/tailscale')
SYSTEM_UNIT = Path('/etc/systemd/system/tailscaled.service')
PACKAGES = 'https://pkgs.tailscale.com/stable/'
ARCH = 'arm64'
DEFAULT_HOSTNAME = 'tici-the-galaxy'
MESSAGE_MAX = 256
STATUS_TTL = 10.0


def unit_text(root):
  root = str(root)
  return f'''[Unit]
    Description=Tailscale node agent
    After=network.target

    [Service]
    ExecStart={root}/tailscaled \\
      --tun=userspace-networking \\
      --socks5-server=localhost:1055 \\
      --state={root}/state/tailscaled.state \\
      --socket={root}/tailscaled.sock \\
      --statedir={root}/state
    Restart=on-failure
    RestartSec=5

    [Install]
    WantedBy=multi-user.target
'''


def clip_message(text):
  if text is None:
    return None
  text = ' '.join(str(text).split())
  if not text:
    return None
  if len(text) > MESSAGE_MAX:
    return text[:MESSAGE_MAX]
  return text


class SubprocessRunner:
  def run(self, args, timeout=30):
    return subprocess.run(args, capture_output=True, text=True, timeout=timeout)


class TailscaleSettings:
  def __init__(self, root=None, system_unit=None, runner=None, sleep=time.sleep, hostname=DEFAULT_HOSTNAME,
               clock=time.monotonic):
    self.root = Path(root) if root is not None else ROOT
    self.system_unit = Path(system_unit) if system_unit is not None else SYSTEM_UNIT
    self.runner = runner if runner is not None else SubprocessRunner()
    self.sleep = sleep
    self.clock = clock
    self.default_hostname = hostname
    self.lock = threading.Lock()
    self._busy = False
    self._message = None
    self._thread = None
    self._parked = None
    self._status_cache = None
    self._status_cache_at = 0.0

  def snapshot(self, parked):
    with self.lock:
      busy = self._busy
      message = self._message
    body = {
      'state': 'not_installed',
      'busy': busy,
      'parked': bool(parked),
      'hostname': None,
      'ips': [],
      'auth_url': None,
      'version': None,
      'message': clip_message(message),
    }
    if not ((self.root / 'tailscale').is_file() and (self.root / 'tailscaled').is_file()):
      return body
    status = self._status_json(parked=bool(parked))
    if status is None:
      body['state'] = 'offline'
      return body
    body['hostname'] = (status.get('Self') or {}).get('HostName') or None
    body['ips'] = list(status.get('TailscaleIPs') or [])
    body['auth_url'] = status.get('AuthURL') or None
    body['version'] = status.get('Version') or None
    backend = status.get('BackendState')
    if backend == 'Running':
      body['state'] = 'online'
    elif backend in ('NeedsLogin', 'NoState'):
      body['state'] = 'needs_login'
    elif backend == 'Starting' and busy:
      body['state'] = 'starting'
    else:
      body['state'] = 'offline'
    return body

  def connect(self, parked):
    with self.lock:
      if not parked():
        raise PermissionError('Park before changing Tailscale.')
      if self._busy:
        raise FileExistsError('Tailscale is already working.')
      self._busy = True
      self._message = None
      self._parked = parked
    thread = threading.Thread(target=self._run, daemon=True)
    self._thread = thread
    try:
      thread.start()
    except Exception:
      with self.lock:
        self._busy = False
      raise
    return {'started': True}

  def _run(self):
    try:
      self._job()
    except Exception as error:
      extra = clip_message(error)
      with self.lock:
        existing = self._message
        if extra and existing:
          self._message = clip_message(f'{extra} {existing}')
        elif extra:
          self._message = extra
    finally:
      with self.lock:
        self._busy = False
        self._status_cache = None

  def _job(self):
    self._require_parked()
    self._ensure_binaries()
    self._ensure_local_unit()
    self._ensure_system_unit()
    self._wait_socket()
    status = self._status_json(fresh=True)
    if (status or {}).get('BackendState') == 'Running':
      return
    self._bring_up(self._prefs_hostname())

  def _require_parked(self):
    check = self._parked
    if check is None or not check():
      raise PermissionError('Park before changing Tailscale.')

  def _ensure_binaries(self):
    if (self.root / 'tailscale').is_file() and (self.root / 'tailscaled').is_file():
      return
    self.root.mkdir(parents=True, exist_ok=True)
    version = self._latest_version()
    tgz_path = self.root / 'tailscale.tgz'
    result = self.runner.run(['curl', '-fsSL', f'{PACKAGES}tailscale_{version}_{ARCH}.tgz', '-o', str(tgz_path)],
                             timeout=120)
    if result.returncode != 0:
      raise RuntimeError('Failed to download Tailscale archive.')
    prefix = f'tailscale_{version}_{ARCH}/'
    with tarfile.open(tgz_path) as archive:
      for name in ('tailscale', 'tailscaled'):
        member = archive.getmember(prefix + name)
        if not member.isreg():
          raise RuntimeError('Failed to download Tailscale archive.')
        reader = archive.extractfile(member)
        target = self.root / name
        target.write_bytes(reader.read())
        target.chmod(0o755)

  def _latest_version(self):
    result = self.runner.run(['curl', '-fsSL', PACKAGES], timeout=30)
    if result.returncode != 0:
      raise RuntimeError('Failed to download Tailscale archive.')
    found = re.findall(r'tailscale_(\d+\.\d+\.\d+)_' + ARCH, result.stdout)
    if not found:
      raise RuntimeError('Failed to download Tailscale archive.')
    return max(found, key=lambda item: tuple(int(part) for part in item.split('.')))

  def _ensure_local_unit(self):
    path = self.root / 'tailscaled.service'
    if path.is_file():
      return
    path.write_text(unit_text(self.root))

  def _ensure_system_unit(self):
    wanted = (self.root / 'tailscaled.service').read_text()
    try:
      current = self.system_unit.read_text()
    except OSError:
      current = None
    if current != wanted or not self._is_enabled():
      self._install_unit()
      return
    if not self._is_active():
      self._require_parked()
      self._sudo(['systemctl', 'restart', 'tailscaled'])

  def _install_unit(self):
    self._require_parked()
    error = None
    remount_error = None
    try:
      self._sudo(['mount', '-o', 'rw,remount', '/'])
      self._sudo(['cp', str(self.root / 'tailscaled.service'), str(self.system_unit)])
      self._sudo(['systemctl', 'daemon-reload'])
      self._sudo(['systemctl', 'enable', '--now', 'tailscaled'])
    except Exception as caught:
      error = caught
    finally:
      try:
        self._remount_ro()
      except Exception as caught:
        remount_error = caught
    if remount_error is not None:
      with self.lock:
        self._message = clip_message('Root filesystem stayed writable.')
    if error is not None:
      raise error

  def _remount_ro(self):
    last = None
    for _ in range(5):
      try:
        self._sudo(['mount', '-o', 'ro,remount', '/'], timeout=15)
        return
      except Exception as error:
        last = error
        self.sleep(0.4)
    raise last

  def _is_active(self):
    result = self.runner.run(['sudo', '-n', 'systemctl', 'is-active', 'tailscaled'], timeout=10)
    return result.returncode == 0 and result.stdout.strip() == 'active'

  def _is_enabled(self):
    result = self.runner.run(['sudo', '-n', 'systemctl', 'is-enabled', 'tailscaled'], timeout=10)
    return result.returncode == 0

  def _wait_socket(self):
    socket = self.root / 'tailscaled.sock'
    for _ in range(150):
      if socket.exists():
        return
      self.sleep(0.2)
    raise RuntimeError('Tailscale did not start.')

  def _bring_up(self, hostname):
    self._require_parked()
    status = self._status_json(fresh=True)
    if self._up_settled(status):
      return
    args = ['sudo', '-n', str(self.root / 'tailscale'), '--socket', str(self.root / 'tailscaled.sock'),
            'up', '--hostname', hostname, '--timeout', '60s']
    result = None
    timed_out = False
    try:
      result = self.runner.run(args, timeout=70)
    except subprocess.TimeoutExpired:
      timed_out = True
    status = self._status_json(fresh=True)
    if self._up_settled(status):
      return
    if (
      not timed_out
      and result is not None
      and result.returncode == 0
      and status
      and status.get('BackendState') in ('NeedsLogin', 'NoState')
    ):
      return
    detail = ''
    if result is not None:
      detail = (result.stderr or result.stdout or '').strip()
    raise RuntimeError(clip_message(detail) or 'Tailscale did not come up.')

  def _up_settled(self, status):
    if not status:
      return False
    if status.get('BackendState') == 'Running':
      return True
    if status.get('BackendState') in ('NeedsLogin', 'NoState') and status.get('AuthURL'):
      return True
    return False

  def _prefs_hostname(self):
    prefs = self._prefs_json()
    name = (prefs or {}).get('Hostname') or (prefs or {}).get('hostname') or ''
    if isinstance(name, str):
      name = name.strip()
    else:
      name = ''
    return name or self.default_hostname

  def _prefs_json(self):
    socket = self.root / 'tailscaled.sock'
    if not socket.exists():
      return None
    try:
      result = self.runner.run(
        [str(self.root / 'tailscale'), '--socket', str(socket), 'debug', 'prefs'],
        timeout=5)
      if result.returncode != 0:
        return None
      return json.loads(result.stdout)
    except (OSError, ValueError, subprocess.SubprocessError):
      return None

  def _status_json(self, fresh=False, parked=True):
    socket = self.root / 'tailscaled.sock'
    if not socket.exists():
      with self.lock:
        self._status_cache = None
      return None
    now = self.clock()
    if not fresh:
      with self.lock:
        cached = self._status_cache
        age = now - self._status_cache_at
      if cached is not None and age < STATUS_TTL:
        return cached
      if not parked:
        return None
    try:
      result = self.runner.run(
        [str(self.root / 'tailscale'), '--socket', str(socket), 'status', '--json'],
        timeout=5)
      if result.returncode != 0:
        parsed = None
      else:
        parsed = json.loads(result.stdout)
    except (OSError, ValueError, subprocess.SubprocessError):
      parsed = None
    with self.lock:
      self._status_cache = parsed
      self._status_cache_at = self.clock()
    return parsed

  def _sudo(self, args, timeout=30):
    result = self.runner.run(['sudo', '-n', *args], timeout=timeout)
    if result.returncode != 0:
      raise RuntimeError(clip_message(result.stderr.strip() or result.stdout.strip()) or 'Tailscale setup failed.')
    return result

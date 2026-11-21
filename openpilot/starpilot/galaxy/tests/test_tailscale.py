import json
import threading
import time
from pathlib import Path

import pytest

from openpilot.starpilot.galaxy.tailscale import MESSAGE_MAX, STATUS_TTL, TailscaleSettings


class Result:
  def __init__(self, returncode=0, stdout='', stderr=''):
    self.returncode = returncode
    self.stdout = stdout
    self.stderr = stderr


class FakeClock:
  def __init__(self):
    self.now = 0.0

  def __call__(self):
    return self.now

  def advance(self, seconds):
    self.now += seconds


class FakeRunner:
  def __init__(self):
    self.calls = []
    self.status = None
    self.prefs = {'Hostname': 'tici-the-galaxy'}
    self.active = True
    self.enabled = True
    self.fail = set()
    self.fail_times = {}
    self.stderr = ''
    self.hold = None

  def run(self, args, timeout=30):
    args = list(args)
    self.calls.append(args)
    if self.hold is not None and 'status' not in args and 'prefs' not in args:
      self.hold.wait()
    for token in list(self.fail_times):
      if token in args and self.fail_times[token] > 0:
        self.fail_times[token] -= 1
        return Result(1, '', self.stderr or 'failed')
    if self.fail.intersection(args):
      return Result(1, '', self.stderr or 'failed')
    if 'status' in args and '--json' in args:
      if self.status is None:
        return Result(1, '', '')
      return Result(0, json.dumps(self.status))
    if 'prefs' in args:
      return Result(0, json.dumps(self.prefs))
    if 'is-active' in args:
      return Result(0 if self.active else 3, 'active' if self.active else 'inactive')
    if 'is-enabled' in args:
      return Result(0 if self.enabled else 1, 'enabled' if self.enabled else 'disabled')
    return Result()


def installed(tmp_path, socket=True, clock=None):
  root = tmp_path / 'tailscale'
  root.mkdir()
  (root / 'tailscale').write_text('x')
  (root / 'tailscaled').write_text('x')
  (root / 'tailscaled.service').write_text('unit')
  if socket:
    (root / 'tailscaled.sock').write_text('')
  system_unit = tmp_path / 'tailscaled.service'
  system_unit.write_text('unit')
  runner = FakeRunner()
  runner.status = {
    'BackendState': 'Running',
    'AuthURL': '',
    'TailscaleIPs': ['100.64.0.1'],
    'Version': '1.104.1',
    'Self': {'HostName': 'tici-the-galaxy'},
  }
  kwargs = dict(root=root, system_unit=system_unit, runner=runner, sleep=lambda _: None)
  if clock is not None:
    kwargs['clock'] = clock
  settings = TailscaleSettings(**kwargs)
  return root, system_unit, runner, settings


def assert_safe_calls(runner):
  for call in runner.calls:
    assert '--reset' not in call
    assert 'logout' not in call
    assert 'down' not in call
    assert 'rm' not in call
    assert 'rmdir' not in call
    for arg in call:
      assert '--state' not in str(arg)
      assert 'tailscaled.state' not in str(arg)
    if 'status' in call or 'prefs' in call:
      assert call[:2] != ['sudo', '-n']
    if 'up' in call:
      assert call[:2] == ['sudo', '-n']


def wait(settings, runner=None):
  settings._thread.join(timeout=2)
  assert not settings._thread.is_alive()
  assert not settings.snapshot(True)['busy']
  if runner is not None:
    assert_safe_calls(runner)


def test_not_installed(tmp_path):
  settings = TailscaleSettings(root=tmp_path / 'tailscale', system_unit=tmp_path / 'unit', runner=FakeRunner())
  assert settings.snapshot(False) == {
    'state': 'not_installed',
    'busy': False,
    'parked': False,
    'hostname': None,
    'ips': [],
    'auth_url': None,
    'version': None,
    'message': None,
  }


def test_offline_without_socket(tmp_path):
  root, system_unit, runner, settings = installed(tmp_path, socket=False)
  assert settings.snapshot(True)['state'] == 'offline'
  assert settings.snapshot(True)['busy'] is False
  assert settings.snapshot(True)['parked'] is True


@pytest.mark.parametrize('backend,expected', [
  ('Running', 'online'),
  ('NeedsLogin', 'needs_login'),
  ('NoState', 'needs_login'),
  ('Starting', 'offline'),
  ('Stopped', 'offline'),
])
def test_backend_state(tmp_path, backend, expected):
  root, system_unit, runner, settings = installed(tmp_path)
  auth = 'https://login.tailscale.com/a/example' if expected == 'needs_login' else ''
  runner.status = {
    'BackendState': backend,
    'AuthURL': auth,
    'TailscaleIPs': ['100.64.0.1'] if backend == 'Running' else [],
    'Version': '1.104.1',
    'Self': {'HostName': 'tici-the-galaxy'},
  }
  body = settings.snapshot(True)
  assert body['state'] == expected
  assert body['busy'] is False
  assert body['parked'] is True
  assert body['hostname'] == 'tici-the-galaxy'
  assert body['version'] == '1.104.1'
  if expected == 'online':
    assert body['ips'] == ['100.64.0.1']
    assert body['auth_url'] is None
  if expected == 'needs_login':
    assert body['auth_url'] == 'https://login.tailscale.com/a/example'


def test_starting_while_busy(tmp_path):
  root, system_unit, runner, settings = installed(tmp_path)
  runner.status = {
    'BackendState': 'Starting',
    'AuthURL': '',
    'TailscaleIPs': [],
    'Version': '1.104.1',
    'Self': {'HostName': 'tici-the-galaxy'},
  }
  settings._busy = True
  body = settings.snapshot(True)
  assert body['state'] == 'starting'
  assert body['busy'] is True


def test_connect_skips_download_when_binaries_exist(tmp_path):
  root, system_unit, runner, settings = installed(tmp_path)
  assert settings.connect(lambda: True) == {'started': True}
  wait(settings, runner)
  assert all(call[0] != 'curl' for call in runner.calls)
  assert all('up' not in call for call in runner.calls)


def test_unit_restore_remounts_ro_when_a_step_fails(tmp_path):
  root, system_unit, runner, settings = installed(tmp_path)
  system_unit.write_text('stale')
  runner.fail.add('cp')
  settings.connect(lambda: True)
  wait(settings, runner)
  assert ['sudo', '-n', 'mount', '-o', 'rw,remount', '/'] in runner.calls
  assert ['sudo', '-n', 'mount', '-o', 'ro,remount', '/'] in runner.calls
  rw = runner.calls.index(['sudo', '-n', 'mount', '-o', 'rw,remount', '/'])
  cp = next(i for i, call in enumerate(runner.calls) if 'cp' in call)
  ro = runner.calls.index(['sudo', '-n', 'mount', '-o', 'ro,remount', '/'])
  assert rw < cp < ro


def test_successful_install_remounts_ro(tmp_path):
  root, system_unit, runner, settings = installed(tmp_path)
  system_unit.write_text('stale')
  settings.connect(lambda: True)
  wait(settings, runner)
  assert ['sudo', '-n', 'mount', '-o', 'rw,remount', '/'] in runner.calls
  assert ['sudo', '-n', 'cp', str(root / 'tailscaled.service'), str(system_unit)] in runner.calls
  assert ['sudo', '-n', 'systemctl', 'daemon-reload'] in runner.calls
  assert ['sudo', '-n', 'systemctl', 'enable', '--now', 'tailscaled'] in runner.calls
  assert ['sudo', '-n', 'mount', '-o', 'ro,remount', '/'] in runner.calls
  rw = runner.calls.index(['sudo', '-n', 'mount', '-o', 'rw,remount', '/'])
  ro = runner.calls.index(['sudo', '-n', 'mount', '-o', 'ro,remount', '/'])
  assert rw < ro


def test_matching_unit_not_enabled_installs(tmp_path):
  root, system_unit, runner, settings = installed(tmp_path)
  runner.enabled = False
  settings.connect(lambda: True)
  wait(settings, runner)
  assert ['sudo', '-n', 'systemctl', 'daemon-reload'] in runner.calls
  assert ['sudo', '-n', 'systemctl', 'enable', '--now', 'tailscaled'] in runner.calls
  assert ['sudo', '-n', 'mount', '-o', 'ro,remount', '/'] in runner.calls


def test_state_dir_never_written(tmp_path):
  root, system_unit, runner, settings = installed(tmp_path)
  system_unit.write_text('stale')
  state = root / 'state'
  state.mkdir()
  marker = state / 'tailscaled.state'
  marker.write_text('keep')
  before = marker.stat().st_mtime_ns
  settings.connect(lambda: True)
  wait(settings, runner)
  assert marker.read_text() == 'keep'
  assert marker.stat().st_mtime_ns == before
  assert [path.name for path in state.iterdir()] == ['tailscaled.state']
  for call in runner.calls:
    assert str(state) not in call
    assert all(str(state) not in str(arg) for arg in call)


def test_connect_busy_guard(tmp_path):
  root, system_unit, runner, settings = installed(tmp_path)
  gate = threading.Event()
  runner.hold = gate
  assert settings.connect(lambda: True) == {'started': True}
  assert settings.snapshot(True)['busy'] is True
  with pytest.raises(FileExistsError, match='already working'):
    settings.connect(lambda: True)
  gate.set()
  wait(settings, runner)
  assert settings.snapshot(True)['busy'] is False


def test_connect_requires_parked(tmp_path):
  root, system_unit, runner, settings = installed(tmp_path)
  with pytest.raises(PermissionError, match='Park before changing Tailscale'):
    settings.connect(lambda: False)
  assert runner.calls == []
  assert settings.snapshot(False)['busy'] is False


def test_parked_rechecked_before_remount(tmp_path):
  root, system_unit, runner, settings = installed(tmp_path)
  system_unit.write_text('stale')
  allow = {'ok': True}
  original = settings._install_unit
  def wrapped():
    allow['ok'] = False
    original()
  settings._install_unit = wrapped
  settings.connect(lambda: allow['ok'])
  wait(settings, runner)
  assert ['sudo', '-n', 'mount', '-o', 'rw,remount', '/'] not in runner.calls
  body = settings.snapshot(False)
  assert body['message'] is not None
  assert 'Park' in body['message']


def test_parked_rechecked_before_up(tmp_path):
  root, system_unit, runner, settings = installed(tmp_path)
  runner.status = {
    'BackendState': 'NeedsLogin',
    'AuthURL': '',
    'TailscaleIPs': [],
    'Version': '1.104.1',
    'Self': {'HostName': 'tici'},
  }
  allow = {'ok': True}
  original = settings._bring_up
  def wrapped(hostname):
    allow['ok'] = False
    original(hostname)
  settings._bring_up = wrapped
  settings.connect(lambda: allow['ok'])
  wait(settings, runner)
  assert all('up' not in call for call in runner.calls)
  body = settings.snapshot(True)
  assert body['message'] is not None
  assert 'Park' in body['message']


def test_parked_rechecked_before_restart(tmp_path):
  root, system_unit, runner, settings = installed(tmp_path)
  runner.active = False
  allow = {'ok': True}
  original = settings._is_active
  def wrapped():
    result = original()
    if not result:
      allow['ok'] = False
    return result
  settings._is_active = wrapped
  settings.connect(lambda: allow['ok'])
  wait(settings, runner)
  assert all('restart' not in call for call in runner.calls)
  body = settings.snapshot(True)
  assert body['message'] is not None
  assert 'Park' in body['message']


def test_needs_login_up_uses_prefs_hostname(tmp_path):
  root, system_unit, runner, settings = installed(tmp_path)
  runner.status = {
    'BackendState': 'NeedsLogin',
    'AuthURL': '',
    'TailscaleIPs': [],
    'Version': '1.104.1',
    'Self': {'HostName': 'tici'},
  }
  runner.prefs = {'Hostname': 'car-node'}
  settings.connect(lambda: True)
  wait(settings, runner)
  up = [call for call in runner.calls if 'up' in call]
  assert len(up) == 1
  assert up[0][:2] == ['sudo', '-n']
  assert '--hostname' in up[0]
  assert up[0][up[0].index('--hostname') + 1] == 'car-node'
  assert '--timeout' in up[0]
  assert 'tici' not in up[0]
  assert '--reset' not in up[0]
  assert 'logout' not in up[0]
  assert 'down' not in up[0]
  prefs = [call for call in runner.calls if 'prefs' in call]
  assert prefs


def test_failed_up_from_needs_login_sets_message(tmp_path):
  root, system_unit, runner, settings = installed(tmp_path)
  runner.status = {
    'BackendState': 'NeedsLogin',
    'AuthURL': '',
    'TailscaleIPs': [],
    'Version': '1.104.1',
    'Self': {'HostName': 'tici'},
  }
  runner.fail.add('up')
  runner.stderr = 'Access denied: checkprefs access denied'
  settings.connect(lambda: True)
  wait(settings, runner)
  body = settings.snapshot(True)
  assert body['message'] is not None
  assert 'denied' in body['message'].lower()
  up = [call for call in runner.calls if 'up' in call]
  assert len(up) == 1
  assert up[0][:2] == ['sudo', '-n']


def test_skips_up_when_auth_url_exists(tmp_path):
  root, system_unit, runner, settings = installed(tmp_path)
  runner.status = {
    'BackendState': 'NeedsLogin',
    'AuthURL': 'https://login.tailscale.com/a/example',
    'TailscaleIPs': [],
    'Version': '1.104.1',
    'Self': {'HostName': 'tici'},
  }
  settings.connect(lambda: True)
  wait(settings, runner)
  assert all('up' not in call for call in runner.calls)


def test_message_clipped_to_256(tmp_path):
  root, system_unit, runner, settings = installed(tmp_path)
  system_unit.write_text('stale')
  runner.fail.add('enable')
  runner.stderr = 'Created symlink ' + ('x' * 300)
  settings.connect(lambda: True)
  wait(settings, runner)
  body = settings.snapshot(True)
  assert body['message'] is not None
  assert len(body['message']) <= MESSAGE_MAX
  assert len(body['message']) == MESSAGE_MAX


def test_snapshot_does_not_sudo(tmp_path):
  root, system_unit, runner, settings = installed(tmp_path)
  settings.snapshot(True)
  settings.snapshot(True)
  status_calls = [call for call in runner.calls if 'status' in call]
  assert len(status_calls) == 1
  assert status_calls[0][:2] != ['sudo', '-n']
  assert all(call[:2] != ['sudo', '-n'] for call in runner.calls)


def test_snapshot_skips_refresh_while_unparked(tmp_path):
  root, system_unit, runner, settings = installed(tmp_path)
  settings.snapshot(True)
  n = len(runner.calls)
  settings.snapshot(False)
  assert len(runner.calls) == n


def test_unparked_does_not_probe(tmp_path):
  root, system_unit, runner, settings = installed(tmp_path)
  body = settings.snapshot(False)
  assert body['state'] == 'offline'
  assert all('status' not in call for call in runner.calls)


def test_snapshot_cache_expires(tmp_path):
  assert STATUS_TTL >= 5.0
  clock = FakeClock()
  root, system_unit, runner, settings = installed(tmp_path, clock=clock)
  settings.snapshot(True)
  def status_calls():
    return sum(1 for call in runner.calls if 'status' in call)
  assert status_calls() == 1
  clock.advance(STATUS_TTL - 0.01)
  settings.snapshot(True)
  assert status_calls() == 1
  clock.advance(0.02)
  settings.snapshot(True)
  assert status_calls() == 2


def test_unparked_expired_cache_is_not_served(tmp_path):
  clock = FakeClock()
  root, system_unit, runner, settings = installed(tmp_path, clock=clock)
  assert settings.snapshot(True)['state'] == 'online'
  clock.advance(STATUS_TTL + 0.1)
  body = settings.snapshot(False)
  assert body['state'] == 'offline'
  assert sum(1 for call in runner.calls if 'status' in call) == 1


def test_ro_remount_failure_still_online(tmp_path):
  root, system_unit, runner, settings = installed(tmp_path)
  system_unit.write_text('stale')
  runner.fail.add('ro,remount')
  settings.connect(lambda: True)
  wait(settings, runner)
  body = settings.snapshot(True)
  assert body['state'] == 'online'
  assert body['busy'] is False
  assert body['message'] is not None
  assert 'writable' in body['message']
  assert ['sudo', '-n', 'systemctl', 'enable', '--now', 'tailscaled'] in runner.calls


def test_ro_remount_retries(tmp_path):
  root, system_unit, runner, settings = installed(tmp_path)
  system_unit.write_text('stale')
  runner.fail_times['ro,remount'] = 3
  sleeps = []
  settings.sleep = lambda t: sleeps.append(t)
  settings.connect(lambda: True)
  wait(settings, runner)
  assert runner.calls.count(['sudo', '-n', 'mount', '-o', 'ro,remount', '/']) == 4
  assert sleeps == [0.4, 0.4, 0.4]
  body = settings.snapshot(True)
  assert body['state'] == 'online'
  assert body['message'] is None


def test_writable_warning_survives_later_error(tmp_path):
  root, system_unit, runner, settings = installed(tmp_path, socket=False)
  system_unit.write_text('stale')
  runner.fail.add('ro,remount')
  settings.connect(lambda: True)
  wait(settings, runner)
  body = settings.snapshot(True)
  assert body['message'] is not None
  assert 'writable' in body['message']
  assert 'did not start' in body['message']

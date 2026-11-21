import http.client
import json
import tempfile
import threading
import unittest
from pathlib import Path
from unittest.mock import Mock

from openpilot.starpilot.galaxy.access import GalaxyAccessOwner
from openpilot.starpilot.galaxy.server import make_server


class FakeTailscaleSettings:
  def __init__(self):
    self.busy = False
    self._parked = None
    self._thread = None
    self.job_error = None
    self.job_done = threading.Event()
    self.after_request = threading.Event()

  def snapshot(self, parked):
    return {
      'state': 'offline',
      'busy': self.busy,
      'parked': bool(parked),
      'hostname': None,
      'ips': [],
      'auth_url': None,
      'version': None,
      'message': None,
    }

  def connect(self, parked):
    if not parked():
      raise PermissionError('Park before changing Tailscale.')
    if self.busy:
      raise FileExistsError('Tailscale is already working.')
    self.busy = True
    self._parked = parked
    self.job_error = None
    self.job_done.clear()
    self.after_request.clear()
    self._thread = threading.Thread(target=self._run, daemon=True)
    self._thread.start()
    return {'started': True}

  def _run(self):
    try:
      self.after_request.wait(timeout=2)
      check = self._parked
      if check is None or not check():
        raise PermissionError('Park before changing Tailscale.')
    except Exception as error:
      self.job_error = error
    finally:
      self.busy = False
      self.job_done.set()


class TestTailscaleHttp(unittest.TestCase):
  def setUp(self):
    self.temp = tempfile.TemporaryDirectory()
    self.addCleanup(self.temp.cleanup)
    root = Path(self.temp.name)
    self.access = GalaxyAccessOwner(root / 'access')
    self.access.configure('password123', lambda: True)
    self.parked = True
    self.tailscale = FakeTailscaleSettings()
    aa = Mock()
    aa.enabled.return_value = False
    self.server = make_server(port=0, owner=self.access, tailscale=self.tailscale,
                              parked=lambda: self.parked, starport_setup=aa)
    self.worker = threading.Thread(target=self.server.serve_forever, kwargs={'poll_interval': 0.01}, daemon=True)
    self.worker.start()
    self.addCleanup(self.stop)

  def stop(self):
    self.tailscale.after_request.set()
    thread = self.tailscale._thread
    if thread is not None:
      thread.join(timeout=2)
    self.server.shutdown()
    self.worker.join(timeout=2)
    self.server.server_close()

  def request(self, path='/api/galaxy/tailscale', payload=None, cookie='', origin=None):
    connection = http.client.HTTPConnection('127.0.0.1', self.server.server_port, timeout=3)
    headers = {'Cookie': cookie}
    if payload is not None:
      headers.update({'Content-Type': 'application/json', 'Origin': origin or f'http://127.0.0.1:{self.server.server_port}'})
    try:
      connection.request('POST' if payload is not None else 'GET', path,
                         body=json.dumps(payload) if payload is not None else None, headers=headers)
      response = connection.getresponse()
      return response.status, json.loads(response.read()), dict(response.getheaders())
    finally:
      connection.close()

  def login(self):
    status, _, headers = self.request('/api/auth/login', {'password': 'password123'})
    self.assertEqual(status, 200)
    return headers['Set-Cookie'].split(';', 1)[0]

  def test_auth_park_busy_action_and_start(self):
    self.assertEqual(self.request()[0], 401)
    self.assertEqual(self.request(payload={'action': 'connect'})[0], 401)
    cookie = self.login()
    status, value, _ = self.request(cookie=cookie)
    self.assertEqual(status, 200)
    self.assertEqual(value['state'], 'offline')
    self.assertFalse(value['busy'])
    self.parked = False
    status, data, _ = self.request(payload={'action': 'connect'}, cookie=cookie)
    self.assertEqual(status, 409)
    self.assertEqual(data['error'], 'Park before changing Tailscale.')
    self.parked = True
    self.tailscale.busy = True
    status, data, _ = self.request(payload={'action': 'connect'}, cookie=cookie)
    self.assertEqual(status, 409)
    self.assertEqual(data['error'], 'Tailscale is already working.')
    self.tailscale.busy = False
    self.assertEqual(self.request(payload={'action': 'down'}, cookie=cookie)[0], 400)
    status, data, _ = self.request(payload={'action': 'connect'}, cookie=cookie)
    self.assertEqual(status, 202)
    self.assertEqual(data, {'started': True})

  def test_job_park_check_after_request_closes(self):
    cookie = self.login()
    status, data, _ = self.request(payload={'action': 'connect'}, cookie=cookie)
    self.assertEqual(status, 202)
    self.assertEqual(data, {'started': True})
    self.tailscale.after_request.set()
    self.assertTrue(self.tailscale.job_done.wait(timeout=2))
    self.assertIsNone(self.tailscale.job_error)

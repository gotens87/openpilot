from openpilot.selfdrive.ui.onroad import cameraview


class FakeFrame:
  def __init__(self, frame_id: int, idx: int):
    self.frame_id = frame_id
    self.idx = idx


def _camera_view():
  view = cameraview.CameraView.__new__(cameraview.CameraView)
  view._name = "camerad"
  view.frame = None
  view._last_frame_id = -1
  view._regressive_frame_count = 0
  view._texture_needs_update = False
  view._closed = True
  return view


def test_reused_egl_slot_cannot_move_camera_backwards(monkeypatch):
  monkeypatch.setattr(cameraview.cloudlog, "warning", lambda *_args, **_kwargs: None)
  view = _camera_view()

  displayed = FakeFrame(frame_id=10, idx=0)
  assert view._accept_frame(displayed, packet_frame_id=10)

  # camerad cycles back to this shared slot while it remains displayed.
  displayed.frame_id = 30
  view._observe_displayed_frame()

  delayed = FakeFrame(frame_id=20, idx=1)
  assert not view._accept_frame(delayed, packet_frame_id=20)
  assert view.frame is displayed
  assert view._last_frame_id == 30
  assert view._regressive_frame_count == 1


def test_newer_camera_frame_is_accepted():
  view = _camera_view()
  view._last_frame_id = 30
  newer = FakeFrame(frame_id=31, idx=2)

  assert view._accept_frame(newer, packet_frame_id=31)
  assert view.frame is newer
  assert view._last_frame_id == 31
  assert view._texture_needs_update

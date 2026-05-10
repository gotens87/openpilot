from openpilot.starpilot.common import starpilot_functions as spf


class FakeParams:
  def __init__(self, values=None):
    self.values = dict(values or {})

  def get(self, key):
    return self.values.get(key)

  def put(self, key, value):
    self.values[key] = value


def test_sync_konik_dongle_id_preserves_stock_id_before_switching(monkeypatch, tmp_path):
  monkeypatch.setattr(spf.Paths, "persist_root", staticmethod(lambda: str(tmp_path)))
  monkeypatch.setattr(spf, "use_konik_server", lambda: True)
  monkeypatch.setattr(spf, "register", lambda **kwargs: "konik-dongle")

  params = FakeParams({"DongleId": "stock-dongle"})

  spf.sync_konik_dongle_id(params)

  assert params.get("StockDongleId") == "stock-dongle"
  assert params.get("KonikDongleId") == "konik-dongle"
  assert params.get("DongleId") == "konik-dongle"


def test_sync_konik_dongle_id_restores_stock_id_from_persist(monkeypatch, tmp_path):
  persist_root = tmp_path / "persist"
  persisted_dongle_id_path = persist_root / "comma" / "dongle_id"
  persisted_dongle_id_path.parent.mkdir(parents=True, exist_ok=True)
  persisted_dongle_id_path.write_text("stock-dongle")

  monkeypatch.setattr(spf.Paths, "persist_root", staticmethod(lambda: str(persist_root)))
  monkeypatch.setattr(spf, "use_konik_server", lambda: False)

  params = FakeParams({
    "DongleId": "konik-dongle",
    "KonikDongleId": "konik-dongle",
  })

  spf.sync_konik_dongle_id(params)

  assert params.get("StockDongleId") == "stock-dongle"
  assert params.get("DongleId") == "stock-dongle"


def test_sync_konik_dongle_id_skips_missing_stock_backup(monkeypatch, tmp_path):
  monkeypatch.setattr(spf.Paths, "persist_root", staticmethod(lambda: str(tmp_path)))
  monkeypatch.setattr(spf, "use_konik_server", lambda: False)

  params = FakeParams({
    "DongleId": "konik-dongle",
    "KonikDongleId": "konik-dongle",
  })

  spf.sync_konik_dongle_id(params)

  assert params.get("DongleId") == "konik-dongle"
  assert params.get("StockDongleId") is None

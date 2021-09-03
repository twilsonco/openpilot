import unittest
from unittest import mock
from datetime import datetime as dt, timezone, timedelta
from selfdrive.config import Conversions as CV
from selfdrive.mapd.lib.WayRelation import is_osm_time_condition_active, speed_limit_for_osm_tag_limit_string, \
  conditional_speed_limit_for_osm_tag_limit_string


class TestWayRelation(unittest.TestCase):
  def test_speed_limit_for_osm_tag_limit_string(self):
    values = [
      None,  # Invalid
      "1000",  # Invalid
      "60 kph",  # Invalid
      "100",
      "30 mph",
      "DE:zone:40",
      "DE:zone:50 mph",
      "AR:urban",
      "CZ:pedestrian_zone",
      "DK:urban",
      "DK:rural",
      "DK:motorway",
      "DE:living_street",
      "DE:residential",
      "DE:urban",
      "DE:rural",
      "DE:trunk",  # No limit
      "DE:motorway",  # No limit
      "GB:nsl_restricted",
      "GB:nsl_single",
      "GB:nsl_dual",
      "GB:motorway",
      "GB:invalid",  # Invalid
    ]

    expected = [
      0.,
      0.,
      0.,
      100. * CV.KPH_TO_MS,
      30. * CV.MPH_TO_MS,
      40. * CV.KPH_TO_MS,
      50. * CV.MPH_TO_MS,
      40. * CV.KPH_TO_MS,
      20. * CV.KPH_TO_MS,
      50. * CV.KPH_TO_MS,
      80. * CV.KPH_TO_MS,
      130. * CV.KPH_TO_MS,
      7. * CV.KPH_TO_MS,
      30. * CV.KPH_TO_MS,
      50. * CV.KPH_TO_MS,
      100. * CV.KPH_TO_MS,
      0.,
      0.,
      30. * CV.MPH_TO_MS,
      60. * CV.MPH_TO_MS,
      70. * CV.MPH_TO_MS,
      70. * CV.MPH_TO_MS,
      0.,
    ]

    result = [speed_limit_for_osm_tag_limit_string(sls) for sls in values]

    self.assertEqual(result, expected)

  @mock.patch('selfdrive.mapd.lib.WayRelation.dt')
  def test_is_osm_time_condition_active(self, mock_dt):
    tz = timezone(timedelta(hours=1), 'berlin')
    wed_10_10_am = dt(2021, 9, 1, 10, 10, 0)
    mock_dt.now.return_value = wed_10_10_am
    mock_dt.tzinfo = tz
    mock_dt.combine = dt.combine
    mock_dt.strptime = dt.strptime

    values = [
      "WE",  # Invalid
      "We",
      "Mo",
      "Fr",
      "Tu-Th",
      "10:00",  # Invalid
      "10:00-10:30",
      "We 10:00-10:30",
      "SU 10:00-10:30",  # Valid, SU string not considered a day string.
      "Sa 10:00-10:30",
      "Tu-Th 10:00-10:30",
    ]

    expected = [
      False,  # Invalid
      True,
      False,
      False,
      True,
      False,  # Invalid
      True,
      True,
      True,
      False,
      True,
    ]

    result = [is_osm_time_condition_active(cs) for cs in values]

    self.assertEqual(result, expected)

  @mock.patch('selfdrive.mapd.lib.WayRelation.dt')
  def test_conditional_speed_limit_for_osm_tag_limit_string(self, mock_dt):
    tz = timezone(timedelta(hours=1), 'berlin')
    wed_10_10_am = dt(2021, 9, 1, 10, 10, 0)
    mock_dt.now.return_value = wed_10_10_am
    mock_dt.tzinfo = tz
    mock_dt.combine = dt.combine
    mock_dt.strptime = dt.strptime

    values = [
      None,  # Invalid
      "Hola",  # Invalid
      "100 @ (WE)",  # Invalid
      "x @ (We)",  # Invalid
      "100 @ (We)",
      "100 @ (Mo)",
      "100 @ (Fr)",
      "100 @ (Tu-Th)",
      "100 @ (10:00)",  # Invalid
      "100 @ (10:00-10:30)",
      "100 @ (We 10:00-10:30)",
      "100 @ (SU 10:00-10:30)",  # Valid, SU string not considered a day string.
      "100 @ (Sa 10:00-10:30)",
      "100 @ (Tu-Th 10:00-10:30)",
      "100 @ (Mo-Th;Su)",
      "100 @ (Mo Th;Fr-Sa)",
      "100 @ (Fr-Su;Mo-Tu)",
      "100 @ (10:00-10:30;15:00-16:00)",
      "100 @ (We;Mo-Tu)",
      "100 @ (We 10:00-10:30;Th 15:00-16:00)",
      "100 @ (Tu 10:00-10:30;Th 15:00-16:00)",
    ]

    _100 = 100. * CV.KPH_TO_MS

    expected = [
      0.,  # Invalid
      0.,  # Invalid
      0.,  # Invalid
      0.,  # Invalid
      _100,
      0.,
      0.,
      _100,
      0.,  # Invalid
      _100,
      _100,
      _100,
      0.,
      _100,
      _100,
      _100,
      0.,
      _100,
      _100,
      _100,
      0.
    ]

    result = [conditional_speed_limit_for_osm_tag_limit_string(ls) for ls in values]

    self.assertEqual(result, expected)

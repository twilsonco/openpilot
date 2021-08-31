import unittest
from selfdrive.config import Conversions as CV
from .WayRelation import speed_limit_for_osm_tag_limit_string


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

    result = list(map(lambda sls: speed_limit_for_osm_tag_limit_string(sls), values))

    self.assertEqual(result, expected)

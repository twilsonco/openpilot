import unittest
from decimal import Decimal
from math import pi
import numpy as np


class TestMapsdGeoLibrary(unittest.TestCase):
  # 0. coord to rad point tuple conversion
  def test_coord_to_rad(self):
    pass

  # Helpers
  def _assertAlmostEqualList(self, a, b, places=0):
      for idx, el_a in enumerate(a):
        self.assertAlmostEqual(el_a, b[idx], places)

  def _assertAlmostEqualListOfTuples(self, a, b, places=0):
    for idx, el_a in enumerate(a):
      for idy, el_el_a in enumerate(el_a):
        self.assertAlmostEqual(el_el_a, b[idx][idy], places)

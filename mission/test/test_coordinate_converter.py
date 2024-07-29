import unittest
from mission.utils.coordinate_converter import CoordinateConverter

class TestCoordinateConverter(unittest.TestCase):

    def setUp(self):
        self.converter = CoordinateConverter(-5.038051, -37.789214, 416)

    def test_wgs84_to_ecef(self):
        X, Y, Z = self.converter.wgs84_to_ecef(-5.038051, -37.789214, 416)
        self.assertAlmostEqual(X, -1603997.596, places=3)
        self.assertAlmostEqual(Y, -5910072.084, places=3)
        self.assertAlmostEqual(Z, 1021047.920, places=3)

    def test_ecef_to_enu(self):
        X, Y, Z = self.converter.wgs84_to_ecef(-5.038051, -37.789214, 416)
        x, y, z = self.converter.ecef_to_enu(X, Y, Z)
        self.assertAlmostEqual(x, 0.0, places=1)
        self.assertAlmostEqual(y, 0.0, places=1)
        self.assertAlmostEqual(z, 0.0, places=1)

    def test_convert(self):
        x, y, z = self.converter.convert(-5.038051, -37.789214, 416)
        self.assertAlmostEqual(x, 0.0, places=1)
        self.assertAlmostEqual(y, 0.0, places=1)
        self.assertAlmostEqual(z, 0.0, places=1)

    def test_invalid_latitude(self):
        with self.assertRaises(ValueError):
            self.converter.lat0 = -10

    def test_invalid_longitude(self):
        with self.assertRaises(ValueError):
            self.converter.lon0 = -50

    def test_invalid_altitude(self):
        with self.assertRaises(ValueError):
            self.converter.alt0 = -10

if __name__ == '__main__':
    unittest.main()

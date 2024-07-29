import numpy as np

class CoordinateConverter:
    # WGS-84 ellipsoid parameters
    a = 6378137.0          # semi-major axis in meters
    f = 1 / 298.257223563  # flattening
    e2 = 2 * f - f ** 2    # eccentricity squared

    def __init__(self, lat0: float, lon0: float, alt0: float) -> None:
        self._lat0 = None
        self._lon0 = None
        self._alt0 = None
        
        self.lat0 = lat0
        self.lon0 = lon0
        self.alt0 = alt0
        
        self.X0, self.Y0, self.Z0 = self.wgs84_to_ecef(lat0, lon0, alt0)
    
    @property
    def lat0(self) -> float:
        return self._lat0
    
    @property
    def lon0(self) -> float:
        return self._lon0
    
    @property
    def alt0(self) -> float:
        return self._alt0

    @lat0.setter
    def lat0(self, value: float) -> None:
        if not (-7 <= value <= -2):
            raise ValueError("Latitude ist too far from the origin point of the simulation.")
        self._lat0 = value

    @lon0.setter
    def lon0(self, value: float) -> None:
        if not (-40 <= value <= -35):
            raise ValueError("Longitude ist too far from the origin point of the simulation.")
        self._lon0 = value
    
    @alt0.setter
    def alt0(self, value: float) -> None:
        if value < 0:
            raise ValueError("Height can't be negative.")
        self._alt0 = value  # No specific range for altitude
    
    def wgs84_to_ecef(self, lat: float, lon: float, alt: float) -> tuple[float, float, float]:
        lat = np.deg2rad(lat)
        lon = np.deg2rad(lon)

        N = self.a / np.sqrt(1 - self.e2 * np.sin(lat)**2)

        X = (N + alt) * np.cos(lat) * np.cos(lon)
        Y = (N + alt) * np.cos(lat) * np.sin(lon)
        Z = (N * (1 - self.e2) + alt) * np.sin(lat)

        return X, Y, Z
    
    def ecef_to_enu(self, X: float, Y: float, Z: float) -> tuple[float, float, float]:
        # Calculate the difference in coordinates
        dX = X - self.X0
        dY = Y - self.Y0
        dZ = Z - self.Z0

        # Convert reference point lat/lon to radians
        lat0 = np.deg2rad(self.lat0)
        lon0 = np.deg2rad(self.lon0)

        # ECEF to ENU conversion matrix
        R = np.array([
            [-np.sin(lon0),                 np.cos(lon0),                  0],
            [-np.sin(lat0)*np.cos(lon0), -np.sin(lat0)*np.sin(lon0), np.cos(lat0)],
            [ np.cos(lat0)*np.cos(lon0),  np.cos(lat0)*np.sin(lon0), np.sin(lat0)]
        ])

        # Apply the rotation matrix
        enu = R @ np.array([dX, dY, dZ])

        return tuple(enu)
    
    def convert(self, lat: float, lon: float, alt: float) -> tuple[float, float, float]:
        X, Y, Z = self.wgs84_to_ecef(lat, lon, alt)
        x, y, z = self.ecef_to_enu(X, Y, Z)
        return x, y, z
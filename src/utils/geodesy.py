import numpy as np

# Pure NumPy WGS84 Geodesy implementation (zero C-library dependencies)
class ENUConverter:
    def __init__(self, lat0: float, lon0: float, alt0: float = 0.0):
        self.lat0 = float(lat0)
        self.lon0 = float(lon0)
        self.alt0 = float(alt0)
        
        # WGS84 constants
        self.a = 6378137.0
        self.f = 1.0 / 298.257223563
        self.e2 = self.f * (2.0 - self.f)
        self.b = self.a * (1.0 - self.f)
        self.ep2 = (self.a**2 - self.b**2) / (self.b**2)
        
        # Reference point in ECEF
        self.x0, self.y0, self.z0 = self._lla_to_ecef(self.lat0, self.lon0, self.alt0)
        
        lat0_rad = np.radians(self.lat0)
        lon0_rad = np.radians(self.lon0)
        
        slat = np.sin(lat0_rad)
        clat = np.cos(lat0_rad)
        slon = np.sin(lon0_rad)
        clon = np.cos(lon0_rad)
        
        self.rot_ecef_to_enu = np.array([
            [-slon, clon, 0.0],
            [-slat * clon, -slat * slon, clat],
            [clat * clon, clat * slon, slat]
        ])

    def _lla_to_ecef(self, lat: float, lon: float, alt: float):
        phi = np.radians(lat)
        lam = np.radians(lon)
        s_phi = np.sin(phi)
        c_phi = np.cos(phi)
        s_lam = np.sin(lam)
        c_lam = np.cos(lam)
        
        N = self.a / np.sqrt(1.0 - self.e2 * s_phi**2)
        x = (N + alt) * c_phi * c_lam
        y = (N + alt) * c_phi * s_lam
        z = (N * (1.0 - self.e2) + alt) * s_phi
        return x, y, z

    def _ecef_to_lla(self, x: float, y: float, z: float):
        p = np.sqrt(x**2 + y**2)
        theta = np.arctan2(z * self.a, p * self.b)
        
        phi = np.arctan2(
            z + self.ep2 * self.b * (np.sin(theta)**3),
            p - self.e2 * self.a * (np.cos(theta)**3)
        )
        lam = np.arctan2(y, x)
        
        s_phi = np.sin(phi)
        N = self.a / np.sqrt(1.0 - self.e2 * s_phi**2)
        alt = (p / np.cos(phi)) - N
        
        return np.degrees(phi), np.degrees(lam), alt

    def lla_to_enu(self, lat: float, lon: float, alt: float = 0.0) -> np.ndarray:
        x, y, z = self._lla_to_ecef(lat, lon, alt)
        diff = np.array([x - self.x0, y - self.y0, z - self.z0])
        enu = self.rot_ecef_to_enu @ diff
        return enu
        
    def enu_to_lla(self, e: float, n: float, u: float = 0.0):
        enu = np.array([e, n, u])
        diff = self.rot_ecef_to_enu.T @ enu
        x = diff[0] + self.x0
        y = diff[1] + self.y0
        z = diff[2] + self.z0
        return self._ecef_to_lla(x, y, z)

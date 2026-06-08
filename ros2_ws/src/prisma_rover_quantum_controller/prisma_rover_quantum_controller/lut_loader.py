import os
import csv
import numpy as np

# Try optional SciPy for sparse grid interpolation
try:
    from scipy.interpolate import griddata
    _HAS_SCIPY = True
except ImportError:
    griddata = None
    _HAS_SCIPY = False

class LUT2D:
    """
    Lookup table 2D on (x, y) -> value.
    Supports bilinear interpolation (if perfect grid or gridified) and nearest neighbor.
    """
    def __init__(self, csv_path: str, columns: list, logger=None, nx=100, ny=100, method='linear'):
        self.csv_path = csv_path
        self.columns = columns
        self.logger = logger
        self.nx = nx
        self.ny = ny
        self.method = method

        self.lut_data = self._load_and_process()

    def _log_info(self, msg):
        if self.logger:
            self.logger.info(msg)
        else:
            print(f"[INFO] {msg}")

    def _log_warn(self, msg):
        if self.logger:
            self.logger.warn(msg)
        else:
            print(f"[WARN] {msg}")

    def _load_and_process(self):
        if not os.path.isfile(self.csv_path):
            self._log_warn(f"⚠️ LUT file not found: {self.csv_path}")
            return None

        rows = []
        with open(self.csv_path, newline='') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                try:
                    vals = [float(row[c]) for c in self.columns]
                    rows.append(vals)
                except (ValueError, KeyError):
                    continue

        data = np.array(rows, dtype=float)  # shape (N, 3) -> [x, y, z]
        if data.shape[0] < 3:
            self._log_warn(f"⚠️ LUT {os.path.basename(self.csv_path)} has too few points -> nearest neighbor only")
            return {"type": "points", "data": data}

        # Check if it forms a complete cartesian grid
        x_unique = np.unique(data[:, 0])
        y_unique = np.unique(data[:, 1])
        expected = len(x_unique) * len(y_unique)
        is_perfect_grid = (expected == data.shape[0])

        if is_perfect_grid:
            Z = np.zeros((len(x_unique), len(y_unique)), dtype=float)
            for i, x in enumerate(x_unique):
                for j, y in enumerate(y_unique):
                    mask = (np.isclose(data[:, 0], x)) & (np.isclose(data[:, 1], y))
                    vals = data[mask, 2]
                    Z[i, j] = np.mean(vals) if vals.size > 0 else 0.0

            self._log_info(
                f"✅ LUT '{os.path.basename(self.csv_path)}' loaded as perfect grid ({len(x_unique)}x{len(y_unique)} points)"
            )
            return {"x": x_unique, "y": y_unique, "z": Z, "type": "grid"}

        # Sparse points: regular grid interpolation
        nx = max(2, int(self.nx))
        ny = max(2, int(self.ny))
        x = data[:, 0]
        y = data[:, 1]
        z = data[:, 2]

        X = np.linspace(np.min(x), np.max(x), nx)
        Y = np.linspace(np.min(y), np.max(y), ny)
        XX, YY = np.meshgrid(X, Y, indexing='ij')

        if _HAS_SCIPY:
            interp_method = self.method if self.method in ("linear", "cubic") else "linear"
            ZZ_primary = griddata(points=np.c_[x, y], values=z, xi=(XX, YY), method=interp_method)
            ZZ_nearest = griddata(points=np.c_[x, y], values=z, xi=(XX, YY), method="nearest")
            Z = np.where(np.isnan(ZZ_primary), ZZ_nearest, ZZ_primary)

            self._log_info(
                f"✅ LUT '{os.path.basename(self.csv_path)}' gridified with SciPy ({interp_method}) -> ({nx}x{ny} nodes)"
            )
            return {"x": X, "y": Y, "z": Z, "type": "grid"}
        else:
            self._log_warn("⚠️ SciPy not available: fallback to regular grid using nearest neighbor search.")
            Z = np.empty((nx, ny), dtype=float)
            pts = np.c_[x, y]
            for i in range(nx):
                for j in range(ny):
                    dx = pts[:, 0] - XX[i, j]
                    dy = pts[:, 1] - YY[i, j]
                    k = np.argmin(dx * dx + dy * dy)
                    Z[i, j] = z[k]

            self._log_info(
                f"✅ LUT '{os.path.basename(self.csv_path)}' gridified (fallback nearest) -> ({nx}x{ny} nodes)"
            )
            return {"x": X, "y": Y, "z": Z, "type": "grid"}

    def query(self, x: float, y: float) -> float:
        """Query value from LUT using bilinear interpolation (if grid) or nearest neighbor (if points)."""
        if self.lut_data is None:
            return 0.0

        if self.lut_data["type"] == "points":
            data = self.lut_data["data"]
            diffs = (data[:, 0] - x) ** 2 + (data[:, 1] - y) ** 2
            return float(data[np.argmin(diffs), 2])

        X, Y, Z = self.lut_data["x"], self.lut_data["y"], self.lut_data["z"]
        x = np.clip(x, X[0], X[-1])
        y = np.clip(y, Y[0], Y[-1])

        i = np.searchsorted(X, x) - 1
        j = np.searchsorted(Y, y) - 1
        i = np.clip(i, 0, len(X) - 2)
        j = np.clip(j, 0, len(Y) - 2)

        x1, x2 = X[i], X[i + 1]
        y1, y2 = Y[j], Y[j + 1]
        Q11, Q12 = Z[i, j], Z[i, j + 1]
        Q21, Q22 = Z[i + 1, j], Z[i + 1, j + 1]

        if (x2 - x1) == 0 or (y2 - y1) == 0:
            return float(Q11)

        val = (
            Q11 * (x2 - x) * (y2 - y)
            + Q21 * (x - x1) * (y2 - y)
            + Q12 * (x2 - x) * (y - y1)
            + Q22 * (x - x1) * (y - y1)
        ) / ((x2 - x1) * (y2 - y1))
        return float(val)

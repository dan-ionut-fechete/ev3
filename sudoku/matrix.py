import logging
import math
import ast

class SudokuMatrix(object):
    """Build monochrome matrix of the sudoku board.
        You can add new scans to the matrix and then you can normalize the
        results. Each scan is an arc of measurements.
    """

    # The radius of the circle covered by the scanning device.
    k_scanner_radius_mm = 51
    # A map from coordinates to a list of measurements for that point in space.
    measurements = {}

    def add_scan(self, cx, cy, degrees, measures):
        """cx, cy center of the measuring device
            degrees the size of the arc measured
            measures the samples gathered while measuring (we assume these are
                uniformly distributed along the movement arc).
        """
        if len(measures) < 1:
            logging.exception("No measurements provided")
        for i in range(0, len(measures)):
            deg_moved = degrees * i / len(measures)
            angle = math.radians(degrees / 2 - deg_moved)
            # Try to reduce the angle somehow the estimate for the angle get's too big
            # as we approach the limits
            angle *= 0.76
            a = self.k_scanner_radius_mm * math.cos(angle)
            b = self.k_scanner_radius_mm * math.sin(angle)
            px = cx - b
            py = cy + a        
            p = (int(math.floor(px)), int(math.floor(py)))
            if p not in self.measurements:
                self.measurements[p] = []
            self.measurements[p].append(measures[i])
        print('Scan added')

    def to_str(self):
        return str(self.measurements)
        
    def dump_to_file(self, filename):
        with open(filename, 'w') as f:
            f.write(str(self.measurements))
            f.close()

    def load_from_file(self, filename):
        fstr = ""
        try:
            with open(filename, 'r') as f:
                fstr = f.read()
                f.close()
            self.measurements = ast.literal_eval(fstr)
        except Exception as e:
            print('Unable to load file:')
            print(str(e))

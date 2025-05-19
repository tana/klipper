import math, logging
import mathutil
from . import probe

def _deg_to_rad(deg):
    return deg * math.pi / 180

def _bed_height_at(x, y, c, tilt, pivot_pos, rot_offset):
    rot_offset_quat_x = mathutil.quat_angle_axis(_deg_to_rad(rot_offset[0]), [1, 0, 0])
    rot_offset_quat_y = mathutil.quat_angle_axis(_deg_to_rad(rot_offset[1]), [0, 1, 0])
    rot_offset_quat_z = mathutil.quat_angle_axis(_deg_to_rad(rot_offset[2]), [0, 0, 1])
    rot_offset_quat = mathutil.quat_mul(mathutil.quat_mul(rot_offset_quat_x,rot_offset_quat_y), rot_offset_quat_z)

    c_quat = mathutil.quat_angle_axis(_deg_to_rad(c), [0, 0, 1])
    tilt_quat = mathutil.quat_angle_axis(_deg_to_rad(tilt), [1, 0, 0])
    rot = mathutil.quat_mul(rot_offset_quat, mathutil.quat_mul(c_quat, tilt_quat))

    x0, y0, z0 = pivot_pos
    nx, ny, nz = mathutil.quat_apply(rot, [0, 0, 1])
    return z0 - ((x - x0) * nx + (y - y0) * ny) / nz


class DancingBedCalibrate:
    def __init__(self, config):
        self.printer = config.get_printer()

        self.points = config.getlists('points', seps=(',', '\n'), count=2, parser=float)
        self.angles = config.getfloatlist('angles')
        self.horizontal_move_z = config.getfloat('horizontal_move_z', 5.)
        self.speed = config.getfloat('speed', 50., above=0.)

        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command('DANCING_BED_CALIBRATE', self.cmd_DANCING_BED_CALIBRATE)

    def cmd_DANCING_BED_CALIBRATE(self, gcmd):
        toolhead = self.printer.lookup_object('toolhead')
        probe_obj = self.printer.lookup_object('probe')
        lift_speed = probe_obj.get_probe_params(gcmd)['lift_speed']

        for c in self.angles:
            for x, y in self.points:
                # Raise toolhead
                toolhead.manual_move((None, None, self.horizontal_move_z, None, None, None), lift_speed)
                # Move and change angle
                toolhead.manual_move((x, y, None, None, None, c, None), self.speed)
                # Do probing
                probe.run_single_probe(probe_obj, gcmd)

    def _fit_params(self, probe_results):
        logging.info(f'results = {self.probe_results}')
        def error_func(params):
            for angle, positions in probe_results:
                for pos in positions:
                    pass


def load_config(config):
    return DancingBedCalibrate(config)

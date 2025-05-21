import math, logging
import mathutil
import kinematics.dancing_bed as dancing_bed
from . import probe

def _deg_to_rad(deg):
    return deg * math.pi / 180

def _bed_height_at(x, y, c, tilt, pivot, c_offset):
    c_quat = mathutil.quat_angle_axis(_deg_to_rad(c + c_offset), [0, 0, 1])
    tilt_quat = mathutil.quat_angle_axis(_deg_to_rad(tilt), [1, 0, 0])
    rot = mathutil.quat_mul(c_quat, tilt_quat)

    x0, y0, z0 = pivot
    nx, ny, nz = mathutil.quat_apply(rot, [0, 0, 1])
    return z0 - ((x - x0) * nx + (y - y0) * ny) / nz


class DancingBedCalibrate:
    def __init__(self, config):
        self.printer = config.get_printer()

        self.points = config.getlists('points', seps=(',', '\n'), count=2, parser=float)
        self.angles = config.getfloatlist('angles')
        self.horizontal_move_z = config.getfloat('horizontal_move_z', 5.)
        self.speed = config.getfloat('speed', 50., above=0.)
        self.stabilize_time = config.getfloat('stabilize_time', 0.5, above=0.)

        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command('DANCING_BED_CALIBRATE', self.cmd_DANCING_BED_CALIBRATE)

    def cmd_DANCING_BED_CALIBRATE(self, gcmd):
        toolhead = self.printer.lookup_object('toolhead')
        probe_obj = self.printer.lookup_object('probe')
        lift_speed = probe_obj.get_probe_params(gcmd)['lift_speed']
        kin = toolhead.get_kinematics()
        kin_calib = kin.get_calibration()

        # Disable calibration params during probing
        kin.set_calibration(dancing_bed.DancingBedCalibration(0., [0., 0., 0.], 0.))

        probe_results = []
        for c in self.angles:
            for x, y in self.points:
                # Raise toolhead
                toolhead.manual_move((None, None, self.horizontal_move_z, None, None, None), lift_speed)
                # Move and change angle
                toolhead.manual_move((x, y, None, None, None, c, None), self.speed)
                # Wait until the sensor stabilize
                toolhead.dwell(self.stabilize_time)
                # Do probing
                z = probe.run_single_probe(probe_obj, gcmd)[2]
                probe_results.append((x, y, c, z))

        logging.info("Probing results: %s", probe_results)

        def error_func(params):
            error_sq_sum = 0
            for x, y, c, z in probe_results:
                error = z - _bed_height_at(
                    x, y, c, kin_calib.tilt,
                    [params['x0'], params['y0'], params['z0']],
                    params['c_offset'],
                )
                error_sq_sum += error * error
            return error_sq_sum

        # Optimize using coordinate descent
        opt_result = mathutil.background_coordinate_descent(
            self.printer,
            ['x0', 'y0', 'z0', 'c_offset'],
            {'x0': kin_calib.pivot[0], 'y0': kin_calib.pivot[1], 'z0': kin_calib.pivot[2], 'c_offset': kin_calib.c_offset},
            error_func,
        )
        self.gcode.respond_info(f"Fitted parameters: {opt_result}")

        kin_calib.pivot = [opt_result['x0'], opt_result['x1'], opt_result['x2']]
        kin_calib.c_offset = opt_result['c_offset']
        self.kin.set_calibration(kin_calib)

        self.gcode.respond_info("Kinematics parameters updated")


def load_config(config):
    return DancingBedCalibrate(config)

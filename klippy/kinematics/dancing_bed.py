# Code for handling the kinematics of cartesian robots with rotating tilted bed
#
# Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math
import logging
import stepper

class DancingBedKinematics:
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()

        # Parameters
        self.tilt = config.getfloat('tilt')
        self.pivot = config.getfloatlist('pivot', count=3)
        self.c_offset = config.getfloat('c_offset')

        # Setup axis rails
        self.rails = [stepper.LookupMultiRail(config.getsection('stepper_' + n))
                      for n in 'xyzc']
        for rail, axis in zip(self.rails, 'xyzc'):
            rail.setup_itersolve(
                'dancing_bed_stepper_alloc',
                axis.encode(),
                self.tilt,
                self.pivot[0], self.pivot[1], self.pivot[2],
                self.c_offset,
            )

        ranges = [r.get_range() for r in self.rails]
        axes_min = self._calc_forward([r[0] for r in ranges])
        axes_max = self._calc_forward([r[1] for r in ranges])
        self.axes_min = toolhead.Coord(*axes_min, e=0.)
        self.axes_max = toolhead.Coord(*axes_max, e=0.)

        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)

        # Setup boundary checks
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat('max_z_velocity', max_velocity,
                                              above=0., maxval=max_velocity)
        self.max_z_accel = config.getfloat('max_z_accel', max_accel,
                                           above=0., maxval=max_accel)
        # C axis does not require homing
        self.limits = [
            (1.0, -1.0),
            (1.0, -1.0),
            (1.0, -1.0),
            (ranges[3][0], ranges[3][1])
        ]
        self.max_speed_x = max_velocity
        self.max_speed_y = max_velocity
        self.max_speed_z = self.max_z_velocity
        self.max_speed_a = 0.0
        self.max_speed_b = 0.0
        self.max_speed_c = config.getfloat('max_angular_velocity')

    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]

    def calc_position(self, stepper_positions):
        return self._calc_forward([stepper_positions[rail.get_name()] for rail in self.rails])

    def _calc_forward(self, stepper_pos):
        stepper_x, stepper_y, stepper_z, stepper_c = stepper_pos
        x0, y0, z0 = self.pivot
        x = stepper_x - x0
        y = stepper_y - y0
        z = stepper_z - z0
        tilt_rad = self.tilt * math.pi / 180
        stepper_c_rad = stepper_c * math.pi / 180
        # Tilt around X axis
        tilt_y = math.cos(tilt_rad) * y + math.sin(tilt_rad) * z
        tilt_z = -math.sin(tilt_rad) * y + math.cos(tilt_rad) * z
        # Rotate around Z axis
        rotated_x = math.cos(stepper_c_rad) * x + math.sin(stepper_c_rad) * tilt_y
        rotated_y = -math.sin(stepper_c_rad) * x + math.cos(stepper_c_rad) * tilt_y
        logging.info(f"{stepper_pos} -> {[rotated_x, rotated_y, tilt_z, 0., 0., stepper_c + self.c_offset]}")
        return [rotated_x, rotated_y, tilt_z, 0., 0., stepper_c + self.c_offset]

    def _calc_inverse(self, pos):
        x = pos[0]
        y = pos[1]
        z = pos[2]
        c = pos[5] - self.c_offset
        # Rotate around Z axis
        rotated_x = math.cos(c) * x - math.sin(c) * y
        rotated_y = math.sin(c) * x + math.cos(c) * y
        # Tilt around X axis
        tilted_y = math.cos(self.tilt) * rotated_y - math.sin(self.tilt) * z
        tilted_z = math.sin(self.tilt) * rotated_y + math.cos(self.tilt) * z;
        return [rotated_x + self.pivot[0], tilted_y + self.pivot[1], tilted_z + self.pivot[2], c]

    def set_position(self, newpos, homing_axes):
        for i, rail in enumerate(self.rails):
            rail.set_position(newpos)

        # Valid for XYZ only
        for axis_name in homing_axes:
            axis = "xyz".index(axis_name)
            rail = self.rails[axis]
            self.limits[axis] = rail.get_range()

    def clear_homing_state(self, clear_axes):
        # Valid for XYZ only
        for axis, axis_name in enumerate("xyz"):
            if axis_name in clear_axes:
                self.limits[axis] = (1.0, -1.0)

    def home_axis(self, homing_state, axis, rail):
        # Determine movement
        position_min, position_max = rail.get_range()
        hi = rail.get_homing_info()
        homepos = [0., 0., 0., 0.]
        homepos[axis] = hi.position_endstop
        forcepos = list(homepos)
        if hi.positive_dir:
            forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
        else:
            forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
        forcepos = self._calc_forward(forcepos) + [None]
        forcepos[3:6] = [None, None, None]
        homepos = self._calc_forward(homepos) + [None]
        homepos[3:6] = [None, None, None]
        # Perform homing
        homing_state.home_rails([rail], forcepos, homepos)

    def home(self, homing_state):
        # Each axis is homed independently and in order
        for axis in homing_state.get_axes():
            # Only XYZ needs homing
            if axis < 3:
                self.home_axis(homing_state, axis, self.rails[axis])

    def _check_endstops(self, move):
        end_pos = self._calc_inverse(move.end_pos)
        for i in (0, 1, 2):
            if (move.axes_d[i]
                and (end_pos[i] < self.limits[i][0]
                     or end_pos[i] > self.limits[i][1])):
                if self.limits[i][0] > self.limits[i][1]:
                    raise move.move_error("Must home axis first")
                raise move.move_error()

    def check_move(self, move):
        limits = self.limits
        xpos, ypos = self._calc_inverse(move.end_pos)[:2]
        if (xpos < limits[0][0] or xpos > limits[0][1]
            or ypos < limits[1][0] or ypos > limits[1][1]):
            self._check_endstops(move)
        if not move.axes_d[2]:
            # Normal XY move - use defaults
            return
        # Move with Z - update velocity and accel for slower Z axis
        self._check_endstops(move)
        z_ratio = move.move_d / abs(move.axes_d[2])
        move.limit_speed(
            self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio)

    def get_status(self, eventtime):
        axes = [a for a, (l, h) in zip("xyz", self.limits) if l <= h]
        return {
            'homed_axes': "".join(axes),
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }

    def calc_move_distance(self, start_pos, end_pos):
        d_xyz = math.sqrt(sum([(e - s) ** 2 for (s, e) in zip(start_pos[:3], end_pos[:3])]))
        if d_xyz > 0.0001:
            return d_xyz
        else:
            logging.info(f"Rotation-only move: {start_pos} to {end_pos}")
            # TODO: rotation only move
            d_abc = math.sqrt(sum([(e - s) ** 2 for (s, e) in zip(start_pos[3:6], end_pos[3:6])]))
            return d_abc

    def get_calibration(self):
        return DancingBedCalibration(self.tilt, self.pivot, self.c_offset)

# Parameters for DANCING_BED_CALIBRATE
class DancingBedCalibration:
    def __init__(self, tilt, pivot, c_offset):
        self.tilt = tilt
        self.pivot = pivot
        self.c_offset = c_offset

    def save_state(self, configfile):
        configfile.set('printer', 'tilt', "%.6f" % (self.tilt,))
        configfile.set('printer', 'pivot', "%.6f, %.6f, %.6f" % self.pivot)
        configfile.set('printer', 'c_offset', "%.6f" % (self.c_offset,))

def load_kinematics(toolhead, config):
    return DancingBedKinematics(toolhead, config)

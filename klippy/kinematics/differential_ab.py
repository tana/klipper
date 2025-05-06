# Code for handling the kinematics of corexy robots
#
# Copyright (C) 2017-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math
import stepper

class DifferentialABKinematics:
    def __init__(self, toolhead, config):
        # Setup axis rails
        self.rails = [stepper.LookupMultiRail(config.getsection('stepper_' + n))
                      for n in 'xyzlr']
        #for s in self.rails[4].get_steppers():
        #    self.rails[3].get_endstops()[0][0].add_stepper(s)
        #for s in self.rails[3].get_steppers():
        #    self.rails[4].get_endstops()[0][0].add_stepper(s)
        self.rails[0].setup_itersolve('differential_ab_stepper_alloc', b'x')
        self.rails[1].setup_itersolve('differential_ab_stepper_alloc', b'y')
        self.rails[2].setup_itersolve('differential_ab_stepper_alloc', b'z')
        self.rails[3].setup_itersolve('differential_ab_stepper_alloc', b'l')
        self.rails[4].setup_itersolve('differential_ab_stepper_alloc', b'r')
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
        # Setup boundary checks
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat(
            'max_z_velocity', max_velocity, above=0., maxval=max_velocity)
        self.max_z_accel = config.getfloat(
            'max_z_accel', max_accel, above=0., maxval=max_accel)
        self.limits = [(1.0, -1.0)] * 5
        ranges = [r.get_range() for r in self.rails]
        self.axes_min = toolhead.Coord(*[r[0] for r in ranges], c=0., e=0.)
        self.axes_max = toolhead.Coord(*[r[1] for r in ranges], c=0., e=0.)
        # Per-axis speed limits (for 6-axis version)
        self.max_speed_x = max_velocity
        self.max_speed_y = max_velocity
        self.max_speed_z = self.max_z_velocity
        self.max_speed_a = config.getfloat('max_angular_velocity')
        self.max_speed_b = config.getfloat('max_angular_velocity')
        self.max_speed_c = config.getfloat('max_angular_velocity')
    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]
    def calc_position(self, stepper_positions):
        pos = [stepper_positions[rail.get_name()] for rail in self.rails]
        return [pos[0], pos[1], pos[2], (pos[3] + pos[4]) / 2, (pos[3] - pos[4]) / 2, 0.0]
    def calc_move_distance(self, start_pos, end_pos):
        # For 6-axis version
        d_xyz = math.sqrt(sum([(e - s) ** 2 for (s, e) in zip(start_pos[:3], end_pos[:3])]))
        if d_xyz > 0.0001:
            return d_xyz
        else:
            logging.info(f"Rotation-only move: {start_pos} to {end_pos}")
            # TODO: rotation only move
            d_abc = math.sqrt(sum([(e - s) ** 2 for (s, e) in zip(start_pos[3:6], end_pos[3:6])]))
            return d_abc
    def set_position(self, newpos, homing_axes):
        for i, rail in enumerate(self.rails):
            rail.set_position(newpos)
            if "xyzlr"[i] in homing_axes:
                self.limits[i] = rail.get_range()
    def clear_homing_state(self, clear_axes):
        for axis, axis_name in enumerate("xyzlr"):
            if axis_name in clear_axes:
                self.limits[axis] = (1.0, -1.0)
    def home(self, homing_state):
        # Always home all axis other than extruder
        for axis, rail in enumerate(self.rails):
            # Determine movement
            position_min, position_max = rail.get_range()
            hi = rail.get_homing_info()
            # xyzabce
            homepos = [None, None, None, None, None, None, None]
            homepos[axis] = hi.position_endstop
            if axis == 3:
                homepos[3] = hi.position_endstop
                homepos[4] = hi.position_endstop
            elif axis == 4:
                homepos[3] = -hi.position_endstop
                homepos[4] = hi.position_endstop
            forcepos = list(homepos)
            if hi.positive_dir:
                forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
            else:
                forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
            # Perform homing
            homing_state.home_rails([rail], forcepos, homepos)
    def _check_endstops(self, move):
        end_pos = move.end_pos
        for i in (0, 1, 2):
            if (move.axes_d[i]
                and (end_pos[i] < self.limits[i][0]
                     or end_pos[i] > self.limits[i][1])):
                if self.limits[i][0] > self.limits[i][1]:
                    raise move.move_error("Must home axis first")
                raise move.move_error()
    def check_move(self, move):
        limits = self.limits
        xpos, ypos = move.end_pos[:2]
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
        axes = [a for a, (l, h) in zip("xyzlr", self.limits) if l <= h]
        return {
            'homed_axes': "".join(axes),
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }

def load_kinematics(toolhead, config):
    return DifferentialABKinematics(toolhead, config)

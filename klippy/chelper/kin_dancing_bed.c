// DancingBed kinematics stepper pulse time generation
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stdlib.h> // malloc
#include <string.h> // memset
#include <stddef.h> // offsetof
#include <math.h> // sin,cos
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "pyhelper.h" // errorf
#include "trapq.h" // move_get_coord

struct db_stepper {
    struct stepper_kinematics sk;
    double tilt, x0, y0, z0, c_offset;
};

static double
db_stepper_x_calc_position(struct stepper_kinematics *sk, struct move *m
                             , double move_time)
{
    struct db_stepper *dbs = container_of(sk, struct db_stepper, sk);
    struct coord pos = move_get_coord(m, move_time);
    double c = pos.c - dbs->c_offset;
    // Rotate around Z axis
    double sc = sin(c);
    double cc = cos(c);
    double rotated_x = cc * pos.x - sc * pos.y;
    // X stepper position does not depend on tilt
    return rotated_x + dbs->x0;
}

static double
db_stepper_y_calc_position(struct stepper_kinematics *sk, struct move *m
                             , double move_time)
{
    struct db_stepper *dbs = container_of(sk, struct db_stepper, sk);
    struct coord pos = move_get_coord(m, move_time);
    double c = pos.c - dbs->c_offset;
    // Rotate around Z axis
    double sc = sin(c);
    double cc = cos(c);
    double rotated_y = sc * pos.x + cc * pos.y;
    // Tilt around X axis
    double st = sin(dbs->tilt);
    double ct = cos(dbs->tilt);
    double tilted_y = ct * rotated_y - st * pos.z;
    return tilted_y + dbs->y0;
}

static double
db_stepper_z_calc_position(struct stepper_kinematics *sk, struct move *m
                             , double move_time)
{
    struct db_stepper *dbs = container_of(sk, struct db_stepper, sk);
    struct coord pos = move_get_coord(m, move_time);
    double c = pos.c - dbs->c_offset;
    // Rotate around Z axis
    double sc = sin(c);
    double cc = cos(c);
    double rotated_y = sc * pos.x + cc * pos.y;
    // Tilt around X axis
    double st = sin(dbs->tilt);
    double ct = cos(dbs->tilt);
    double tilted_z = st * rotated_y + ct * pos.z;
    return tilted_z + dbs->z0;
}

static double
db_stepper_c_calc_position(struct stepper_kinematics *sk, struct move *m
                             , double move_time)
{
    struct db_stepper *dbs = container_of(sk, struct db_stepper, sk);
    return move_get_coord(m, move_time).c - dbs->c_offset;
}

struct stepper_kinematics * __visible
dancing_bed_stepper_alloc(char axis, double tilt, double x0, double y0, double z0, double c_offset)
{
    struct db_stepper *dbs = malloc(sizeof(*dbs));
    memset(dbs, 0, sizeof(*dbs));
    dbs->tilt = tilt;
    dbs->x0 = x0;
    dbs->y0 = y0;
    dbs->z0 = z0;
    dbs->c_offset = c_offset;
    if (axis == 'x') {
        dbs->sk.calc_position_cb = db_stepper_x_calc_position;
        dbs->sk.active_flags = AF_X;
    } else if (axis == 'y') {
        dbs->sk.calc_position_cb = db_stepper_y_calc_position;
        dbs->sk.active_flags = AF_Y;
    } else if (axis == 'z') {
        dbs->sk.calc_position_cb = db_stepper_z_calc_position;
        dbs->sk.active_flags = AF_Z;
    } else if (axis == 'c') {
        dbs->sk.calc_position_cb = db_stepper_c_calc_position;
        dbs->sk.active_flags = AF_C;
    }
    return &dbs->sk;
}

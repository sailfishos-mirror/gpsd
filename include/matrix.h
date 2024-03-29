/*
 * matrix.h - matrix-algebra prototypes
 *
 * This file is Copyright by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 */

extern bool matrix_invert(double mat[4][4], double inverse[4][4]);
extern void matrix_symmetrize(double mat[4][4], double inverse[4][4]);

// vim: set expandtab shiftwidth=4

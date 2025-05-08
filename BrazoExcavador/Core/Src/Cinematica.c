/*
 * Cinematica.c
 *
 *  Created on: May 8, 2025
 *      Author: Usuario
 */


#include "Cinematica.h"
#include <math.h>

#define L1 100.0f  // Aún por definir las longitudes de las extremidades
#define L2 100.0f
#define L3 100.0f
#define L4 50.0f

Position forward_kinematics(Angles a) {
    Position p;

    float t1 = a.theta1 * M_PI / 180.0f;
    float t2 = a.theta2 * M_PI / 180.0f;
    float t3 = a.theta3 * M_PI / 180.0f;

    float r = L2*cosf(t2) + L3*cosf(t2 + t3);
    p.x = cosf(t1) * r;
    p.y = sinf(t1) * r;
    p.z = L1 + L2*sinf(t2) + L3*sinf(t2 + t3);

    return p;
}

Angles inverse_kinematics(Position p) {
    Angles a;
    a.theta1 = atan2f(p.y, p.x) * 180.0f / M_PI;

    float r = sqrtf(p.x * p.x + p.y * p.y);
    float z = p.z - L1;

    float D = (r*r + z*z - L2*L2 - L3*L3) / (2 * L2 * L3);
    a.theta3 = atan2f(sqrtf(1 - D*D), D) * 180.0f / M_PI;

    float phi = atan2f(z, r);
    float beta = atan2f(L3*sinf(a.theta3*M_PI/180.0f), L2 + L3*cosf(a.theta3*M_PI/180.0f));
    a.theta2 = (phi - beta) * 180.0f / M_PI;


    a.theta4 = 0.0f;

    return a;
}

Velocity differential_kinematics(Angles a, JointVelocity dq) {
    float t1 = a.theta1 * M_PI / 180.0f;
    float t2 = a.theta2 * M_PI / 180.0f;
    float t3 = a.theta3 * M_PI / 180.0f;

    float dt1 = dq.dtheta1 * M_PI / 180.0f;
    float dt2 = dq.dtheta2 * M_PI / 180.0f;
    float dt3 = dq.dtheta3 * M_PI / 180.0f;

    float s1 = sinf(t1), c1 = cosf(t1);
    float s2 = sinf(t2), c2 = cosf(t2);
    float s23 = sinf(t2 + t3), c23 = cosf(t2 + t3);

    // Derivadas parciales para el Jacobiano J
    float j11 = -s1 * (L2 * c2 + L3 * c23);
    float j12 = -c1 * (L2 * s2 + L3 * s23);
    float j13 = -c1 * (L3 * s23);

    float j21 =  c1 * (L2 * c2 + L3 * c23);
    float j22 = -s1 * (L2 * s2 + L3 * s23);
    float j23 = -s1 * (L3 * s23);

    float j32 =  L2 * c2 + L3 * c23;
    float j33 =  L3 * c23;

    // Velocidades lineales
    Velocity v;
    v.dx = j11 * dt1 + j12 * dt2 + j13 * dt3;
    v.dy = j21 * dt1 + j22 * dt2 + j23 * dt3;
    v.dz = j32 * dt2 + j33 * dt3;

    return v;
}


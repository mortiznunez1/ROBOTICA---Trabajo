/*
 * Cinematica.c
 *
 *  Created on: May 8, 2025
 *      Author: Usuario
 */


#include "Cinematica.h"
#include <math.h>

#define L1 14.0f //Aún no son definitivas
#define L2 12.0f
#define L3 12.0f

#define DEG2RAD(a) ((a) * M_PI / 180.0f)
#define RAD2DEG(a) ((a) * 180.0f / M_PI)

Position forward_kinematics(Angles a) {
    float t1 = DEG2RAD(a.theta1);
    float t2 = DEG2RAD(a.theta2);
    float t3 = DEG2RAD(a.theta3);

    float r = L2 * cosf(t2) + L3 * cosf(t2 + t3);

    Position p;
    p.x = cosf(t1) * r;
    p.y = sinf(t1) * r;
    p.z = L1 + L2 * sinf(t2) + L3 * sinf(t2 + t3);
    p.orientation = a.theta4;  // Orientación del efector (independiente)

    return p;
}

Angles inverse_kinematics(Position p, float desired_orientation) {
    Angles a;

    // θ1: rotación base
    a.theta1 = RAD2DEG(atan2f(p.y, p.x));

    float r = sqrtf(p.x * p.x + p.y * p.y);
    float z = p.z - L1;

    // Ley del coseno para θ3
    float D = (r*r + z*z - L2*L2 - L3*L3) / (2 * L2 * L3);
    if (D > 1.0f) D = 1.0f;
    if (D < -1.0f) D = -1.0f;

    float theta3_rad = atan2f(sqrtf(1 - D*D), D);
    a.theta3 = RAD2DEG(theta3_rad);

    // θ2: ángulo del primer brazo
    float phi = atan2f(z, r);
    float beta = atan2f(L3 * sinf(theta3_rad), L2 + L3 * cosf(theta3_rad));
    a.theta2 = RAD2DEG(phi - beta);

    // θ4: orientación deseada del efector (libre)
    a.theta4 = desired_orientation;

    return a;
}

Velocity differential_kinematics(Angles a, JointVelocity dq) {
    float t1 = DEG2RAD(a.theta1);
    float t2 = DEG2RAD(a.theta2);
    float t3 = DEG2RAD(a.theta3);

    float dt1 = DEG2RAD(dq.dtheta1);
    float dt2 = DEG2RAD(dq.dtheta2);
    float dt3 = DEG2RAD(dq.dtheta3);

    float s1 = sinf(t1), c1 = cosf(t1);
    float s2 = sinf(t2), c2 = cosf(t2);
    float s23 = sinf(t2 + t3), c23 = cosf(t2 + t3);

    // Derivadas parciales del Jacobiano
    float j11 = -s1 * (L2 * c2 + L3 * c23);
    float j12 = -c1 * (L2 * s2 + L3 * s23);
    float j13 = -c1 * (L3 * s23);

    float j21 =  c1 * (L2 * c2 + L3 * c23);
    float j22 = -s1 * (L2 * s2 + L3 * s23);
    float j23 = -s1 * (L3 * s23);

    float j32 =  L2 * c2 + L3 * c23;
    float j33 =  L3 * c23;

    Velocity v;
    v.dx = j11 * dt1 + j12 * dt2 + j13 * dt3;
    v.dy = j21 * dt1 + j22 * dt2 + j23 * dt3;
    v.dz = j32 * dt2 + j33 * dt3;
    v.dorientation = dq.dtheta4;  // Independiente

    return v;
}

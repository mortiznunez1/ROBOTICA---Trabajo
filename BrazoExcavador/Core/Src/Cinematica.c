/*
 * Cinematica.c
 *
 *  Created on: May 8, 2025
 *      Author: Usuario
 */


#include "Cinematica.h"
#include <math.h>

#define L1 4.5f
#define L2 11.7f
#define L3 11.0f
#define L4 12.5f

#define DEG2RAD(a) ((a) * M_PI / 180.0f)
#define RAD2DEG(a) ((a) * 180.0f / M_PI)

Position forward_kinematics(Angles a) {
    float t1 = DEG2RAD(a.theta1);
    float t2 = DEG2RAD(a.theta2);
    float t3 = DEG2RAD(a.theta3);
    float t4 = DEG2RAD(a.theta4);

    float t23 = t2 + t3;
    float t234 = t23 + t4;

    float r = L2 * cosf(t2) + L3 * cosf(t23) + L4 * cosf(t234);

    Position p;
    p.x = cosf(t1) * r;
    p.y = sinf(t1) * r;
    p.z = L1 + L2 * sinf(t2) + L3 * sinf(t23) + L4 * sinf(t234);
    p.orientation = a.theta4;

    return p;
}

Angles inverse_kinematics(Position p, float desired_orientation) {
    Angles a;

    float r = sqrtf(p.x * p.x + p.y * p.y);
    float z_prime = p.z - L1;

    float L3p = L3 + L4;

    a.theta1 = RAD2DEG(atan2f(p.y, p.x));

    float D = (r * r + z_prime * z_prime - L2 * L2 - L3p * L3p) / (2 * L2 * L3p);
    if (D > 1.0f) D = 1.0f;
    if (D < -1.0f) D = -1.0f;

    float theta3p_rad = acosf(D);
    float phi = atan2f(z_prime, r);
    float psi = atan2f(L3p * sinf(theta3p_rad), L2 + L3p * cosf(theta3p_rad));

    a.theta2 = RAD2DEG(phi - psi);
    a.theta3 = RAD2DEG(theta3p_rad);
    a.theta4 = desired_orientation - a.theta2 - a.theta3;

    return a;
}

Velocity differential_kinematics(Angles a, JointVelocity dq) {

    float t1 = DEG2RAD(a.theta1);
    float t2 = DEG2RAD(a.theta2);
    float t3 = DEG2RAD(a.theta3);
    float t4 = DEG2RAD(a.theta4);

    float dt1 = DEG2RAD(dq.dtheta1);
    float dt2 = DEG2RAD(dq.dtheta2);
    float dt3 = DEG2RAD(dq.dtheta3);
    float dt4 = DEG2RAD(dq.dtheta4);


    float t23 = t2 + t3;
    float t234 = t2 + t3 + t4;


    float s1 = sinf(t1), c1 = cosf(t1);
    float s2 = sinf(t2), c2 = cosf(t2);
    float s23 = sinf(t23), c23 = cosf(t23);
    float s234 = sinf(t234), c234 = cosf(t234);


    float R = L2 * c2 + L3 * c23 + L4 * c234;
    float S2 = L2 * s2 + L3 * s23 + L4 * s234;
    float S3 = L3 * s23 + L4 * s234;
    float S4 = L4 * s234;
    float C2 = L2 * c2 + L3 * c23 + L4 * c234;
    float C3 = L3 * c23 + L4 * c234;
    float C4 = L4 * c234;


    Velocity v;
    v.dx = -s1 * R * dt1 - c1 * S2 * dt2 - c1 * S3 * dt3 - c1 * S4 * dt4;
    v.dy =  c1 * R * dt1 - s1 * S2 * dt2 - s1 * S3 * dt3 - s1 * S4 * dt4;
    v.dz =           0.0f * dt1 + C2 * dt2 + C3 * dt3 + C4 * dt4;
    v.dorientation = dq.dtheta4; // directa

    return v;
}

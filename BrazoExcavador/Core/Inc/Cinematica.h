/*
 * Cinematica.h
 *
 *  Created on: May 8, 2025
 *      Author: Usuario
 */

#ifndef INC_CINEMATICA_H_
#define INC_CINEMATICA_H_

typedef struct {
    float x, y, z;
} Position;

typedef struct {
    float theta1, theta2, theta3, theta4;
} Angles;

typedef struct {
    float dx, dy, dz;
} Velocity;

typedef struct {
    float dtheta1, dtheta2, dtheta3, dtheta4;
} JointVelocity;

Position forward_kinematics(Angles a);
Angles inverse_kinematics(Position p);
Velocity differential_kinematics(Angles a, JointVelocity dq);


#endif /* INC_CINEMATICA_H_ */

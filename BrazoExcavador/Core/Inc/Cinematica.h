/*
 * Cinematica.h
 *
 *  Created on: May 8, 2025
 *      Author: Usuario
 */

#ifndef INC_CINEMATICA_H_
#define INC_CINEMATICA_H_

typedef struct {
    float x, y, z;          // Posición del efector final
    float orientation;      // Orientación del efector (rotación propia - θ₄)
} Position;

typedef struct {
    float theta1, theta2, theta3, theta4; // Ángulos de las articulaciones en grados
} Angles;

typedef struct {
    float dx, dy, dz;       // Velocidades lineales en cm/s
    float dorientation;     // Velocidad angular del efector (en grados/s)
} Velocity;

typedef struct {
    float dtheta1, dtheta2, dtheta3, dtheta4; // Velocidades articulares en grados/s
} JointVelocity;

// Prototipos
Position forward_kinematics(Angles a);
Angles inverse_kinematics(Position p, float desired_orientation);
Velocity differential_kinematics(Angles a, JointVelocity dq);


#endif /* INC_CINEMATICA_H_ */

#ifndef PILOTE_MODE_H
#define PILOTE_MODE_H

void setup_pilote_mode();
void loop_pilote_mode(float dt);
void applyControl(float tau_x, float tau_y, float tau_z);

// Declare T as an external variable
extern float T;

#endif // PILOTE_MODE_H
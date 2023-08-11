#ifndef __MOTOR_PID_H
#define __MOTOR_OID_H

void IncPIDInit(void);
int IncPIDCalc(int NextPoint);
unsigned int LocPIDCalc(int NextPoint);

#endif

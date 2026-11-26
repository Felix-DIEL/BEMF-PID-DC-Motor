/* 
 * File:   pid_regulator.h
 * Author: stan
 *
 * Created on 28. Mai 2013, 08:40
 */

#ifndef PID_REGULATOR_H
#define	PID_REGULATOR_H

#ifdef	__cplusplus
extern "C" {
#endif

#ifdef	__cplusplus
}
#endif

//----<Defines>----
//----</Defines>----



//----<Local Variables>----
typedef struct _stPIDParams
{
    float kp;           //Proportional term
    float ki;           //Integral term
    float kd;           //Deriviate term
    float t;            //Regulator timer periode (s)

    float Integral;     //Error integral
    float last_et;      //Last error
    float ut[3];        //Output
}stPIDParams;

//----</Local Variables----

//----<Declarations>----
void ConfigPID (stPIDParams *pstPIDParams, float kp, float ki, float kd, float t);
float PIDRegulator (float wt, float yt, stPIDParams *pstPIDParams);
//----</Declarations>----

#endif	/* PID_REGULATOR_H */

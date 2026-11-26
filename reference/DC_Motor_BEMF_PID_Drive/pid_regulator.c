/*
 * File:   main.c
 * Author: stan
 *
 * Created on 28. May 2013, 08:46
 */
#include "pid_regulator.h"

//----<Defines>----
//----</Defines>----

//----<Interrup Declarations>----
//----</Interrup Declaration>----

//----<Local Variables>----
extern stPIDParams MystPIDParams;
extern stPIDParams *pstPIDParams;
//----</Local Variables----


/* Function ConfigPID
 *  Description: Configures all terms of the PID-Regulator.
 *  Input: Structure of PID parameters, proportional, intergral and deriviative term. Time intervall for loop time.
 *  Output: None
 *  Side Effects: None observed
 *
 */
void ConfigPID (stPIDParams *pstPIDParams, float kp, float ki, float kd, float t)
{
    pstPIDParams->kp=kp;
    pstPIDParams->ki=ki;
    pstPIDParams->kd=kd;

    if (t>0)    //Check if sane...
    {
    pstPIDParams->t=t;
    }
    else
    {
    pstPIDParams->t=1; //Fallback: 1s
    }

}



/* Function PIDRegulator
 *  Description: Performs a software PID regulation loop.
 *  Input: w(t), y(t), PID parameters.
 *  Output: u(t)
 *  Side Effects: None observed.  Execute config PID before using this function
 *                    _____             _________
 *                   |     |           |         |
 *  w(t)--->O--e(t)->| PID |---u(t)--->| Strecke |---y(t)--->
 *          |        |_____|           |_________|   |
 *          |<----------------feedback---------------|
 *
 */
float PIDRegulator (float wt, float yt, stPIDParams *pstPIDParams)
{
    float ut=0;
    float et=0;
    float deriviative=0;


    et=wt-yt;                                                               //Get error from set value to output value

    if (et>1 || et<1)
    {
        pstPIDParams->ut[2]=pstPIDParams->ut[1];                                //Shift filter array
        pstPIDParams->ut[1]=pstPIDParams->ut[0];                                //Shift filter array

        pstPIDParams->Integral=pstPIDParams->Integral+et*pstPIDParams->t;       //Add error to integral
        deriviative=(et-pstPIDParams->last_et)/pstPIDParams->t;                 //Calculate deriviative term

    //                 Proportional                  Integral                                 Deriviative
        pstPIDParams->ut[0]=(pstPIDParams->kp*et)+(pstPIDParams->ki*pstPIDParams->Integral)+(pstPIDParams->kd*deriviative); //Calculate output

        pstPIDParams->last_et=et;                                               //Store last error for next deriviative calculations

    }
    return (pstPIDParams->ut[0]+pstPIDParams->ut[1]+pstPIDParams->ut[2])/3; //Return filtered output
}

/*********************************************************************************//**
* @file Robit.h
* Header file for the myRobit class and its thread functions
*************************************************************************************/
#ifndef _ROBIT_H
#define _ROBIT_H

/*************************************************************************************
* Includes
*************************************************************************************/
#include "WPILib.h"

#include "JoystickButtons.h"

#include "fcntl.h"
#include "pthread.h"
#include "semaphore.h"

/*************************************************************************************
* Defines
*************************************************************************************/
#define L_DRIVE_PORT        2               /**< Left drive Talon                  */
#define R_DRIVE_PORT        1               /**< Right drive Talon                  */

#define DEAD_ZONE           0.2             /**< Dead zone on the Joystick          */
#define PRESSURE_SWITCH     0               /**< Pressure switch DIO port           */
#define TURN_REDUC_FACTOR   8.0             /**< How to adjust the turn power reduction */
#define UNEVEN_ADJ_FACTOR   0.09            /**< Adjustment for the mechanical difference :( */

/*********************************************************************************//**
* Main robot class
*************************************************************************************/
class myRobit : public IterativeRobot {
public:
    myRobit();                              /**< Constructor                        */
    ~myRobit();                             /**< Destructor                         */

    void RobotInit();                       /**< Robot init function                */
    void AutonomousInit();                  /**< Autonomous init function           */
    
    void DisabledInit();
    void TeleopInit();
    void TestInit();

    void DisabledPeriodic();
    void AutonomousPeriodic();
    void TeleopPeriodic();
    void TestPeriodic();
};

/*************************************************************************************
* Global variables
*************************************************************************************/
pthread_t                   driveThread;    /**< Drive thread object                */
pthread_t                   inputThread;    /**< Input thread object                */
pthread_t                   winchThread;    /**< Winch thread object                */

Talon *                     lDriveTalon;    /**< Left drive Talon                   */
Talon *                     rDriveTalon;    /**< Right drive Talon                  */

AnalogChannel *             ACDC;           /**< Load sensor analong input          */
Talon *                     filthyWench;    /**< Winch Talon                        */
Joystick *                  joystick;       /**< Joystick for input                 */

extern float                desVoltage;     /**< Desired voltage on load cell       */

/*************************************************************************************
* Actuator thread functions
*************************************************************************************/
void * driveFunc( void * );                 /**< Drive thread function              */
void * inputFunc( void * );                 /**< Input thread function              */
void * winchFunc( void * );                 /**< Winch thread function              */


#endif

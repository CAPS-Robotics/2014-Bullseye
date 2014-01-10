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

#include "fcntl.h"
#include "pthread.h"
#include "semaphore.h"

/*************************************************************************************
* Defines
*************************************************************************************/
#define FRONT_RIGHT_MOTOR   2               /**< Front right motor port             */
#define FRONT_LEFT_MOTOR    1               /**< Front left motor port              */
#define BACK_RIGHT_MOTOR    4               /**< Back right motor port              */
#define BACK_LEFT_MOTOR     3               /**< Back left motor port               */

#define COMPRESSOR_PORT     1               /**< Compressor relay port              */
#define JOYSTICK_PORT       1               /**< Joystick USB port                  */
#define PRESSURE_SWITCH     1               /**< Pressure switch DIO port           */
#define SHOOTER_PISTON_FORE 1               /**< Shooter piston fore solenoid port  */
#define SHOOTER_PISTON_BACK 2               /**< Shooter piston back solenoid port  */
#define TALON_PORT          5               /**< Shooter motor port                 */

#define BTN_X               1               /**< Button X channel on the joystick   */
#define BTN_LBM             5
#define BTN_LTG             7
#define BTN_RTG             8

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
sem_t *                     shooter_semaphore;
                                            /**< Shot synchro semaphore             */
pthread_t                   driveThread;    /**< Drive thread object                */
pthread_t                   inputThread;    /**< Input thread object                */
pthread_t                   shooterThread;  /**< Shooter thread object              */

Compressor *                compressor;     /**< Compressor for pneumatics          */
RobotDrive *                drivetrain;     /**< Drivetrain - Mecanum               */
Joystick *                  joystick;       /**< Joystick for input                 */
Talon *                     shooter_motor;  /**< CIM for shooting                   */
DoubleSolenoid *            shooter_piston; /**< Piston for firing discs            */

/*************************************************************************************
* Actuator thread functions
*************************************************************************************/
void * driveFunc( void * );                 /**< Drive thread function              */
void * inputFunc( void * );                 /**< Input thread function              */
void * shooterFunc( void * );               /**< Shooter thread function            */

#endif

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

#include "math.h"
#include <pthread.h>

/*************************************************************************************
* Defines
*************************************************************************************/
// Drive motor constants
#define LF_DRIVE_PORT       1               /**< Left F drive Talon                 */
#define RF_DRIVE_PORT       2               /**< Right F drive Talon                */
#define LR_DRIVE_PORT       3               /**< Left R drive Talon                 */
#define RR_DRIVE_PORT       4               /**< Right R drive Talon                */

// Drive loop constants
#define DEAD_ZONE           0.2             /**< Dead zone on the Joystick          */

// Compressor things
#define COMPRESSOR_PORT		1				/**< Compressor relay port				*/
#define PRESSURE_SWITCH     1               /**< Pressure switch DIO port           */

// Winch stuffs
#define LOADCELL_CHANNEL    1               /**< Loadcell analog channel            */
#define WINCH_PORT          5               /**< Winch talon                        */
#define WINCH_RELEASE_PORT  1               /**< Piston that quickreleases the winch */

// Ball aq gizmos
#define BALLAQ_DEPLOY       2               /**< Piston port to extend the ball aq  */
#define BALLAQ_UNDEPLOY     3               /**< Piston port to retract the ball aq */
#define BALLAQ_WINDIN       6               /**< Talon to wind balls into the shooter */

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

RobotDrive *                rDrive;         /**< Robot drive object                 */

AnalogChannel *             ACDC;           /**< Load sensor analong input          */
DoubleSolenoid *            aqDeploy;       /**< Deploy / Undeploy ball aq          */
Compressor *                compressor;     /**< Compressor for pneumatics          */
Talon *                     filthyWench;    /**< Winch Talon                        */
Joystick *                  joystick;       /**< Joystick for input                 */
Solenoid *                  winchRelease;   /**< Release piston for the winch       */
Talon *                     windInBall;     /**< Talon on the ball aq system        */

extern float                desVoltage;     /**< Desired voltage on load cell       */

/*************************************************************************************
* Actuator thread functions
*************************************************************************************/
void * driveFunc( void * );                 /**< Drive thread function              */
void * inputFunc( void * );                 /**< Input thread function              */
void * winchFunc( void * );                 /**< Winch thread function              */

#endif

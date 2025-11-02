package frc.robot

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import kotlin.math.PI

/*
 * The Constants file provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This file should not be used for any other purpose.
 * All String, Boolean, and numeric (Int, Long, Float, Double) constants should use
 * `const` definitions. Other constant types should use `val` definitions.
 */

object Constants
{


    object OperatorConstants
    {
        const val DRIVER_CONTROLLER_PORT = 0
    }




    object RobotConstants {

        const val RATE_LIMIT = 3.5

        /** Other CAN ID */
        const val PDH_CAN = 1

        /** Controller Configurations */
        const val TRANSLATION_DEADBAND = .15
        const val ROTATION_DEADBAND = .15

        /** In kilograms, include bumpers and battery and all */
        const val ROBOT_WEIGHT = 55.0

        /** Drive configuration */
        const val MAX_LINEAR_SPEED = 1.5
        const val MAX_ROT_SPEED = PI / 2 // rad/s
        val MAX_ACCEL = 15.0

        val INITIAL_POSE = Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0))

        init {
            println("Max Accel $MAX_ACCEL")
        }


        // Robot dimensions (INCLUDING BUMPERS)
        val ROBOT_WIDTH = Units.inchesToMeters(27.0 + 3.25 * 2)
        val ROBOT_LENGTH = Units.inchesToMeters(30.0 + 3.25 * 2)
    }

}




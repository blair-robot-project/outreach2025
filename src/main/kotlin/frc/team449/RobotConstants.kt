package frc.team449

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import kotlin.math.PI

object RobotConstants {
    const val RATE_LIMIT = 3.5

    /** Other CAN ID */
    const val PDH_CAN = 1

    /** Controller Configurations */
    const val ROT_RATE_LIMIT = 4.0 * PI
    const val NEG_ROT_RATE_LIM = -8.0 * PI
    const val TRANSLATION_DEADBAND = .15
    const val ROTATION_DEADBAND = .15

    /** In kilograms, include bumpers and battery and all */
    const val ROBOT_WEIGHT = 55.0

    /** Drive configuration */
    const val MAX_LINEAR_SPEED = 1.5
    const val MAX_ROT_SPEED = PI / 2 // rad/s
    const val MAX_ACCEL = 15.0

    val INITIAL_POSE = Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0))

    init {
        println("Max Accel $MAX_ACCEL")
    }

    const val LOOP_TIME = 0.020

    // Robot dimensions (INCLUDING BUMPERS)
    val ROBOT_WIDTH = Units.inchesToMeters(27.0 + 3.25 * 2)
    val ROBOT_LENGTH = Units.inchesToMeters(30.0 + 3.25 * 2)
}

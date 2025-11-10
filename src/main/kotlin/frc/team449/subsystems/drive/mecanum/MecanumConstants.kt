package frc.team449.subsystems.drive.mecanum

import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.Amps

object MecanumConstants {
    /** Drive motor ports */
    const val DRIVE_MOTOR_FL = 1
    const val DRIVE_MOTOR_FR = 2
    const val DRIVE_MOTOR_BL = 3
    const val DRIVE_MOTOR_BR = 4

    /** Feed forward values for driving each module */
    const val DRIVE_KS = 0.0 // 0.16475
    const val DRIVE_KV = 0.0 // 2.0909
    const val DRIVE_KA = 0.0 // 0.29862

    /** PID gains for driving each module*/
    const val DRIVE_KP = 0.08//0.35
    const val DRIVE_KI = 0.0
    const val DRIVE_KD = 0.0

    /** Drive configuration */
    const val DRIVE_GEARING = 1 / 8.0
    val DRIVE_UPR = Math.PI * Units.inchesToMeters(6.0)
    const val MAX_ATTAINABLE_WHEEL_SPEED = (12 - DRIVE_KS) / DRIVE_KV
    val WHEEL_BASE = Units.inchesToMeters(21.426)
    val TRACK_WIDTH = Units.inchesToMeters(21.000)
    val CURRENT_LIM = Amps.of(50.0)
}
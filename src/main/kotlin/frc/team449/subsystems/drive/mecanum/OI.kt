package frc.team449.subsystems.drive.mecanum

import edu.wpi.first.math.kinematics.ChassisSpeeds

/**
 * An Operator Input (OI) that gets the desired ChassisSpeeds to give a drivetrain
 */
fun interface OI {
    fun get(): ChassisSpeeds
}

package frc.team449.drive.mecanum

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.drive.DriveSubsystem

/**
 * Generic driving command that applies the OI output to the Drive
*/
class MecanumDriveCommand(
    private val drive: DriveSubsystem,
    private val oi: OI
) : Command() {
    init {
        addRequirements(drive)
    }

    /**
     * Feed [ChassisSpeeds] from the joystick/[OI] to drive[DriveSubsystem]
     */
    override fun execute() {
        drive.set(oi.get())
    }
}

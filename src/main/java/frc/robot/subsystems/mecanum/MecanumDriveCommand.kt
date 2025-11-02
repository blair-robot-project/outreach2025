package frc.robot.subsystems.mecanum

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.control.holonomic.mecanum.MecanumDrive

class MecanumDriveCommand(
    private val drive: MecanumDrive,
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


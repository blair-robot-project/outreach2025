package frc.robot

import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.control.holonomic.mecanum.MecanumDrive

abstract class RobotBase {

    val field = Field2d()

    abstract val powerDistribution: PowerDistribution

    abstract val drive: MecanumDrive

    abstract val driveCommand: Command
}

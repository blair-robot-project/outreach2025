package frc.robot

import com.studica.frc.AHRS
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.XboxController
import frc.robot.subsystems.mecanum.MecanumDriveCommand
import frc.robot.subsystems.mecanum.MecanumOI.Companion.createMecanumOI
import frc.team449.control.holonomic.mecanum.MecanumDrive

class Robot : RobotBase() {

    val driveController = XboxController(0)

//    val ahrs = AHRS(SPI.Port.kMXP)
    val ahrs = AHRS(AHRS.NavXComType.kMXP_SPI)
    // Instantiate/declare PDP and othe';[r stuff here

    override val powerDistribution: PowerDistribution = PowerDistribution(
        Constants.RobotConstants.PDH_CAN,
        PowerDistribution.ModuleType.kRev
    )

    override val drive = MecanumDrive.createMecanum(ahrs)

    val oi = createMecanumOI(drive, driveController)

    override val driveCommand = MecanumDriveCommand(drive, oi)



}

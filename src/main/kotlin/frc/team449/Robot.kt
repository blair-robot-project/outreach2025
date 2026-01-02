package frc.team449

import com.studica.frc.AHRS
import edu.wpi.first.epilogue.Logged
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.team449.leds.light.Light
import frc.team449.subsystems.drive.mecanum.MecanumDrive
import frc.team449.subsystems.drive.mecanum.MecanumDriveCommand
import frc.team449.subsystems.drive.mecanum.MecanumOI.Companion.createMecanumOI
import frc.team449.subsystems.indexer.Indexer
import frc.team449.subsystems.intake.Intake
import frc.team449.subsystems.shooter.Shooter

@Logged
class Robot {

    val field = Field2d()

    val driveController = CommandXboxController(0)

    val ahrs = AHRS(AHRS.NavXComType.kMXP_SPI)

    // Instantiate/declare PDP and other stuff here

    val powerDistribution: PowerDistribution =
        PowerDistribution(
            RobotConstants.PDH_CAN,
            PowerDistribution.ModuleType.kRev
        )

    val drive = MecanumDrive.createMecanum(ahrs)

    val oi = createMecanumOI(drive, driveController)

    val driveCommand = MecanumDriveCommand(drive, oi)

    val light = Light.createLight()

    val intake = Intake()

    val shooter = Shooter()

    val indexer = Indexer()

//  val infrared = DigitalInput(RobotConstants.IR_CHANNEL)
}

package frc.team449

import com.studica.frc.AHRS
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.team449.drive.mecanum.MecanumDrive
import frc.team449.drive.mecanum.MecanumDriveCommand
import frc.team449.drive.mecanum.MecanumOI.Companion.createMecanumOI
import frc.team449.outreach2025.constants.RobotConstants
import frc.team449.outreach2025.subsystems.indexer.Indexer
import frc.team449.outreach2025.subsystems.intake.Intake
import frc.team449.outreach2025.subsystems.light.Light
import frc.team449.outreach2025.subsystems.shooter.Shooter
import monologue.Annotations.Log
import monologue.Logged

class Robot :
    RobotBase(),
    Logged {
    val driveController = CommandXboxController(0)

    private val ahrs = AHRS(AHRS.NavXComType.kMXP_SPI)

    // Instantiate/declare PDP and other stuff here

    @Log.NT
    override val powerDistribution: PowerDistribution =
        PowerDistribution(
            RobotConstants.PDH_CAN,
            PowerDistribution.ModuleType.kRev
        )

    @Log.NT
    override val drive = MecanumDrive.createMecanum(ahrs)

    @Log.NT
    val oi = createMecanumOI(drive, driveController)

    @Log.NT
    override val driveCommand = MecanumDriveCommand(drive, oi)

    val light = Light.createLight()

    val intake = Intake()

    val shooter = Shooter()

    val indexer = Indexer()

//  val infrared = DigitalInput(RobotConstants.IR_CHANNEL)
}

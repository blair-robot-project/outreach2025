package frc.robot
import edu.wpi.first.hal.FRCNetComm.tInstances
import edu.wpi.first.hal.FRCNetComm.tResourceType
import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.RobotContainer.configureBindings
import frc.robot.subsystems.indexer.Indexer
import org.littletonrobotics.urcl.URCL

/**
 * The functions in this object (which basically functions as a singleton class) are called automatically
 * corresponding to each mode, as described in the TimedRobot documentation.
 * This is written as an object rather than a class since there should only ever be a single instance, and
 * it cannot take any constructor arguments. This makes it a natural fit to be an object in Kotlin.
 *
 * If you change the name of this object or its package after creating this project, you must also update
 * the `Main.kt` file in the project. (If you use the IDE's Rename or Move refactorings when renaming the
 * object or package, it will get changed everywhere.)
 */

/** The main class of the robot, constructs all the subsystems and initializes default commands. */
class RobotLoop : TimedRobot() {

    private val robot = Robot()
    private val field = robot.field
    val indexer =  Indexer()

    private var autoCommand: Command? = null
    private var routineMap = hashMapOf<String, Command>()


    override fun robotInit() {
        // Yes this should be a print statement, it's useful to know that robotInit started.
        println("Started robotInit.")

        HAL.report(tResourceType.kResourceType_Language, tInstances.kLanguage_Kotlin)

        SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance())
        configureBindings()

        URCL.start()


    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()

        robot.field.robotPose = robot.drive.pose

        robot.field.getObject("bumpers").pose = robot.drive.pose


    }

    override fun autonomousInit() {
        /** Every time auto starts, we update the chosen auto command */
        CommandScheduler.getInstance().schedule(this.autoCommand)


    }

    override fun autonomousPeriodic() {}

    override fun teleopInit() {
        if (autoCommand != null) {
            CommandScheduler.getInstance().cancel(autoCommand)
        }

        robot.drive.defaultCommand = robot.driveCommand
    }

    override fun teleopPeriodic() {
        indexer.runIndexer()

    }

    override fun disabledInit() {
        robot.drive.stop()

    }

    override fun disabledPeriodic() {
    }

    override fun testInit() {
        if (autoCommand != null) {
            CommandScheduler.getInstance().cancel(autoCommand)
        }
    }

    override fun testPeriodic() {}

    override fun simulationInit() {}

    override fun simulationPeriodic() {
    }
}

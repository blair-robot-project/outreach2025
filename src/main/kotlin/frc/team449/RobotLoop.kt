package frc.team449

import edu.wpi.first.epilogue.Epilogue
import edu.wpi.first.epilogue.Logged
import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj.DataLogManager
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.team449.leds.BlairChasing
import frc.team449.leds.BreatheHue
import frc.team449.leds.Crazy
import frc.team449.leds.Rainbow
import org.littletonrobotics.urcl.URCL
import kotlin.jvm.optionals.getOrNull

/** The main class of the robot, constructs all the subsystems and initializes default commands. */
@Logged
class RobotLoop : TimedRobot() {
    private val robot = Robot()

    private val field = robot.field

    private var autoCommand: Command? = null
    private var routineMap = hashMapOf<String, Command>()
    private val controllerBinder = ControllerBindings(robot.driveController, robot)

    override fun robotInit() {
        println("Started robotInit.")

        HAL.report(FRCNetComm.tResourceType.kResourceType_Language, FRCNetComm.tInstances.kLanguage_Kotlin)

        if (RobotBase.isSimulation()) {
            // Don't complain about joysticks if there aren't going to be any
            DriverStation.silenceJoystickConnectionWarning(true)
        }

        println("Generating Auto Routines : ${Timer.getFPGATimestamp()}")
        println("DONE Generating Auto Routines : ${Timer.getFPGATimestamp()}")

        SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance())

        robot.light.defaultCommand = BlairChasing(robot.light)

        controllerBinder.bindButtons()

        if (RobotBase.isReal()) {
            URCL.start()
        }

        DataLogManager.start()
        Epilogue.bind(this)
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()

        robot.field.robotPose = robot.drive.pose

        robot.field.getObject("bumpers").pose = robot.drive.pose
    }

    override fun autonomousInit() {
        /** Every time auto starts, we update the chosen auto command */
        CommandScheduler.getInstance().schedule(this.autoCommand)

        if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red) {
            BreatheHue(robot.light, 0).schedule()
        } else {
            BreatheHue(robot.light, 95).schedule()
        }
    }

    override fun autonomousPeriodic() {}

    override fun teleopInit() {
        if (autoCommand != null) {
            CommandScheduler.getInstance().cancel(autoCommand)
        }

        (robot.light.currentCommand ?: InstantCommand()).cancel()

        robot.drive.defaultCommand = robot.driveCommand
        //robot.indexer.defaultCommand = robot.indexer.runIndexer()

        robot.indexer.runIndexer()
    }

    override fun teleopPeriodic() {
        Crazy(robot.light).schedule()
    }

    override fun disabledInit() {
        robot.drive.stop()

        (robot.light.currentCommand ?: InstantCommand()).cancel()
        Rainbow(robot.light).schedule()
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

    override fun simulationPeriodic() {}
}

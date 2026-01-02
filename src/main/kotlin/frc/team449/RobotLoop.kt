package frc.team449

import edu.wpi.first.epilogue.Epilogue
import edu.wpi.first.epilogue.Logged
import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator3d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.proto.Pose2dProto
import edu.wpi.first.math.geometry.proto.Pose3dProto
import edu.wpi.first.math.proto.Geometry2D
import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.ProtobufPublisher
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.team449.leds.BlairChasing
import frc.team449.leds.BreatheHue
import frc.team449.leds.Crazy
import frc.team449.leds.Rainbow
import gg.questnav.questnav.QuestNav
import gg.questnav.questnav.protos.generated.Commands.ProtobufQuestNavCommand
import org.littletonrobotics.urcl.URCL
import kotlin.jvm.optionals.getOrNull

/** The main class of the robot, constructs all the subsystems and initializes default commands. */
@Logged
class RobotLoop : TimedRobot() {
    private val robot = Robot()

    private val field = robot.field

    // VR testing
    private val vr = QuestNav()
    private val vrPoseEstimator = MecanumDrivePoseEstimator3d(
        robot.drive.kinematics,
        robot.ahrs.rotation3d,
        robot.drive.getPositions(),
        Pose3d(RobotConstants.INITIAL_POSE)
    )
    private val debugtable: NetworkTable = NetworkTableInstance.getDefault().getTable("VR Debug")

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

        // take current robot position (the center of the frame on the floor plane)
        // offset by wherever the vr headset is on the robot
        // height = (floor to bottom of driveframe + driveframe + top of frame to middle of VR)
        vr.setPose(Pose3d(robot.drive.pose)
            .transformBy(
                Transform3d(0.0, 0.0, 0.0,
                    Rotation3d(0.0, 0.0, 0.0))
            )
        )

        robot.indexer.runIndexer()
    }

    override fun teleopPeriodic() {
        Crazy(robot.light).schedule()

        // update data
        vrPoseEstimator.update(robot.ahrs.rotation3d, robot.drive.getPositions())
        vr.commandPeriodic()


//        debugtable.getProtobufTopic("NativePose", Pose2dProto.pack(robot.drive.pose).publish()
        debugtable.getProtobufTopic("VRPoseRaw", Pose3dProto()).publish()
        debugtable.getProtobufTopic("FusedPose", Pose3dProto()).publish()
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

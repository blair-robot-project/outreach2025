package frc.team449

import edu.wpi.first.epilogue.Epilogue
import edu.wpi.first.epilogue.Logged
import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator3d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.proto.Pose2dProto
import edu.wpi.first.math.geometry.proto.Pose3dProto
import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.team449.leds.BlairChasing
import frc.team449.leds.BreatheHue
import frc.team449.leds.Crazy
import frc.team449.leds.Rainbow
import gg.questnav.questnav.PoseFrame
import gg.questnav.questnav.QuestNav
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
    private val debugTable: NetworkTable = NetworkTableInstance.getDefault().getTable("VR Debug")
    private val regularPoseEntry = debugTable.getProtobufTopic("NativePose", Pose2dProto()).publish()
    private val vrPoseEntry = debugTable.getProtobufTopic("VRPoseRaw", Pose3dProto()).publish()
    private val fusionPoseEntry = debugTable.getProtobufTopic("FusedPose", Pose3dProto()).publish()
    // end of VR testing

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

        // VR Testing
        // take current robot position (the center of the frame on the floor plane)
        // offset by wherever the vr headset is on the robot
        // height = (floor to bottom of driveframe + driveframe + top of frame to middle of VR)
        vr.setPose(
            Pose3d(robot.drive.pose)
                .transformBy(
                    Transform3d(
                        0.0,
                        0.0,
                        0.0,
                        Rotation3d(0.0, 0.0, 0.0)
                    )
                )
        )
        // END of VR testing

        robot.indexer.runIndexer()
    }

    override fun teleopPeriodic() {
        Crazy(robot.light).schedule()

        // VR testing
        // update data
        vrPoseEstimator.update(robot.ahrs.rotation3d, robot.drive.getPositions())
        vr.commandPeriodic()
        val frames: Array<PoseFrame?>? = vr.getAllUnreadPoseFrames()

        // report drive estimated pose
        regularPoseEntry.set(robot.drive.pose)
        if (frames != null) {
            for (f in frames) {
                if (f != null && f.isTracking) {
                    // report raw vr pose
                    vrPoseEntry.set(f.questPose3d)

                    // fuse and report fused pose
                    vrPoseEstimator.addVisionMeasurement(
                        f.questPose3d,
                        f.dataTimestamp,
                        VecBuilder.fill(0.1, 0.1, 0.1, 0.05)
                    )
                    fusionPoseEntry.set(vrPoseEstimator.estimatedPosition)
                }
            }
        }
        // END of VR testing
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

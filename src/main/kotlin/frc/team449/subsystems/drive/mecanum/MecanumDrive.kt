package frc.team449.subsystems.drive.mecanum

import com.revrobotics.spark.SparkMax
import com.studica.frc.AHRS
import edu.wpi.first.epilogue.Logged
import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.Nat
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.MecanumDriveKinematics
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.RobotConstants
import frc.team449.subsystems.drive.HolonomicDrive
import frc.team449.system.motor.createSparkMax

/**
 * @param frontLeftMotor the front left motor
 * @param frontRightMotor the front right motor
 * @param backLeftMotor the back left motor
 * @param backRightMotor the back right motor
 * @param frontLeftLocation the offset of the front left wheel to the center of the robot
 * @param frontRightLocation the offset of the front right wheel to the center of the robot
 * @param backLeftLocation the offset of the back left wheel to the center of the robot
 * @param backRightLocation the offset of the back right wheel to the center of the robot
 * @param maxLinearSpeed the maximum translation speed of the chassis.
 * @param maxRotSpeed the maximum rotation speed of the chassis
 * @param feedForward the SimpleMotorFeedforward for mecanum
 * @param controller the PIDController for the robot
 */
@Logged
open class MecanumDrive(
    private val frontLeftMotor: SparkMax,
    private val frontRightMotor: SparkMax,
    private val backLeftMotor: SparkMax,
    private val backRightMotor: SparkMax,
    frontLeftLocation: Translation2d,
    frontRightLocation: Translation2d,
    backLeftLocation: Translation2d,
    backRightLocation: Translation2d,
    private val ahrs: AHRS,
    override var maxLinearSpeed: Double,
    override var maxRotSpeed: Double,
    private val feedForward: SimpleMotorFeedforward,
    private val controller: () -> PIDController
) : SubsystemBase(),
    HolonomicDrive {
    private val flController = controller()
    private val frController = controller()
    private val blController = controller()
    private val brController = controller()

    private var lastTime = Timer.getFPGATimestamp()

    private val kinematics =
        MecanumDriveKinematics(
            frontLeftLocation,
            frontRightLocation,
            backLeftLocation,
            backRightLocation
        )

    private val poseEstimator =
        MecanumDrivePoseEstimator(
            kinematics,
            ahrs.rotation2d,
            getPositions(),
            RobotConstants.INITIAL_POSE,
            MatBuilder.fill(Nat.N3(), Nat.N1(), .005, .005, .0005), // [x, y, theta] other estimates
            MatBuilder.fill(Nat.N3(), Nat.N1(), .005, .005, .0005) // [x, y, theta] vision estimates
        )

    override var pose: Pose2d
        get() {
            return this.poseEstimator.estimatedPosition
        }
        set(value) {
            this.poseEstimator.resetPosition(ahrs.rotation2d, getPositions(), value)
        }

    private var desiredWheelSpeeds = MecanumDriveWheelSpeeds()

    init {
        frontLeftMotor.encoder.position = 0.0
        frontRightMotor.encoder.position = 0.0
        backLeftMotor.encoder.position = 0.0
        backRightMotor.encoder.position = 0.0
    }

    override fun set(desiredSpeeds: ChassisSpeeds) {
        desiredWheelSpeeds = kinematics.toWheelSpeeds(desiredSpeeds)
        desiredWheelSpeeds.desaturate(MecanumConstants.MAX_ATTAINABLE_WHEEL_SPEED)
    }

    override fun stop() {
        this.set(ChassisSpeeds(0.0, 0.0, 0.0))
    }

    override fun periodic() {
        val currTime = Timer.getFPGATimestamp()

        val frontLeftPID =
            flController.calculate(
                frontLeftMotor.encoder.velocity,
                desiredWheelSpeeds.frontLeftMetersPerSecond
            )
        val frontRightPID =
            frController.calculate(
                frontRightMotor.encoder.velocity,
                desiredWheelSpeeds.frontRightMetersPerSecond
            )
        val backLeftPID = blController.calculate(
            backLeftMotor.encoder.velocity,
            desiredWheelSpeeds.rearLeftMetersPerSecond
        )
        val backRightPID = brController.calculate(
            backRightMotor.encoder.velocity,
            desiredWheelSpeeds.rearRightMetersPerSecond
        )

        val frontLeftFF = feedForward.calculate(desiredWheelSpeeds.frontLeftMetersPerSecond)
        val frontRightFF = feedForward.calculate(desiredWheelSpeeds.frontRightMetersPerSecond)
        val backLeftFF = feedForward.calculate(desiredWheelSpeeds.rearLeftMetersPerSecond)
        val backRightFF = feedForward.calculate(desiredWheelSpeeds.rearRightMetersPerSecond)

        frontLeftMotor.setVoltage(frontLeftPID + frontLeftFF)
        frontRightMotor.setVoltage(frontRightPID + frontRightFF)
        backLeftMotor.setVoltage(backLeftPID + backLeftFF)
        backRightMotor.setVoltage(backRightPID + backRightFF)

        this.poseEstimator.update(ahrs.rotation2d, getPositions())
        lastTime = currTime
    }

    /**
     * @return the position readings of the wheels bundled into one object (meters)
     */
    private fun getPositions(): MecanumDriveWheelPositions =
        MecanumDriveWheelPositions(
            frontLeftMotor.encoder.position,
            frontRightMotor.encoder.position,
            backLeftMotor.encoder.position,
            backRightMotor.encoder.position
        )

    /**
     * @return the velocity readings of the wheels bundled into one object (meters/s)
     */
    private fun getSpeeds(): MecanumDriveWheelSpeeds =
        MecanumDriveWheelSpeeds(
            frontLeftMotor.encoder.velocity,
            frontRightMotor.encoder.velocity,
            backLeftMotor.encoder.velocity,
            backRightMotor.encoder.velocity
        )

    companion object {
        /** Create a new Mecanum Drive from DriveConstants */
        fun createMecanum(ahrs: AHRS): MecanumDrive =
            MecanumDrive(
                createSparkMax(
                    MecanumConstants.DRIVE_MOTOR_FL,
                    false,
                    gearing = MecanumConstants.DRIVE_GEARING,
                    upr = MecanumConstants.DRIVE_UPR,
                    currentLimit = MecanumConstants.CURRENT_LIM
                ),
                createSparkMax(
                    MecanumConstants.DRIVE_MOTOR_FR,
                    true,
                    gearing = MecanumConstants.DRIVE_GEARING,
                    upr = MecanumConstants.DRIVE_UPR,
                    currentLimit = MecanumConstants.CURRENT_LIM
                ),
                createSparkMax(
                    MecanumConstants.DRIVE_MOTOR_BL,
                    false,
                    gearing = MecanumConstants.DRIVE_GEARING,
                    upr = MecanumConstants.DRIVE_UPR,
                    currentLimit = MecanumConstants.CURRENT_LIM
                ),
                createSparkMax(
                    MecanumConstants.DRIVE_MOTOR_BR,
                    true,
                    gearing = MecanumConstants.DRIVE_GEARING,
                    upr = MecanumConstants.DRIVE_UPR,
                    currentLimit = MecanumConstants.CURRENT_LIM
                ),
                Translation2d(MecanumConstants.WHEEL_BASE / 2, MecanumConstants.TRACK_WIDTH / 2),
                Translation2d(MecanumConstants.WHEEL_BASE / 2, -MecanumConstants.TRACK_WIDTH / 2),
                Translation2d(-MecanumConstants.WHEEL_BASE / 2, MecanumConstants.TRACK_WIDTH / 2),
                Translation2d(-MecanumConstants.WHEEL_BASE / 2, -MecanumConstants.TRACK_WIDTH / 2),
                ahrs,
                RobotConstants.MAX_LINEAR_SPEED,
                RobotConstants.MAX_ROT_SPEED,
                SimpleMotorFeedforward(MecanumConstants.DRIVE_KS, MecanumConstants.DRIVE_KV, MecanumConstants.DRIVE_KA),
                { PIDController(MecanumConstants.DRIVE_KP, MecanumConstants.DRIVE_KI, MecanumConstants.DRIVE_KD) }
            )
    }
}

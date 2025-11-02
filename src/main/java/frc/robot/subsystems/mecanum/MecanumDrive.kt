package frc.team449.control.holonomic.mecanum

import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import com.revrobotics.spark.config.SparkMaxConfig
import com.studica.frc.AHRS
import dev.doglog.DogLog
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
import frc.robot.Constants
import frc.robot.subsystems.mecanum.MecanumConstants
import edu.wpi.first.epilogue.Logged


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
open class MecanumDrive(
    private val frontLeftMotor: SparkMax,
    private val frontRightMotor: SparkMax,
    private val backLeftMotor: SparkMax,
    private val backRightMotor: SparkMax,
    private val ahrs: AHRS,
    frontLeftLocation: Translation2d,
    frontRightLocation: Translation2d,
    backLeftLocation: Translation2d,
    backRightLocation: Translation2d,
    var maxLinearSpeed: Double,
    var maxRotSpeed: Double,
    private val feedForward: SimpleMotorFeedforward,
    private val controller: () -> PIDController,
) : SubsystemBase() {

    var configL = SparkMaxConfig()
    var configR = SparkMaxConfig()

    init {
        configL.smartCurrentLimit(MecanumConstants.CURRENT_LIM)
        configL.inverted(false)
        configL.idleMode(IdleMode.kCoast)

        configR.smartCurrentLimit(MecanumConstants.CURRENT_LIM)
        configR.inverted(true)
        configR.idleMode(IdleMode.kCoast)

        frontLeftMotor.configure(configL,SparkBase.ResetMode.kResetSafeParameters,SparkBase.PersistMode.kPersistParameters)
        frontRightMotor.configure(configR,SparkBase.ResetMode.kResetSafeParameters,SparkBase.PersistMode.kPersistParameters)
        backLeftMotor.configure(configL,SparkBase.ResetMode.kResetSafeParameters,SparkBase.PersistMode.kPersistParameters)
        backRightMotor.configure(configR,SparkBase.ResetMode.kResetSafeParameters,SparkBase.PersistMode.kPersistParameters)
    }



    private val flController = controller()
    private val frController = controller()
    private val blController = controller()
    private val brController = controller()

    private var lastTime = Timer.getFPGATimestamp()

    val kinematics = MecanumDriveKinematics(
        frontLeftLocation,
        frontRightLocation,
        backLeftLocation,
        backRightLocation
    )

    private val poseEstimator = MecanumDrivePoseEstimator(
        kinematics,
        ahrs.rotation2d,
        getPositions(),
        Constants.RobotConstants.INITIAL_POSE,
        MatBuilder.fill(Nat.N3(), Nat.N1(), .005, .005, .0005), // [x, y, theta] other estimates
        MatBuilder.fill(Nat.N3(), Nat.N1(), .005, .005, .0005) // [x, y, theta] vision estimates
    )

     var pose: Pose2d
        get() { return this.poseEstimator.estimatedPosition }
        set(value) { this.poseEstimator.resetPosition(ahrs.rotation2d, getPositions(), value) }

    private var desiredWheelSpeeds = MecanumDriveWheelSpeeds()

     fun set(desiredSpeeds: ChassisSpeeds) {
        desiredWheelSpeeds = kinematics.toWheelSpeeds(desiredSpeeds)
        desiredWheelSpeeds.desaturate(MecanumConstants.MAX_ATTAINABLE_WHEEL_SPEED)
    }

     fun stop() {
         this.set(ChassisSpeeds(0.0, 0.0, 0.0)) }

    private fun logData() {
        DogLog.log("FrontLeft/velocity", frontLeftMotor.encoder.velocity)
        DogLog.log("FrontLeft/position", frontLeftMotor.encoder.position)
        DogLog.log("FrontRight/velocity", frontRightMotor.encoder.velocity)
        DogLog.log("FrontRight/position", frontRightMotor.encoder.position)
        DogLog.log("BackLeft/velocity", backLeftMotor.encoder.velocity)
        DogLog.log("BackLeft/position", backLeftMotor.encoder.position)
        DogLog.log("BackRight/velocity", backRightMotor.encoder.velocity)
        DogLog.log("BackRight/position", backRightMotor.encoder.position)

        DogLog.log("FR", frontRightMotor.appliedOutput)
        DogLog.log("BL", backLeftMotor.appliedOutput)
        DogLog.log("BR", backRightMotor.appliedOutput)
        DogLog.log("FL", frontLeftMotor.appliedOutput)

        DogLog.log("desiredSpeed", desiredWheelSpeeds)
        DogLog.log("pose2d", pose)
        DogLog.log("ahrs", ahrs.rotation2d)
    }

    override fun periodic() {
        logData()
        val currTime = Timer.getFPGATimestamp()

        val frontLeftPID = flController.calculate(frontLeftMotor.encoder.velocity, desiredWheelSpeeds.frontLeftMetersPerSecond)
        val frontRightPID = frController.calculate(frontRightMotor.encoder.velocity, desiredWheelSpeeds.frontRightMetersPerSecond)
        val backLeftPID = blController.calculate(backLeftMotor.encoder.velocity, desiredWheelSpeeds.rearLeftMetersPerSecond)
        val backRightPID = brController.calculate(backRightMotor.encoder.velocity, desiredWheelSpeeds.rearRightMetersPerSecond)

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
        fun createMecanum(ahrs: AHRS): MecanumDrive {
            return MecanumDrive(
                frontLeftMotor = SparkMax(MecanumConstants.DRIVE_MOTOR_FL,SparkLowLevel.MotorType.kBrushless) ,
                frontRightMotor = SparkMax(MecanumConstants.DRIVE_MOTOR_FR,SparkLowLevel.MotorType.kBrushless),
                backLeftMotor = SparkMax(MecanumConstants.DRIVE_MOTOR_BL,SparkLowLevel.MotorType.kBrushless),
                backRightMotor = SparkMax(MecanumConstants.DRIVE_MOTOR_BR,SparkLowLevel.MotorType.kBrushless),
                ahrs,
                Translation2d(MecanumConstants.WHEELBASE / 2, MecanumConstants.TRACKWIDTH / 2),
                Translation2d(MecanumConstants.WHEELBASE / 2, -MecanumConstants.TRACKWIDTH / 2),
                Translation2d(-MecanumConstants.WHEELBASE / 2, MecanumConstants.TRACKWIDTH / 2),
                Translation2d(-MecanumConstants.WHEELBASE / 2, -MecanumConstants.TRACKWIDTH / 2),
                Constants.RobotConstants.MAX_LINEAR_SPEED,
                Constants.RobotConstants.MAX_ROT_SPEED,
                SimpleMotorFeedforward(MecanumConstants.DRIVE_KS, MecanumConstants.DRIVE_KV, MecanumConstants.DRIVE_KA),
                { PIDController(MecanumConstants.DRIVE_KP, MecanumConstants.DRIVE_KI, MecanumConstants.DRIVE_KD) },
            )
        }
    }
}

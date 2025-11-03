package frc.team449.system.motor

import com.revrobotics.REVLibError
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import frc.team449.system.encoder.EncoderCreator

/**
 * Create a Spark Max with the given configurations
 *
 * @param name The motor's name
 * @param id The motor's CAN ID
 */

// TODO: Test if disabling voltage compensation helps reduce brownouts
fun createSparkMax(
    name: String,
    id: Int,
    encCreator: EncoderCreator<SparkMax>,
    enableBrakeMode: Boolean = true,
    inverted: Boolean = false,
    currentLimit: Int = 0,
    enableVoltageComp: Boolean = false,
    slaveSparks: Map<Int, Boolean> = mapOf(),
    controlFrameRateMillis: Int = -1,
    statusFrameRatesMillis: Map<SparkLowLevel.PeriodicFrame, Int> = mapOf()
): WrappedMotor {
    val motor =
        SparkMax(
            id,
            SparkLowLevel.MotorType.kBrushless
        )
    if (motor.lastError != REVLibError.kOk) {
        println(
            "Motor could not be constructed on port " +
                id +
                " due to error " +
                motor.lastError
        )
    }

//  motor.restoreFactoryDefaults()

    val enc = encCreator.create(name + "Enc", motor, inverted)

//  val brakeMode =
//    if (enableBrakeMode) {
//      SparkBase.IdleMode.kBrake
//    } else {
//      CANSparkBase.IdleMode.kCoast
//    }

//    motor.inverted = inverted
    // Set brake mode
    // motor.idleMode = brakeMode

    // Set frame rates
    if (controlFrameRateMillis >= 1) {
        // Must be between 1 and 100 ms.
        motor.setControlFramePeriodMs(controlFrameRateMillis)
    }
//
//  for ((statusFrame, period) in statusFrameRatesMillis) {
//    motor.setPeriodicFramePeriod(statusFrame, period)
//  }

//  // Set the current limit if it was given
//  if (currentLimit > 0) {
//    motor.setSmartCurrentLimit(currentLimit)
//  }
//
//  if (enableVoltageComp) {
//    motor.enableVoltageCompensation(RobotController.getBatteryVoltage())
//  } else {
//    motor.disableVoltageCompensation()
//  }

//  for ((slavePort, slaveInverted) in slaveSparks) {
//    val slave = createFollowerSpark(slavePort)
//    slave.restoreFactoryDefaults()
//    slave.follow(motor, slaveInverted)
//    slave.idleMode = brakeMode
//    if (currentLimit > 0) {
//      slave.setSmartCurrentLimit(currentLimit)
//    }
//    slave.burnFlash()
//  }
//
//  motor.burnFlash()

    return WrappedMotor(
        motor,
        enc,
        { motor.get() },
        { motor.appliedOutput },
        { motor.busVoltage },
        { motor.outputCurrent }
    )
}

/**
 * Create a Spark that will follow another Spark
 *
// * @param port The follower's CAN ID
// */
// private fun createFollowerSpark(port: Int): SparkMax {
//  val follower = SparkMax(
//    port,
//    SparkLowLevel.MotorType.kBrushless
//  )
//
//  follower
//    .getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen)
//    .enableLimitSwitch(false)
//  follower
//    .getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen)
//    .enableLimitSwitch(false)
//
//  follower.setPeriodicFramePeriod(
//    SparkLowLevel.PeriodicFrame.kStatus0,
//    100
//  )
//  follower.setPeriodicFramePeriod(
//    SparkLowLevel.PeriodicFrame.kStatus1,
//    100
//  )
//  follower.setPeriodicFramePeriod(
//    SparkLowLevel.PeriodicFrame.kStatus2,
//    100
//  )
//
//  return follower
//
// }

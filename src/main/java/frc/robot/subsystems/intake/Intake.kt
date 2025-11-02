package frc.robot.subsystems.intake

import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkLowLevel.MotorType
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import com.revrobotics.spark.config.SparkMaxConfigAccessor
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax
import edu.wpi.first.wpilibj.motorcontrol.Spark
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.SubsystemBase

class Intake() : SubsystemBase() {
    val motor = SparkMax(IntakeConstants.INTAKE_ID, MotorType.kBrushless)
    val config = SparkMaxConfig()

    init {
        config.smartCurrentLimit(IntakeConstants.CURRENT_LIMIT)
        config.inverted(true)
        config.idleMode(SparkBaseConfig.IdleMode.kCoast)
        motor.configure(config,SparkBase.ResetMode.kResetSafeParameters,SparkBase.PersistMode.kPersistParameters)

    }


    fun runIntake(): Command{
        return runOnce {
            motor.setVoltage(IntakeConstants.INTAKE_VOLT)
        }
    }


    fun runIntakeReversed(): Command{
        return runOnce {
            motor.setVoltage(-IntakeConstants.INTAKE_VOLT)
        }
    }


    fun stopIntake(): Command{
        return runOnce {
            motor.stopMotor()
        }
    }



}
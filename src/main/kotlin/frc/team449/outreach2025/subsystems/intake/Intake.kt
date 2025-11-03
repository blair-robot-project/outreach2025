package frc.team449.outreach2025.subsystems.intake

import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase

class Intake : SubsystemBase() {
    val motor = SparkMax(IntakeConstants.INTAKE_ID, SparkLowLevel.MotorType.kBrushless)
    val config = SparkMaxConfig()

    init {
        config.smartCurrentLimit(IntakeConstants.CURRENT_LIMIT)
        config.inverted(true)
        config.idleMode(SparkBaseConfig.IdleMode.kCoast)
        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
    }

    fun runIntake(): Command =
        runOnce {
            motor.setVoltage(IntakeConstants.INTAKE_VOLT)
        }

    fun runIntakeReversed(): Command =
        runOnce {
            motor.setVoltage(-IntakeConstants.INTAKE_VOLT)
        }

    fun stopIntake(): Command = runOnce { motor.stopMotor() }
}

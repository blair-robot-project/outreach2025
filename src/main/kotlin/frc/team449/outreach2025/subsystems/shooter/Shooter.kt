package frc.team449.outreach2025.subsystems.shooter

import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase

class Shooter : SubsystemBase() {
    val motor = SparkMax(ShooterConstants.SHOOTER_ID, SparkLowLevel.MotorType.kBrushless)
    val config = SparkMaxConfig()

    init {
        config.smartCurrentLimit(ShooterConstants.CURRENT_LIMIT)
        config.idleMode(SparkBaseConfig.IdleMode.kCoast)
        config.inverted(true)
        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
    }

    fun runShooter(): Command =
        runOnce {
            motor.setVoltage(ShooterConstants.SHOOTER_VOLT)
        }

    fun runShooterReversed(): Command =
        runOnce {
            motor.setVoltage(-ShooterConstants.SHOOTER_VOLT)
        }

    fun stopShooter(): Command =
        runOnce {
            motor.stopMotor()
        }
}

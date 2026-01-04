package frc.team449.subsystems.shooter

import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.WaitCommand

class Shooter : SubsystemBase() {
    val shooter = SparkMax(ShooterConstants.SHOOTER_ID, SparkLowLevel.MotorType.kBrushless)
    val feeder = SparkMax(ShooterConstants.FEEDER_ID, SparkLowLevel.MotorType.kBrushless)
    val shooterConfig = SparkMaxConfig()
    val feederConfig = SparkMaxConfig()

    init {
        shooterConfig.smartCurrentLimit(ShooterConstants.CURRENT_LIMIT)
        shooterConfig.idleMode(SparkBaseConfig.IdleMode.kCoast)
        shooterConfig.inverted(true)

        shooter.configure(shooterConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)

        feederConfig.smartCurrentLimit(ShooterConstants.CURRENT_LIMIT)
        feederConfig.idleMode(SparkBaseConfig.IdleMode.kCoast)

        feederConfig.inverted(false)

        feeder.configure(feederConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
    }

    fun runShooter(): Command =
        runOnce {
            shooter.setVoltage(ShooterConstants.SHOOTER_VOLT)
        }.andThen(WaitCommand(1.0))
            .andThen(
                runOnce {
                    feeder.setVoltage(ShooterConstants.SHOOTER_VOLT)
                }
            )

    fun runShooterReversed(): Command =
        runOnce {
            shooter.setVoltage(-ShooterConstants.SHOOTER_VOLT)
        }.andThen(WaitCommand(1.0))
            .andThen(
                runOnce {
                    feeder.setVoltage(-ShooterConstants.SHOOTER_VOLT)
                }
            )

    fun stopShooter(): Command =
        runOnce {
            shooter.stopMotor()
        }.andThen(
            runOnce {
                feeder.stopMotor()
            }
        )
}

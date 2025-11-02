package frc.robot.subsystems.shooter
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import dev.doglog.DogLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase

class Shooter(): SubsystemBase() {
    val motor = SparkMax(ShooterConstants.SHOOTER_ID,SparkLowLevel.MotorType.kBrushless)
    val config = SparkMaxConfig()

    init {
        config.smartCurrentLimit(ShooterConstants.CURRENT_LIMIT)
        config.idleMode(SparkBaseConfig.IdleMode.kCoast)
        config.inverted(true)
        motor.configure(config,SparkBase.ResetMode.kResetSafeParameters,SparkBase.PersistMode.kPersistParameters)
    }

    fun runShooter(): Command {
        return runOnce {
            motor.setVoltage(ShooterConstants.SHOOTER_VOLT)
        }
    }


    fun runShooterReversed(): Command {
        return runOnce {
            motor.setVoltage(-ShooterConstants.SHOOTER_VOLT)
        }
    }


    fun stopShooter(): Command {
        return runOnce {
            motor.stopMotor()
        }
    }

    override fun periodic() {
        logData()
    }

    private fun logData(){
        DogLog.log("Shooter motor", motor.appliedOutput)
    }


}
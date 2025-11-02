package frc.robot.subsystems.indexer

import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase

class Indexer():SubsystemBase() {
    val motor = SparkMax(IndexerConstants.INDEXER_ID,SparkLowLevel.MotorType.kBrushless)
    val config = SparkMaxConfig()

    init {
        config.smartCurrentLimit(IndexerConstants.CURRENT_LIMIT)
        config.inverted(false)
        config.idleMode(SparkBaseConfig.IdleMode.kCoast)
        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
    }

    fun runIndexer():Command{
       return  runOnce {
            motor.setVoltage(IndexerConstants.INDEXER_VOLTS)
        }
    }

}
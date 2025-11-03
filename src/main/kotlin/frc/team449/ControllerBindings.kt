package frc.team449

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.team449.outreach2025.constants.RobotConstants
import kotlin.math.PI

class ControllerBindings(
    private val driveController: CommandXboxController,
    private val robot: Robot
) {
    private fun evergreenBindings() {
        // slow drive
        Trigger { driveController.rightTriggerAxis >= .75 }
            .onTrue(
                InstantCommand({
                    robot.drive.maxLinearSpeed = 5.66
                    robot.drive.maxRotSpeed = PI
                })
            ).onFalse(
                InstantCommand({
                    robot.drive.maxLinearSpeed = RobotConstants.MAX_LINEAR_SPEED
                    robot.drive.maxRotSpeed = RobotConstants.MAX_ROT_SPEED
                })
            )

        // reset gyro
        driveController.povDown().onTrue(
            InstantCommand({ robot.drive.heading = Rotation2d() })
        )
    }

    fun bindButtons() {
        evergreenBindings()
        driveController.y().onTrue(robot.shooter.runShooter()).onFalse(robot.shooter.stopShooter())
        driveController.x().onTrue(robot.intake.runIntake()).onFalse(robot.intake.stopIntake())
    }
}

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.SwerveCommands;
import frc.robot.Commands.SwerveCommands.DriveToPosition;
import frc.robot.Subsystems.Swerve.GyroIOPigeon;
import frc.robot.Subsystems.Swerve.ModuleIOReal;
import frc.robot.Subsystems.Swerve.Swerve;

public class RobotContainer {
    private Swerve swerve = new Swerve(new GyroIOPigeon(), new ModuleIOReal(0), new ModuleIOReal(1),
            new ModuleIOReal(2), new ModuleIOReal(3));
    private DriveToPosition driveToPosition = new DriveToPosition(swerve, new Pose2d(5, 0, null));

    private XboxController driverController = new XboxController(0);

    public RobotContainer() {
        swerve.setDefaultCommand(
                SwerveCommands.joystickDrive(swerve, driverController::getLeftY, driverController::getLeftX,
                        () -> driverController.getLeftTriggerAxis() - driverController.getRightTriggerAxis()));
        configureBindings();
    }

    private void configureBindings() {
        new Trigger(driverController::getAButton).onTrue(SwerveCommands.goToAngle(swerve, new Rotation2d(Math.PI)));
        new Trigger(driverController::getXButton)
                .onTrue(SwerveCommands.goToAngle(swerve, new Rotation2d(Math.PI / 2.0)));
        new Trigger(driverController::getYButton).onTrue(swerve.resetGyroCommand());
        new Trigger(driverController::getAButton).onTrue(driveToPosition);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}

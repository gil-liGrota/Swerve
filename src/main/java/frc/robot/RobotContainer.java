package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Commands.SwerveCommands;
import frc.robot.Subsystems.Swerve.GyroIOPigeon;
import frc.robot.Subsystems.Swerve.ModuleIOReal;
import frc.robot.Subsystems.Swerve.Swerve;

public class RobotContainer {
    private Swerve swerve = new Swerve(new GyroIOPigeon(), new ModuleIOReal(0), new ModuleIOReal(1),
            new ModuleIOReal(2), new ModuleIOReal(3));

    private XboxController driverController = new XboxController(0);

    public RobotContainer() {
        swerve.setDefaultCommand(
                SwerveCommands.joystickDrive(swerve, driverController::getLeftY, driverController::getLeftX,
                        () -> driverController.getLeftTriggerAxis() - driverController.getRightTriggerAxis()));
        configureBindings();
    }

    private void configureBindings() {
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}

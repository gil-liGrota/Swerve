package frc.robot;

import static frc.robot.Subsystems.vision.VisionConstants.camera0Name;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.SwerveCommands;
import frc.robot.Commands.SwerveCommands.DriveToPosition;
import frc.robot.Subsystems.vision.VisionConstants;
import frc.robot.Subsystems.Swerve.GyroIOPigeon;
import frc.robot.Subsystems.Swerve.ModuleIOReal;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.vision.Vision;
import frc.robot.Subsystems.vision.VisionIOPhotonVision;

public class RobotContainer {
    private final LoggedDashboardChooser<Command> autoChooser;
    private Swerve swerve = new Swerve(new GyroIOPigeon(), new ModuleIOReal(0), new ModuleIOReal(1),
            new ModuleIOReal(2), new ModuleIOReal(3));
    private DriveToPosition driveToPosition = new DriveToPosition(swerve, new Pose2d(5, 0, null));

    private XboxController driverController = new XboxController(0);

    private Vision vision = new Vision(swerve::addVisionMeasurement,
            new VisionIOPhotonVision(camera0Name, VisionConstants.robotToCamera0));

    public RobotContainer() {
        swerve.setDefaultCommand(
                SwerveCommands.joystickDrive(swerve, driverController::getLeftY, driverController::getLeftX,
                        () -> driverController.getLeftTriggerAxis() - driverController.getRightTriggerAxis()));
        configureBindings();
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        autoChooser.addDefaultOption("none", null);
    }

    private void configureBindings() {
        new Trigger(driverController::getAButton).onTrue(SwerveCommands.goToAngle(swerve, new Rotation2d(Math.PI)));
        new Trigger(driverController::getXButton)
                .onTrue(SwerveCommands.goToAngle(swerve, new Rotation2d(Math.PI / 2.0)));
        new Trigger(driverController::getYButton).onTrue(swerve.resetGyroCommand());
        // new Trigger(driverController::getAButton).onTrue(driveToPosition);
    }

    public Command getAutonomousCommand() {
        // return Commands.print("No autonomous command configured");
        // return new PathPlannerAuto("Example auto");
        return autoChooser.get();
    }

    public Command runPath() {
        try {
            // Load the path you want to follow using its name in the GUI
            PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");

            // Create a path following command using AutoBuilder. This will also trigger
            // event markers.
            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }
}

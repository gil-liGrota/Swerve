package frc.robot.Subsystems.Swerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import static frc.robot.Subsystems.Swerve.DriveConstants.*;

public class Module {
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final int index;

    private final Alert driveDisconnectedAlert;
    private final Alert turnDisconnectedAlert;
    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

    public Module(ModuleIO io, int index) {
        this.io = io;
        this.index = index;
        driveDisconnectedAlert = new Alert(
                "Disconnected drive motor on module " + Integer.toString(index) + ".",
                AlertType.kError);
        turnDisconnectedAlert = new Alert(
                "Disconnected turn motor on module " + Integer.toString(index) + ".", AlertType.kError);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/" + getModuleString(), inputs);

        // Calculate positions for odometry
        int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
        odometryPositions = new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
            double positionMeters = inputs.odometryDrivePositionsRad[i] * wheelRadiusMeters;
            Rotation2d angle = inputs.odometryTurnPositions[i];
            odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }

        // Update alerts
        driveDisconnectedAlert.set(!inputs.driveConnected);
        turnDisconnectedAlert.set(!inputs.turnConnected);
    }

    /**
     * Runs the module with the specified setpoint state. Mutates the state to
     * optimize it.
     */
    public void runSetpoint(SwerveModuleState state, boolean isOpenLoop) {
        state.optimize(getAngle());
        state.cosineScale(inputs.turnPosition);

        // Apply setpoints
        if (isOpenLoop) {
            io.setDriveOpenLoop(state.speedMetersPerSecond / wheelRadiusMeters);
        } else {
            io.setDriveVelocity(state.speedMetersPerSecond / wheelRadiusMeters);
        }
        io.setTurnPosition(state.angle);
    }

    public Rotation2d getAngle() {
        return inputs.turnPosition;
    }

    public void runSteerCharacterization(double output) {
        io.setDriveOpenLoop(0);
        io.setTurnOpenLoop(output);
    }

    public void stop() {
        io.setDriveOpenLoop(0.0);
        io.setTurnOpenLoop(0.0);
    }

    public void runCharacterization(double output) {
        io.setDriveOpenLoop(output);
        io.setTurnPosition(new Rotation2d());
    }

    public double getPositionMeters() {
        return inputs.drivePositionRad * wheelRadiusMeters;
    }

    public double getVelocityMetersPerSec() {
        return inputs.driveVelocityRadPerSec * wheelRadiusMeters;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }

    public SwerveModulePosition[] getOdometryPositions() {
        return odometryPositions;
    }

    public double[] getOdometryTimestamps() {
        return inputs.odometryTimestamps;
    }

    public double getWheelRadiusCharacterizationPosition() {
        return inputs.drivePositionRad;
    }

    public double getFFCharacterizationVelocity() {
        return inputs.driveVelocityRadPerSec;
    }

    private String getModuleString() {
        return "Module " + switch (index) {
            case 0 -> "FL";
            case 1 -> "FR";
            case 2 -> "BL";
            case 3 -> "BR";
            default -> "Unknown";
        };
    }
}
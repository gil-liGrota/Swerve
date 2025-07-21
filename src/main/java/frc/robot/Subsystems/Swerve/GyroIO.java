package frc.robot.Subsystems.Swerve;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;
        public Rotation2d yawPosition = new Rotation2d();
        public double yawVelocityRadPerSec = 0.0;
        public double[] odometryYawTimestams = new double[] {};
        public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    }

    public default void updateInputs(GyroIOInputs inputs) {
    }

    public default Rotation2d getGyroRotation() {
        return new Rotation2d();
    }

    public default AngularVelocity getGyroAngularVelocity() {
        return null;
    }

    public default void reset() {
    }

    public default void reset(Rotation2d to) {
    }

}

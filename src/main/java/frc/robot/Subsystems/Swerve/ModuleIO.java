package frc.robot.Subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.logfields.IOBase;
import frc.lib.logfields.LogFieldsTable;

public interface ModuleIO{

    @AutoLog
    public static class ModulIOInputs {

        //drive
        public boolean driveConnected = false;
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;

        //turn
        public boolean turnConnected = false;
        public Rotation2d turnPosition = new Rotation2d();
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0;
        public double absolutePosition = 0.0;
    
        //odometry
        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositionsRad = new double[] {};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
    }

    public default void updateInputs(ModulIOInputs inputs){}
    public default void setDriveOpenLoop(double output){}
    public default void setTurnOpenLoop(double output){}
    public default void setDriveVelocity(double velocityRadPerSec){}
    public default void setTurnPosition(Rotation2d rotation){}

}

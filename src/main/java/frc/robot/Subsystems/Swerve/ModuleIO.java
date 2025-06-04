package frc.robot.Subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.logfields.IOBase;
import frc.lib.logfields.LogFieldsTable;

public abstract class ModuleIO extends IOBase {

    //public double[] odometryTimestamps = new double[] {};
    // public double[] odometryDrivePositionsRad = new double[] {};
    //public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};


    BooleanSupplier driveConnected = fields.addBoolean("drive Connected", this::getDriveConnected);
    DoubleSupplier drivePositionRad = fields.addDouble("drive Position Rad", this::getDrivePositionRad);
    DoubleSupplier driveVelocityRadPerSec = fields.addDouble("drive Velocity Rad Per Sec", this::getDriveVelocityRadPerSec);
    DoubleSupplier driveAppliedVolts = fields.addDouble("drive Applied Volts", this::getDriveAppliedVolts);
    DoubleSupplier driveCurrentAmps = fields.addDouble("drive Current Amps", this::getDriveCurrentAmps);
    
    BooleanSupplier turnConnected = fields.addBoolean("turn Connected", this::getTurnConnected);
    Supplier<Rotation2d> turnPosition = fields.addObject("turn Positions", this::turnPosition, new Rotation2d());
    DoubleSupplier turnVelocityRadPerSec = fields.addDouble("turn Velocity Rad Per Sec", this::getTurnVelocityRadPerSec);
    DoubleSupplier turnAppliedVolts = fields.addDouble("turn Applied Volts", this::getTurnAppliedVolts);
    DoubleSupplier turnCurrentAmps = fields.addDouble("turn Current Amps", this::getTurnCurrentAmps);
    DoubleSupplier absolutePosition = fields.addDouble("absolute Position", this::getAbsolutePosition);
    
    Supplier<double[]> odometryTimestamps = fields.addDoubleArray("odometry Timestamps", getOdometryTimestamps());
    Supplier<double[]> odometryDrivePositionsRad = fields.addDoubleArray("odometryDrivePositionsRad", getodometryDrivePositionsRad());
    Supplier<Rotation2d[]> odometryTurnPositions = fields.addObjectArray("odometry Turn Positions", getOdometryTurnPositions() , new Rotation2d[]{});

    public ModuleIO(LogFieldsTable fieldsTable){
        super(fieldsTable);
    }
    
    //get:
    
    public abstract boolean getDriveConnected();
    public abstract double getDrivePositionRad();
    public abstract double getDriveVelocityRadPerSec();
    public abstract double getDriveAppliedVolts();
    public abstract double getDriveCurrentAmps();
    
    public abstract boolean getTurnConnected();
    public abstract Rotation2d turnPosition();
    public abstract double getTurnVelocityRadPerSec();
    public abstract double getTurnAppliedVolts();
    public abstract double getTurnCurrentAmps();
    public abstract double getAbsolutePosition();
    
    public abstract Supplier<double[]> getOdometryTimestamps();
    public abstract Supplier<double[]> getodometryDrivePositionsRad();
    public abstract Supplier<Rotation2d[]> getOdometryTurnPositions();


    
    //set:

    public abstract void setDriveOpenLoop(double output);
    public abstract void setTurnOpenLoop(double output);
    public abstract void setDriveVelocity(double velocityRadPerSec);
    public abstract void setTurnPosition(Rotation2d rotation);

}

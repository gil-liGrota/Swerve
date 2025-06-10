package frc.robot.Subsystems.Swerve;

import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import java.util.Queue;
import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;

public class ModuleIOReal implements ModuleIO{
    
    private final Rotation2d zeroRotation;

    // Hardware objects
    private final SparkBase driveSpark;
    private final SparkBase turnSpark;
    private final RelativeEncoder driveEncoder;
    private final AbsoluteEncoder turnEncoder;

    // Close loop controllers
    private final SparkClosedLoopController driveController;
    private final SparkClosedLoopController turnController;

    // Queue inputs from odometry thread
    private final Queue<Double> timestampQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> turnPositionQueue;

    // Connection debouncers
    private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnConnectedDebounce = new Debouncer(0.5);


    public ModuleIOReal(int module){
        zeroRotation = switch(module){
            case 0 -> DriveConstants.frontLeftZeroRotation;
            case 1 -> DriveConstants.frontRightZeroRotation;
            case 2 -> DriveConstants.backLeftZeroRotation;
            case 3 -> DriveConstants.backRightZeroRotation;
            default -> new Rotation2d();
        };

        driveSpark = new SparkFlex(DriveConstants.swerveBaseID + DriveConstants.swerveModuleIDsCount * module, MotorType.kBrushless);
        turnSpark = new SparkMax(DriveConstants.swerveBaseID + 1 + DriveConstants.swerveModuleIDsCount * module, MotorType.kBrushless);
        driveEncoder = driveSpark.getEncoder();
        turnEncoder = turnSpark.getAbsoluteEncoder();
        driveController = driveSpark.getClosedLoopController();
        turnController = turnSpark.getClosedLoopController();

        //config drive motor

        var driveConfig = new SparkFlexConfig();
        driveConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(DriveConstants.driveMotorCurrentLimit)
                    .voltageCompensation(12.0);

        driveConfig.encoder
                    .positionConversionFactor(DriveConstants.driveEncoderPositionFactor)
                    .velocityConversionFactor(DriveConstants.driveEncoderVelocityFactor)
                    .uvwMeasurementPeriod(10)
                    .uvwAverageDepth(2);

        driveConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pidf(0.0, 0.0, 0.0, 0.0);

        driveConfig.signals
                    .primaryEncoderPositionAlwaysOn(true)
                    .primaryEncoderPositionPeriodMs((int) (1000.0 / DriveConstants.odometryFrequency))
                    .primaryEncoderVelocityAlwaysOn(true)
                    .primaryEncoderVelocityPeriodMs(20)
                    .appliedOutputPeriodMs(20)
                    .busVoltagePeriodMs(20)
                    .outputCurrentPeriodMs(20);

        tryUntilOk(
                    driveSpark,
                    5,
                    () -> driveSpark.configure(
                        driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        tryUntilOk(driveSpark, 5, () -> driveEncoder.setPosition(0.0));


        //config turn motor

        var turnConfig = new SparkMaxConfig();
        turnConfig
                    .inverted(DriveConstants.turnInverted)
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(DriveConstants.turnMotorCurrentLimit)
                    .voltageCompensation(12.0);

        turnConfig.absoluteEncoder
                    .inverted(DriveConstants.turnEncoderInverted)
                    .positionConversionFactor(DriveConstants.turnEncoderPositionFactor)
                    .velocityConversionFactor(DriveConstants.turnEncoderVelocityFactor)
                    .averageDepth(2);

        turnConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(DriveConstants.turnPIDMinInput, DriveConstants.turnPIDMaxInput)
                    .pidf(DriveConstants.turnKp, 0.0, DriveConstants.turnKd, 0.0);

        turnConfig.signals
                    .absoluteEncoderPositionAlwaysOn(true)
                    .absoluteEncoderPositionPeriodMs((int) (1000.0 / DriveConstants.odometryFrequency))
                    .absoluteEncoderVelocityAlwaysOn(true)
                    .absoluteEncoderVelocityPeriodMs(20)
                    .appliedOutputPeriodMs(20)
                    .busVoltagePeriodMs(20)
                    .outputCurrentPeriodMs(20);

        tryUntilOk(
                    turnSpark,
                    5,
                    () -> turnSpark.configure(
                            turnConfig, ResetMode.kResetSafeParameters,
                            PersistMode.kPersistParameters));
                            
        
        // create odomtry queues
        timestampQueue = OdometryThread.getInstance().makeTimestampQueue();
        drivePositionQueue = OdometryThread.getInstance().registerSignal(driveSpark, driveEncoder::getPosition);
        turnPositionQueue = OdometryThread.getInstance().registerSignal(turnSpark, turnEncoder::getPosition);
        
    }

    @Override
    public void updateInputs(ModulIOInputs inputs) {
        // update drive inputs
        sparkStickyFault = false;
        ifOk(driveSpark, driveEncoder::getPosition, (value) -> inputs.drivePositionRad = value);
        ifOk(driveSpark, driveEncoder::getVelocity, (value) -> inputs.driveVelocityRadPerSec = value);
        ifOk(
            driveSpark, 
            new DoubleSupplier[] {driveSpark::getAppliedOutput ,driveSpark::getBusVoltage }, 
            (value) -> inputs.driveAppliedVolts = value[0] * value[1]);
        ifOk(driveSpark, driveSpark::getOutputCurrent, (value) -> inputs.driveCurrentAmps = value);
        inputs.driveConnected = driveConnectedDebounce.calculate(!sparkStickyFault);

        // update turn inputs
        sparkStickyFault = false;
        ifOk(
            turnSpark,
            turnEncoder::getPosition,
            (value) -> inputs.turnPosition = new Rotation2d(value).minus(zeroRotation));

        ifOk(turnSpark, turnEncoder::getVelocity, (value) -> inputs.turnVelocityRadPerSec = value);
        ifOk(
            turnSpark,
            new DoubleSupplier[] {turnSpark::getAppliedOutput, turnSpark::getBusVoltage},
            (value) -> inputs.turnAppliedVolts =  value[0] * value[1]);
        ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
        inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

        //update odmetry inputs
        inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRad = drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryTurnPositions = turnPositionQueue.stream().map((Double value) -> new Rotation2d(value).minus(zeroRotation))
        .toArray(Rotation2d[]::new);
        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }


}

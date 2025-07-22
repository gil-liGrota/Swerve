package frc.robot.Subsystems.Swerve;

import static frc.robot.Subsystems.Swerve.DriveConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import java.util.Queue;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

public class ModuleIOReal implements ModuleIO {

    private final Rotation2d zeroRotation;
    private final int module;
    private Timer timer = new Timer();
    // Hardware objects
    private final TalonFX driveMotor;
    private final SparkMax turnMotor;
    private final CANcoder turnEncoder;

    // Close loop controllers
    private final SparkClosedLoopController turnController;
    private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

    // Queue inputs from odometry thread
    private final Queue<Double> timestampQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> turnPositionQueue;

    // Connection debouncers
    private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnConnectedDebounce = new Debouncer(0.5);

    public ModuleIOReal(int module) {
        this.module = module;
        zeroRotation = switch (module) {
            case 0 -> frontLeftZeroRotation;
            case 1 -> frontRightZeroRotation;
            case 2 -> backLeftZeroRotation;
            case 3 -> backRightZeroRotation;
            default -> new Rotation2d();
        };

        driveMotor = new TalonFX(swerveBaseID + swerveModuleIDsCount * module);
        turnMotor = new SparkMax(swerveBaseID + 1 + swerveModuleIDsCount * module,
                MotorType.kBrushless);
        turnEncoder = new CANcoder(swerveBaseID + 2 + swerveModuleIDsCount * module);

        var encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.SensorDirection = module == 1 ? SensorDirectionValue.CounterClockwise_Positive
                : SensorDirectionValue.Clockwise_Positive;

        encoderConfig.MagnetSensor.MagnetOffset = zeroRotation.getRotations();

        tryUntilOk(5, () -> turnEncoder.getConfigurator().apply(encoderConfig, 0.25));

        // config drive motor
        var driveConfig = new TalonFXConfiguration();
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        Slot0Configs driveMotorGains = new Slot0Configs()
                .withKP(driveKp).withKD(driveKd).withKS(driveKs).withKV(driveKv);
        driveConfig.Slot0 = driveMotorGains;
        driveConfig.Feedback.SensorToMechanismRatio = driveEncoderPositionFactor;
        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = driveSlipCurrent;
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -driveSlipCurrent;
        driveConfig.CurrentLimits.StatorCurrentLimit = driveSlipCurrent;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        driveConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = driveRampRate;
        driveConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = driveRampRate;
        driveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = driveRampRate;
        tryUntilOk(5, () -> driveMotor.getConfigurator().apply(driveConfig, 0.25));
        tryUntilOk(5, () -> driveMotor.setPosition(0.0, 0.25));

        // config turn motor
        var turnConfig = new SparkMaxConfig();
        turnConfig
                .inverted(turnInverted)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(turnMotorCurrentLimit)
                .voltageCompensation(12.0)
                .closedLoopRampRate(turnMotorRampRate)
                .openLoopRampRate(turnMotorRampRate);

        turnConfig.encoder
                // .inverted(turnEncoderInverted)
                .positionConversionFactor(turnEncoderPositionFactor)
                .velocityConversionFactor(turnEncoderVelocityFactor)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);

        turnConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(turnPIDMinInput, turnPIDMaxInput)
                .pidf(turnKp, 0.0, turnKd, 0.0).outputRange(-turnPIDMaxOutput, turnPIDMaxOutput);

        turnConfig.signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);

        tryUntilOk(
                turnMotor,
                5,
                () -> turnMotor.configure(
                        turnConfig, ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters));

        turnController = turnMotor.getClosedLoopController();

        // create odomtry queues
        timestampQueue = OdometryThread.getInstance().makeTimestampQueue();
        drivePositionQueue = OdometryThread.getInstance().registerSignal(driveMotor.getPosition()::getValueAsDouble);
        turnPositionQueue = OdometryThread.getInstance().registerSignal(turnMotor, turnMotor.getEncoder()::getPosition);

        timer.start();
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {

        // update drive inputs
        var driveStatus = BaseStatusSignal.refreshAll(driveMotor.getPosition(), driveMotor.getVelocity(),
                driveMotor.getMotorVoltage(), driveMotor.getStatorCurrent());
        inputs.drivePositionRad = driveMotor.getPosition().getValueAsDouble();
        inputs.driveVelocityRadPerSec = driveMotor.getVelocity().getValueAsDouble();
        inputs.driveAppliedVolts = driveMotor.getMotorVoltage().getValueAsDouble();
        inputs.driveCurrentAmps = driveMotor.getStatorCurrent().getValueAsDouble();
        inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());

        // update turn inputs
        sparkStickyFault = false;
        ifOk(
                turnMotor,
                turnMotor.getEncoder()::getPosition,
                (value) -> inputs.turnPosition = new Rotation2d(value));

        ifOk(turnMotor, turnMotor.getEncoder()::getVelocity, (value) -> inputs.turnVelocityRadPerSec = value);
        ifOk(
                turnMotor,
                new DoubleSupplier[] { turnMotor::getAppliedOutput, turnMotor::getBusVoltage },
                (value) -> inputs.turnAppliedVolts = value[0] * value[1]);
        ifOk(turnMotor, turnMotor::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
        inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);
        inputs.absolutePosition = getAbsolutePosition();

        // update odmetry inputs
        inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRad = drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryTurnPositions = turnPositionQueue.stream()
                .map((Double value) -> new Rotation2d(value).minus(zeroRotation))
                .toArray(Rotation2d[]::new);
        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();

        if (timer.get() >= 3) {
            turnMotor.getEncoder().setPosition(getAbsolutePosition());
            timer.restart();
        }
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveMotor.setVoltage(output);
        Logger.recordOutput(getModuleString() + " applied volt", output);
    }

    @Override
    public void setTurnOpenLoop(double output) {
        turnMotor.setVoltage(output);
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        driveMotor.setControl(velocityVoltageRequest.withVelocity(velocityRadPerSec));
    }

    @Override
    public void setTurnPosition(Rotation2d setpoint) {
        double error = setpoint.minus(Rotation2d.fromRadians(getAbsolutePosition())).getRadians();
        if (Math.abs(error) >= Math.PI) {
            error -= Math.copySign(Math.PI, error);
            error *= -1;
        }
        var ks = Math.copySign(turnKs, error);
        Logger.recordOutput(getModuleString() + "/ks", ks);
        Logger.recordOutput(getModuleString() + "/error", error);
        if (Math.abs(error) > 0.05) {
            turnController.setReference(setpoint.getRadians(), ControlType.kPosition,
                    ClosedLoopSlot.kSlot0, ks, ArbFFUnits.kVoltage);
        } else {
            setTurnOpenLoop(0);
        }
    }

    public double getAbsolutePosition() {
        return turnEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
    }

    private String getModuleString() {
        return "Module " + switch (module) {
            case 0 -> "FL";
            case 1 -> "FR";
            case 2 -> "BL";
            case 3 -> "BR";
            default -> "Unknown";
        };
    }

}

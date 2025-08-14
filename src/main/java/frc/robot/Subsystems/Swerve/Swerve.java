package frc.robot.Subsystems.Swerve;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.currentMode;
import static frc.robot.Subsystems.Swerve.DriveConstants.driveBaseRadius;
import static frc.robot.Subsystems.Swerve.DriveConstants.maxSpeedMetersPerSec;
import static frc.robot.Subsystems.Swerve.DriveConstants.*;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Mode;
import frc.robot.util.LocalADStarAK;

public class Swerve extends SubsystemBase { // ODED WAS HERE
    boolean goodVision = true;
    public static Lock odometryLock = new ReentrantLock();
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR
    private final SysIdRoutine driveSysId;
    private final SysIdRoutine steerSysId;
    private final Alert gyroDisconnectedAlert = new Alert("Disconnected gyro, using kinematics as fallback.",
            AlertType.kError);

    public static double ODOMETRY_FREQUENCY = 0.0;
    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
    private Rotation2d rawGyroRotation = new Rotation2d();
    private SwerveModulePosition[] lastModulePositions = // For delta tracking
            new SwerveModulePosition[] {
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition()
            };
    private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation,
            lastModulePositions, new Pose2d(3, 3, new Rotation2d()));

    public Swerve(GyroIO gyroIO, ModuleIO flModuleIO, ModuleIO frModuleIO, ModuleIO blModuleIO, ModuleIO brModuleIO) {
        this.gyroIO = gyroIO;
        modules[0] = new Module(flModuleIO, 0);
        modules[1] = new Module(frModuleIO, 1);
        modules[2] = new Module(blModuleIO, 2);
        modules[3] = new Module(brModuleIO, 3);

        // Usage reporting for swerve template
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

        // Start odometry thread
        OdometryThread.getInstance().start();

        // Configure AutoBuilder for PathPlanner
        AutoBuilder.configure(
                this::getPose,
                this::setPose,
                this::getChassisSpeeds,
                this::runPureVelocity,
                new PPHolonomicDriveController(new PIDConstants(2.5, 0.0, 0.0), new PIDConstants(2.5, 0.0, 0.0)), // TODOn
                                                                                                                  // pid
                ppConfig, // FIXME on comment in drive constance
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this);

        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback(
                (activePath) -> {
                    Logger.recordOutput(
                            "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
                });
        PathPlannerLogging.setLogTargetPoseCallback(
                (targetPose) -> {
                    Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
                });

        // Configure SysId
        driveSysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null,
                        null,
                        null,
                        (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
        steerSysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null,
                        null,
                        null,
                        (state) -> Logger.recordOutput("Drive/SteerSysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> runSteerCharacterization(voltage.in(Volts)), null, this));

        PushSwerveData();
        Field2d field = new Field2d();
        for (int i = 0; i < 6; i += 1) {
            field.getObject("left" + i).setPose(FieldConstants.Reef.redLeftBranches[i]);
            field.getObject("right" + i).setPose(FieldConstants.Reef.redRightBranches[i]);
        }
        SmartDashboard.putData("Field", field);
    }

    public static final DriveTrainSimulationConfig maplesimConfig = DriveTrainSimulationConfig.Default()// TODO יש פה
                                                                                                        // המון מספרים
                                                                                                        // שאני לא מבינה
                                                                                                        // אותם
            .withRobotMass(Kilograms.of(35.0))
            .withCustomModuleTranslations(moduleTranslations)
            .withGyro(COTS.ofPigeon2())
            .withSwerveModule(new SwerveModuleSimulationConfig(
                    DCMotor.getKrakenX60(1),
                    DCMotor.getNEO(1),
                    DriveConstants.driveMotorReduction,
                    DriveConstants.turnMotorReduction, // זה כאילו פרמטרים לסימולציה? יעננו גבולות וכאלה?
                    Volts.of(0.2),
                    Volts.of(0.2),
                    Inches.of(2),
                    KilogramSquareMeters.of(0.004),
                    DriveConstants.wheelCOF))
            .withBumperSize(Meters.of(0.7), Meters.of(0.7));

    @Override
    public void periodic() {
        odometryLock.lock(); // Prevents odometry updates while reading data
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        for (var module : modules) {
            module.periodic();
        }
        odometryLock.unlock();

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
        }

        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }
        Logger.recordOutput("good vision", goodVision);

        // Update odometry
        double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                        modulePositions[moduleIndex].distanceMeters
                                - lastModulePositions[moduleIndex].distanceMeters,
                        modulePositions[moduleIndex].angle);
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
                Logger.recordOutput("Module " + moduleIndex + " distance meters",
                        modulePositions[moduleIndex].distanceMeters);
            }

            // Update gyro angle
            if (gyroInputs.connected) {
                // Use the real gyro angle
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            // Apply update
            poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
        }

        // Update gyro alert
        gyroDisconnectedAlert.set(!gyroInputs.connected && currentMode != Mode.SIM);
    }

    public void runPureVelocity(ChassisSpeeds speeds) {
        runVelocity(speeds, false);
    }

    public void runVelocity(ChassisSpeeds speeds, boolean isOpenLoop) {
        // Calculate module setpoints
        speeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, maxSpeedMetersPerSec);

        // Log unoptimized setpoints
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", speeds);

        if (Math.abs(speeds.omegaRadiansPerSecond) + Math.abs(speeds.vxMetersPerSecond)
                + Math.abs(speeds.vyMetersPerSecond) < 0.01) {
            for (int i = 0; i < 4; i++) {
                modules[i].stop();
            }
        } else {
            // Send setpoints to modules
            for (int i = 0; i < 4; i++) {
                modules[i].runSetpoint(setpointStates[i], isOpenLoop);
            }
        }

        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }

    public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
        if (goodVision
        // && !(DriverStation.isEnabled() &&
        // visionPose.getTranslation().getDistance(getPose().getTranslation()) > 0.5)) {
        ) {
            poseEstimator.addVisionMeasurement(visionPose, timestamp);
        }
    }

    /** Runs the drive in a straight line with the specified drive output. */
    public void runCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(output);
        }
    }

    /** Runs the drive in a straight line with the specified drive output. */
    public void runSteerCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            modules[i].runSteerCharacterization(output);
        }
    }

    /** Stops the drive. */
    public void stop() {
        runVelocity(new ChassisSpeeds(), true);
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement.
     * The modules will
     * return to their normal orientations the next time a nonzero velocity is
     * requested.
     */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = moduleTranslations[i].getAngle();
        }
        kinematics.resetHeadings(headings);
        stop();
    }

    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {// TODO לא הבנתי עד ל
        return run(() -> runCharacterization(0.0))
                .withTimeout(1.0)
                .andThen(driveSysId.quasistatic(direction))
                .andThen(this::stop);
    }

    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command sysIdSteerQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0))
                .withTimeout(1.0)
                .andThen(steerSysId.quasistatic(direction))
                .andThen(this::stop);
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(driveSysId.dynamic(direction))
                .andThen(this::stop);
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdSteerDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(steerSysId.dynamic(direction))
                .andThen(this::stop);
    }// TODO כאן

    /**
     * Returns the module states (turn angles and drive velocities) for all of the
     * modules.
     */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /**
     * Returns the module positions (turn angles and drive positions) for all of the
     * modules.
     */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    /** Returns the measured chassis speeds of the robot. */
    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    /** Returns the position of each module in radians. */
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = modules[i].getWheelRadiusCharacterizationPosition();
        }
        return values;
    }

    /** Returns the average velocity of the modules in rad/sec. */
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += modules[i].getFFCharacterizationVelocity() / 4.0;
        }
        return output;
    }

    /** Returns the current odometry pose. */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    /** Resets the current odometry pose. */
    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }

    /** Adds a new timestamped vision measurement. */
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        if (goodVision
        // && !(DriverStation.isEnabled()
        // &&
        // visionRobotPoseMeters.getTranslation().getDistance(getPose().getTranslation())
        // > 0.5)) {
        ) {
            poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);

        }
    }

    /** Returns the maximum linear speed in meters per sec. */
    public double getMaxLinearSpeedMetersPerSec() {
        return maxSpeedMetersPerSec;
    }

    /** Returns the maximum angular speed in radians per sec. */
    public double getMaxAngularSpeedRadPerSec() {
        return maxSpeedMetersPerSec / driveBaseRadius;
    }

    public void disableGoodVision() {
        this.goodVision = false;
    }

    public void resetKinematics() {
        Rotation2d[] rotations = new Rotation2d[4];
        for (int i = 0; i < rotations.length; i++) {
            rotations[i] = modules[i].getAngle();
        }
        kinematics.resetHeadings(rotations);
    }

    public void resetGyro() {
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            resetGyro(new Rotation2d());
            setPose(new Pose2d(getPose().getTranslation(), new Rotation2d()));
        } else {
            resetGyro(new Rotation2d(Math.PI));
            setPose(new Pose2d(getPose().getTranslation(), new Rotation2d(Math.PI)));
        }
    }

    public void resetGyro(Rotation2d to) {
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            to = to.minus(new Rotation2d(Math.PI));
        }
        gyroIO.reset(to);
        setPose(new Pose2d(getPose().getTranslation(), to));
    }

    public Command resetGyroCommand() {
        return this.runOnce(this::resetGyro).ignoringDisable(true);
    }

    public Command resetGyroCommand(Rotation2d to) {
        return this.runOnce(() -> this.resetGyro(to)).ignoringDisable(true);
    }

    public Command resetGyroCommand(Supplier<Rotation2d> to) {
        return this.runOnce(() -> resetGyro(to.get())).ignoringDisable(true);
    }

    public Command testSteeringCommand(DoubleSupplier x, DoubleSupplier y) {// מה לעזאזל קורה פה
        return this.run(() -> { // TODO
            Rotation2d angle = new Rotation2d(x.getAsDouble(), y.getAsDouble());
            double a = MathUtil.applyDeadband(Math.hypot(x.getAsDouble(), y.getAsDouble()), 0.15);
            Logger.recordOutput("wanted angle", angle.getRadians());
            modules[0].runSetpoint(new SwerveModuleState(a / 4.0, angle), true);

        });
    }

    public Command testSteeringAngleCommand(Rotation2d angle) {// גם פה
        return this.run(() -> {

            // double a = MathUtil.applyDeadband(Math.hypot(x.getAsDouble(),
            // y.getAsDouble()), 0.15);
            Logger.recordOutput("wanted angle", angle.getRadians());
            modules[0].runSetpoint(new SwerveModuleState(4.0, angle), true);

        });
    }

    public void PushSwerveData() {
        SmartDashboard.putData("Swerve",
                builder -> {
                    builder.setSmartDashboardType("SwerveDrive");

                    builder.addDoubleProperty(
                            "Front Left Angle", () -> getModuleStates()[0].angle.getRadians(), null);
                    builder.addDoubleProperty(
                            "Front Left Velocity", () -> getModuleStates()[0].speedMetersPerSecond, null);

                    builder.addDoubleProperty(
                            "Front Right Angle", () -> getModuleStates()[1].angle.getRadians(), null);
                    builder.addDoubleProperty(
                            "Front Right Velocity", () -> getModuleStates()[1].speedMetersPerSecond, null);

                    builder.addDoubleProperty(
                            "Back Left Angle", () -> getModuleStates()[2].angle.getRadians(), null);
                    builder.addDoubleProperty(
                            "Back Left Velocity", () -> getModuleStates()[2].speedMetersPerSecond, null);

                    builder.addDoubleProperty(
                            "Back Right Angle", () -> getModuleStates()[3].angle.getRadians(), null);
                    builder.addDoubleProperty(
                            "Back Right Velocity", () -> getModuleStates()[3].speedMetersPerSecond, null);

                    builder.addDoubleProperty(
                            "Robot Angle", () -> gyroInputs.yawPosition.getRadians(), null);
                });
    }

}

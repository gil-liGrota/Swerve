package frc.robot.Commands;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Subsystems.Swerve.Swerve;

import static frc.robot.Subsystems.Swerve.DriveConstants.*;

public class SwerveCommands {

        private static final double DEADBAND = 0.15;
        private static final double ANGLE_KP = 1.8;
        private static final double ANGLE_KD = 0.2;
        private static final double TOLERANCE = 0.03;
        private static final double ANGLE_MAX_VELOCITY = 7.0;
        private static final double ANGLE_MAX_ACCELERATION = 7.0;
        private static final double FF_START_DELAY = 2.0; // Secs
        private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
        // private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
        // private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

        private SwerveCommands() {
        }

        private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
                // Apply deadband
                double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
                Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

                // Square magnitude for more precise control
                linearMagnitude = linearMagnitude * linearMagnitude;

                // Return new linear velocity
                return new Pose2d(new Translation2d(), linearDirection)
                                .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                                .getTranslation();
        }

        /**
         * Field relative drive command using two joysticks (controlling linear and
         * angular velocities).
         */
        public static Command joystickDrive(Swerve drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier,
                        DoubleSupplier omegaSupplier) {
                return Commands.run(
                                () -> {
                                        // Get linear velocity
                                        Translation2d linearVelocity = getLinearVelocityFromJoysticks(
                                                        -xSupplier.getAsDouble(),
                                                        -ySupplier.getAsDouble());

                                        linearVelocity = linearVelocity.times(0.15);// TODO remove

                                        // Apply rotation deadband
                                        double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

                                        // Square rotation value for more precise control
                                        omega = Math.copySign(omega * omega * 0.15, omega);

                                        // Convert to field relative speeds & send command
                                        ChassisSpeeds speeds = new ChassisSpeeds(
                                                        linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                                                        linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                                                        omega * drive.getMaxAngularSpeedRadPerSec());
                                        boolean isFlipped = DriverStation.getAlliance().isPresent()
                                                        && DriverStation.getAlliance().get() == Alliance.Red;
                                        drive.runVelocity(
                                                        ChassisSpeeds.fromFieldRelativeSpeeds(
                                                                        speeds,
                                                                        isFlipped
                                                                                        ? drive.getRotation().plus(
                                                                                                        new Rotation2d(Math.PI))
                                                                                        : drive.getRotation()),
                                                        true);
                                },
                                drive);
        }

        /**
         * Field relative drive command using joystick for linear control and PID for
         * angular control.
         * Possible use cases include snapping to an angle, aiming at a vision target,
         * or controlling
         * absolute rotation with a joystick.
         */
        public static Command joystickDriveAtAngle(Swerve drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier,
                        Supplier<Rotation2d> rotationSupplier) {
                // Create PID controller
                ProfiledPIDController angleController = new ProfiledPIDController(
                                ANGLE_KP,
                                0.0,
                                ANGLE_KD,
                                new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
                angleController.enableContinuousInput(-Math.PI, Math.PI);

                // Construct command
                return Commands.run(
                                () -> {
                                        // Get linear velocity
                                        Translation2d linearVelocity = getLinearVelocityFromJoysticks(
                                                        xSupplier.getAsDouble(), ySupplier.getAsDouble());

                                        // Calculate angular speed
                                        double omega = angleController.calculate(
                                                        drive.getRotation().getRadians(),
                                                        rotationSupplier.get().getRadians());

                                        // Convert to field relative speeds & send command
                                        ChassisSpeeds speeds = new ChassisSpeeds(
                                                        linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                                                        linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                                                        omega);
                                        boolean isFlipped = DriverStation.getAlliance().isPresent()
                                                        && DriverStation.getAlliance().get() == Alliance.Red;
                                        drive.runVelocity(
                                                        ChassisSpeeds.fromFieldRelativeSpeeds(
                                                                        speeds,
                                                                        isFlipped
                                                                                        ? drive.getRotation().plus(
                                                                                                        new Rotation2d(Math.PI))
                                                                                        : drive.getRotation()),
                                                        true); // אותה בעיה, שוב שמתי ככה
                                },
                                drive)

                                // Reset PID controller when command starts
                                .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
        }

        /**
         * Measures the velocity feedforward constants for the drive motors.
         *
         * <p>
         * This command should only be used in voltage control mode.
         */
        public static Command feedforwardCharacterization(Swerve drive) {// לא הבנתי כלום
                List<Double> velocitySamples = new LinkedList<>();
                List<Double> voltageSamples = new LinkedList<>();
                Timer timer = new Timer();

                return Commands.sequence(
                                // Reset data
                                Commands.runOnce(
                                                () -> {
                                                        velocitySamples.clear();
                                                        voltageSamples.clear();
                                                }),

                                // Allow modules to orient
                                Commands.run(
                                                () -> {
                                                        drive.runCharacterization(0.0);
                                                },
                                                drive)
                                                .withTimeout(FF_START_DELAY),

                                // Start timer
                                Commands.runOnce(timer::restart),

                                // Accelerate and gather data
                                Commands.run(
                                                () -> {
                                                        double voltage = timer.get() * FF_RAMP_RATE;
                                                        drive.runCharacterization(voltage);
                                                        velocitySamples.add(drive.getFFCharacterizationVelocity());
                                                        voltageSamples.add(voltage);
                                                },
                                                drive)

                                                // When cancelled, calculate and print results
                                                .finallyDo(
                                                                () -> {
                                                                        int n = velocitySamples.size();
                                                                        double sumX = 0.0;
                                                                        double sumY = 0.0;
                                                                        double sumXY = 0.0;
                                                                        double sumX2 = 0.0;
                                                                        for (int i = 0; i < n; i++) {
                                                                                sumX += velocitySamples.get(i);
                                                                                sumY += voltageSamples.get(i);
                                                                                sumXY += velocitySamples.get(i)
                                                                                                * voltageSamples.get(i);
                                                                                sumX2 += velocitySamples.get(i)
                                                                                                * velocitySamples
                                                                                                                .get(i);
                                                                        }
                                                                        double kS = (sumY * sumX2 - sumX * sumXY)
                                                                                        / (n * sumX2 - sumX * sumX);
                                                                        double kV = (n * sumXY - sumX * sumY)
                                                                                        / (n * sumX2 - sumX * sumX);

                                                                        NumberFormat formatter = new DecimalFormat(
                                                                                        "#0.00000");
                                                                        System.out.println(
                                                                                        "********** Drive FF Characterization Results **********");
                                                                        System.out.println("\tkS: "
                                                                                        + formatter.format(kS));
                                                                        System.out.println("\tkV: "
                                                                                        + formatter.format(kV));
                                                                }));
        }

        /*
         * public static Command wheelRadiusCharacterization(Swerve drive) {// לא הבנתי
         * כלום
         * SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
         * WheelRadiusCharacterizationState state = new
         * WheelRadiusCharacterizationState();
         * 
         * return Commands.parallel(
         * // Drive control sequence
         * Commands.sequence(
         * // Reset acceleration limiter
         * Commands.runOnce(
         * () -> {
         * limiter.reset(0.0);
         * }),
         * 
         * // Turn in place, accelerating up to full speed
         * Commands.run(
         * () -> {
         * double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
         * drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed), true);// אותה בעיה...
         * },
         * drive)),
         * 
         * // Measurement sequence
         * Commands.sequence(
         * // Wait for modules to fully orient before starting measurement
         * Commands.waitSeconds(1.0),
         * 
         * // Record starting measurement
         * Commands.runOnce(
         * () -> {
         * state.positions = drive.getWheelRadiusCharacterizationPositions();
         * state.lastAngle = drive.getRotation();
         * state.gyroDelta = 0.0;
         * }),
         * 
         * // Update gyro delta
         * Commands.run(
         * () -> {
         * var rotation = drive.getRotation();
         * state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
         * state.lastAngle = rotation;
         * })
         * 
         * // When cancelled, calculate and print results
         * .finallyDo(
         * () -> {
         * double[] positions = drive.getWheelRadiusCharacterizationPositions();
         * double wheelDelta = 0.0;
         * for (int i = 0; i < 4; i++) {
         * wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
         * }
         * double wheelRadius = (state.gyroDelta * Swerve.DRIVE_BASE_RADIUS)
         * // אין לי
         * // מושג מה
         * // זה
         * / wheelDelta;
         * 
         * NumberFormat formatter = new DecimalFormat("#0.000");
         * System.out.println(
         * "********** Wheel Radius Characterization Results **********");
         * System.out.println(
         * "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
         * System.out.println(
         * "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
         * System.out.println(
         * "\tWheel Radius: "
         * + formatter.format(wheelRadius)
         * + " meters, "
         * + formatter.format(Units.metersToInches(wheelRadius))
         * + " inches");
         * })));
         * }
         */
        // private static class WheelRadiusCharacterizationState {
        // double[] positions = new double[4];
        // Rotation2d lastAngle = new Rotation2d();
        // double gyroDelta = 0.0;
        // }

        public static class DriveToPosition extends Command {
                private final Swerve m_drive;
                private final Pose2d m_target;
                private final ProfiledPIDController m_controllerX;
                private final ProfiledPIDController m_controllerY;
                private final ProfiledPIDController m_controllerTheta;
                private final Timer m_timer = new Timer();

                public DriveToPosition(Swerve drive, Pose2d target) {
                        this.m_drive = drive;
                        this.m_target = target;
                        this.m_controllerX = new ProfiledPIDController(KP_XY, KI_XY, KD_XY,
                                        new TrapezoidProfile.Constraints(MAX_VELOCETY_XY, MAX_ACCELERATION_XY));
                        this.m_controllerY = new ProfiledPIDController(KP_XY, KI_XY, KD_XY,
                                        new TrapezoidProfile.Constraints(MAX_VELOCETY_XY, MAX_ACCELERATION_XY));
                        this.m_controllerTheta = new ProfiledPIDController(KP_OMEGA, KI_OMEGA, KD_OMEGA,
                                        new TrapezoidProfile.Constraints(MAX_VELOCETY_OMEGA, MAX_ACCELERATION_OMEGA));
                        this.m_controllerX.setTolerance(TRANSLATION_TOLERANCE);
                        this.m_controllerY.setTolerance(TRANSLATION_TOLERANCE);
                        this.m_controllerTheta.setTolerance(OMEGA_TOLERANCE);
                        this.m_controllerTheta.enableContinuousInput(-Math.PI, Math.PI);
                        addRequirements(drive);

                }

                LoggedNetworkNumber kpTune = new LoggedNetworkNumber("tunes/translation kp", KP_XY);
                LoggedNetworkNumber kiTune = new LoggedNetworkNumber("tunes/translation ki", KI_XY);
                LoggedNetworkNumber kdTune = new LoggedNetworkNumber("tunes/translation kd", KD_XY);
                LoggedNetworkNumber maxVelocityTune = new LoggedNetworkNumber("tunes/translation max velocity",
                                MAX_VELOCETY_XY);
                LoggedNetworkNumber maxAccelerationTune = new LoggedNetworkNumber("tunes/translation max acceleration",
                                MAX_ACCELERATION_XY);

                LoggedNetworkNumber kpThetaTune = new LoggedNetworkNumber("tunes/rotation kp", KP_OMEGA);
                LoggedNetworkNumber kiThetaTune = new LoggedNetworkNumber("tunes/rotation ki", KI_OMEGA);
                LoggedNetworkNumber kdThetaTune = new LoggedNetworkNumber("tunes/rotation kd", KD_OMEGA);
                LoggedNetworkNumber maxVelocityThetaTune = new LoggedNetworkNumber("tunes/rotation max velocity",
                                MAX_VELOCETY_OMEGA);
                LoggedNetworkNumber maxAccelerationThetaTune = new LoggedNetworkNumber(
                                "tunes/rotation max acceleration",
                                MAX_ACCELERATION_OMEGA);

                LoggedNetworkNumber translationTolerance = new LoggedNetworkNumber("tunes/translation tolerance",
                                TRANSLATION_TOLERANCE);
                LoggedNetworkNumber omegaTolerance = new LoggedNetworkNumber("tunes/rotation tolerance",
                                OMEGA_TOLERANCE);

                @Override
                public void initialize() {
                        m_timer.reset();
                        m_timer.start();
                        var currPose = m_drive.getPose();
                        ChassisSpeeds currS = ChassisSpeeds.fromRobotRelativeSpeeds(m_drive.getChassisSpeeds(),
                                        currPose.getRotation());

                        m_controllerX.reset(currPose.getX(), currS.vxMetersPerSecond);
                        m_controllerY.reset(currPose.getY(), currS.vyMetersPerSecond);
                        m_controllerTheta.reset(currPose.getRotation().getRadians(), currS.omegaRadiansPerSecond);
                }

                @Override
                public void execute() {
                        // m_controllerX.setPID(kpTune.get(), kiTune.get(), kdTune.get());
                        // m_controllerX
                        // .setConstraints(new TrapezoidProfile.Constraints(maxVelocityTune.get(),
                        // maxAccelerationTune.get()));
                        // m_controllerY.setPID(kpTune.get(), kiTune.get(), kdTune.get());
                        // m_controllerY
                        // .setConstraints(new TrapezoidProfile.Constraints(maxVelocityTune.get(),
                        // maxAccelerationTune.get()));
                        // m_controllerTheta.setPID(kpThetaTune.get(), kiThetaTune.get(),
                        // kdThetaTune.get());
                        // m_controllerTheta.setConstraints(
                        // new TrapezoidProfile.Constraints(maxVelocityThetaTune.get(),
                        // maxAccelerationThetaTune.get()));

                        var pose = m_drive.getPose();
                        var chassisSpeeds = new ChassisSpeeds(
                                        m_controllerX.calculate(pose.getTranslation().getX(),
                                                        m_target.getTranslation().getX()),
                                        m_controllerY.calculate(pose.getTranslation().getY(),
                                                        m_target.getTranslation().getY()),
                                        m_controllerTheta.calculate(pose.getRotation().getRadians(),
                                                        m_target.getRotation().getRadians()));

                        // if (Math.abs(m_target.getX() - pose.getX()) + Math.abs(m_target.getY() -
                        // pose.getY()) < 0.2) {
                        // chassisSpeeds = new ChassisSpeeds(chassisSpeeds.vxMetersPerSecond,
                        // chassisSpeeds.vyMetersPerSecond, 0);
                        // }

                        // if ((pose.getRotation().getDegrees() % 360 + 470) % 360 > 180) {
                        // chassisSpeeds = new ChassisSpeeds(-chassisSpeeds.vxMetersPerSecond,
                        // -chassisSpeeds.vyMetersPerSecond,
                        // chassisSpeeds.omegaRadiansPerSecond);
                        // }
                        chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds,
                                        pose.getRotation().unaryMinus());
                        Logger.recordOutput("Requested speeds", chassisSpeeds);
                        // Logger.recordOutput("pose", pose);
                        // Logger.recordOutput("pose to", m_target);
                        Logger.recordOutput("error x", m_target.getTranslation().getX() - pose.getTranslation().getX());
                        Logger.recordOutput("error y", m_target.getTranslation().getY() - pose.getTranslation().getY());
                        m_drive.runVelocity(chassisSpeeds, false);
                }

                @Override
                public void end(boolean interrupted) {
                        // if (!interrupted) {
                        // m_drive.stop();
                        // }
                }

                @Override
                public boolean isFinished() {
                        // return m_timer.hasElapsed(5) ||
                        return (m_controllerX.atGoal() && m_controllerY.atGoal() && m_controllerTheta.atGoal());
                }
        }

        public static Command goToAngle(Swerve drive, Rotation2d goal) {
                ProfiledPIDController angleController = new ProfiledPIDController(
                                ANGLE_KP,
                                0.0,
                                ANGLE_KD,
                                new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
                angleController.enableContinuousInput(-Math.PI, Math.PI);
                angleController.setTolerance(TOLERANCE);
                // return Commands.startEnd(() -> drive.runVelocity(
                // new ChassisSpeeds(0, 0, angleController.calculate(
                // drive.getPose().getRotation().getRadians(), goal.getRadians())),
                // true),
                // () -> drive.stop(), drive);

                return new FunctionalCommand(
                                () -> angleController.reset(drive.getRotation().getRadians(), 0),
                                () -> drive.runVelocity(
                                                new ChassisSpeeds(0, 0, angleController.calculate(
                                                                drive.getPose().getRotation().getRadians(),
                                                                goal.getRadians())),
                                                true),
                                interrupted -> drive.stop(),
                                angleController::atGoal,
                                drive);
        }

}

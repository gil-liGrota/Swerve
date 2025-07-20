package frc.robot.Subsystems.Swerve;

import static frc.robot.Subsystems.Swerve.DriveConstants.pigeonCanId;

import java.util.Queue;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;

public class GyroIOPigeon implements GyroIO {
    private final WPI_PigeonIMU pigeon = new WPI_PigeonIMU(pigeonCanId);
    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;
    private Rotation2d offset = new Rotation2d();

    public GyroIOPigeon() {
        pigeon.reset();

        yawTimestampQueue = OdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = OdometryThread.getInstance().registerSignal(pigeon::getYaw);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = pigeon.getState() != PigeonState.NoComm;
        inputs.yawPosition = Rotation2d.fromDegrees(pigeon.getYaw()).minus(offset);
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(pigeon.getRate());

        inputs.odometryYawTimestams = yawTimestampQueue.stream().mapToDouble((Double Value) -> Value).toArray();
        inputs.odometryYawPosition = yawPositionQueue.stream().map((Double Value) -> Rotation2d.fromDegrees(Value))
                .toArray(Rotation2d[]::new);

        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }

    @Override
    public Rotation2d getGyroRotation() {
        return pigeon.getRotation2d().minus(offset);
    }

    @Override
    public AngularVelocity getGyroAngularVelocity() {
        return AngularVelocity.ofBaseUnits(Units.degreesToRadians(pigeon.getRate()), RadiansPerSecond);
    }

    @Override
    public void reset() {
        reset(new Rotation2d());
    }

    @Override
    public void reset(Rotation2d to) {
        offset = pigeon.getRotation2d().minus(to);
    }

}

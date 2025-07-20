package frc.robot.Subsystems.Swerve;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public static Lock odometryLock = new ReentrantLock();
}

package frc.robot.Subsystems.Swerve;

import java.util.Queue;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;

public class ModuleIOReal extends ModuleIO{
    
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
            case 0 -> front
        }
    }

    
}

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterConstants {

    //TODO change these
    public static final int leftMotorID = 17;
    public static final int rightMotorID = 15;

    public static final double left_kP = 0;
    public static final double left_kI = 0.02;
    public static final double left_kD = 0;
    public static final double left_kS = 0.28;
    public static final double left_kV = 0.12;
    public static final double left_kA = 0;

    public static final double right_kP = 0;
    public static final double right_kI = 0.02;
    public static final double right_kD = 0;
    public static final double right_kS = 0.28;
    public static final double right_kV = 0.12;
    public static final double right_kA = 0;



    public static final double leftMaxAcceleration = 0.1;
    public static final double leftMaxJerk = 0.1;

    public static final double rightMaxAcceleration = 0.1;
    public static final double rightMaxJerk = 0.1;

    public static final CANBus canbus = new CANBus("SuperStructure");

    // (distance, desired vel)
    public static InterpolatingDoubleTreeMap shooterSpeedMapScoring =
        new InterpolatingDoubleTreeMap();
    public static InterpolatingDoubleTreeMap timeOfFlightMapScoring =
        new InterpolatingDoubleTreeMap();
    public static InterpolatingDoubleTreeMap shooterSpeedMapFerrying =
        new InterpolatingDoubleTreeMap();

    //TODO Retune with new values in cafeteria
    static {
        shooterSpeedMapScoring.put(2.93, 59.0);
        shooterSpeedMapScoring.put(3.81, 67.0);
        shooterSpeedMapScoring.put(3.886, 70.0);
        shooterSpeedMapScoring.put(2.609, 58.0);
        shooterSpeedMapScoring.put(3.32, 62.0);
        shooterSpeedMapScoring.put(3.56, 65.0);
        shooterSpeedMapScoring.put(3.13, 61.0);
        shooterSpeedMapScoring.put(2.78, 58.0);
    }

    public static final double ferrySpeed = 80.0;
}
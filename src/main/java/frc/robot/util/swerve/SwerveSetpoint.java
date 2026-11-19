package frc.robot.util.swerve;

import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveModuleState;

public record SwerveSetpoint(ChassisVelocities chassisSpeeds, SwerveModuleState[] moduleStates) {}

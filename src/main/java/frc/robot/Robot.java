package frc.robot;

import org.wpilib.util.sendable.Sendable;
import org.wpilib.util.sendable.SendableBuilder;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchType;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.driverstation.Alliance;
import org.wpilib.shuffleboard.BuiltInWidgets;
import org.wpilib.shuffleboard.Shuffleboard;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import frc.robot.autos.AutoChooser;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.subsystems.drive.Drive;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command autonomousCommand;

  private final Field2d field = new Field2d();

  private AutoChooser autoChooser;

  private final RobotContainer robotContainer;

  public Robot() {
    FieldConstants.setAlliance(MatchState.getAlliance().get());
    robotContainer = new RobotContainer();
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    Logger.start();

    Drive drive = robotContainer.getDrive();

    SmartDashboard.putData("Swerve Drive", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("Front Left Angle", () -> drive.getModules()[0].getAngle().getRadians() + (FieldConstants.getAlliance() == Alliance.RED ? Math.PI : 0) , null);
        builder.addDoubleProperty("Front Left Velocity", () -> drive.getModules()[0].getVelocityMetersPerSec(), null);

        builder.addDoubleProperty("Front Right Angle", () -> drive.getModules()[1].getAngle().getRadians() + (FieldConstants.getAlliance() == Alliance.RED ? Math.PI : 0), null);
        builder.addDoubleProperty("Front Right Velocity", () -> drive.getModules()[1].getVelocityMetersPerSec(), null);

        builder.addDoubleProperty("Back Left Angle", () -> drive.getModules()[2].getAngle().getRadians() + (FieldConstants.getAlliance() == Alliance.RED ? Math.PI : 0), null);
        builder.addDoubleProperty("Back Left Velocity", () -> drive.getModules()[2].getVelocityMetersPerSec(), null);

        builder.addDoubleProperty("Back Right Angle", () -> drive.getModules()[3].getAngle().getRadians() + (FieldConstants.getAlliance() == Alliance.RED ? Math.PI : 0), null);
        builder.addDoubleProperty("Back Right Velocity", () -> drive.getModules()[3].getVelocityMetersPerSec(), null);

        builder.addDoubleProperty("Robot Angle", () -> drive.getRotation().getRadians() + (FieldConstants.getAlliance() == Alliance.RED ? Math.PI : 0), null);
      }
    });

    SmartDashboard.putData(field);
    SmartDashboard.putData(CommandScheduler.getInstance());
    


    autoChooser = AutoChooser.create(
        robotContainer, robotContainer.getDrive(),
        robotContainer.getSuperstructure());
    Shuffleboard.getTab("Testing")
        .add("Auto Program", autoChooser)
        .withSize(6, 3)
        .withPosition(12, 0)
        .withWidget(BuiltInWidgets.kComboBoxChooser);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    field.setRobotPose(robotContainer.getDrive().getPose());
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {

    // autoChooser.reset("SmartDashboard/Autonomous/2025Programs");
    // robotContainer.getSuperstructure().setWantedSuperState(SuperState.STOPPED);
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    autoChooser.update();
    FieldConstants.setAlliance(MatchState.getAlliance().get());
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    // autonomousCommand = robotContainer.getAutonomousCommand();

    // // schedule the autonomous command (example)
    // if (autonomousCommand != null) {
    // autonomousCommand.schedule();
    // }
    robotContainer.getSuperstructure().setDesiredSuperStateCommand(SuperState.DRIVING);
    //robotContainer.getSuperstructure().getShooter().setShooterChangeVelocity(2);
    autoChooser.getSelectedCommand().ifPresent(CommandScheduler.getInstance()::schedule);
    
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    // FieldConstants.setAlliance(MatchState.getAlliance().get());
    robotContainer.getSuperstructure().setDesiredSuperStateCommand(SuperState.DRIVING);
    RobotState.getInstance().setAutoAiming(false);
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void utilityInit() {
    CommandScheduler.getInstance().cancelAll();
  }

   @Override
  public void utilityPeriodic() {
  }

  @Override
  public void utilityExit() {
  }
}

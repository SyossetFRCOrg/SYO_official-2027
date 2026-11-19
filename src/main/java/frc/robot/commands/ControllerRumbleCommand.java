package frc.robot.commands;

import org.wpilib.driverstation.GenericHID;
import org.wpilib.driverstation.XboxController;
import org.wpilib.command2.Command;
import java.util.function.BooleanSupplier;

public class ControllerRumbleCommand extends Command {
  private XboxController controller;
  private BooleanSupplier condition;

  public ControllerRumbleCommand(XboxController controller, BooleanSupplier condition) {
    this.controller = controller;
    this.condition = condition;
  }

  @Override
  public void execute() {
    if (condition.getAsBoolean()) {
      controller.setRumble(GenericHID.RumbleType.kBothRumble, 1);
    }
  }

  @Override
  public void end(boolean interrupted) {
    controller.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
  }
}

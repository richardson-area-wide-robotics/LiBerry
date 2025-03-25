package frc.robot.common.components;

import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.experimental.UtilityClass;

@UtilityClass
public class RobotUtils  {

  public RobotConfig robotConfig;

  /**
   * Helper method to bind a control action to a command.
   *
   * @param control The button to bind to.
   * @param command The command to execute when that button is pressed.
   * @param stopCommand The command to execute when that button is *not* pressed
   *
   * @author Hudson Strub
   * @since 2025
   */
  public static void bindControl(Trigger control, Command command, Command stopCommand) {
    control.whileTrue(command).whileFalse(stopCommand);
  }

  /**
   * Helper method to get the team number, the same as {@link HALUtil#getTeamNumber}
   * Only added because I can never remember the import
   *
   * @author Hudson Strub
   * @since 2025
   */
  public static int getTeamNumber(){
    return HALUtil.getTeamNumber();
  }


   /**
   * Load the robot config used for pathplanner, 
   *
   * @author Alan Trinh
   * @since 2025
   */
  public static void loadRobotConfig() {
    try {
        robotConfig = RobotConfig.fromGUISettings();
      } 
      catch (Exception e) {
        throw new RuntimeException("Failed to load robot config from GUI settings");
      }
  }


  /**
   * Run a command for a given amount of time
   * 
   * @param seconds The amount of time to run commandDuring for, in seconds
   * @param commandDuring The command ran
   * @param commandAfter The command ran after the time has passed (Ex: Stop motor)
   * 
   *
   * @author Alan Trinh
   * @since 2025
   */
  public static Command timedCommand(double seconds, Command commandDuring, Command commandAfter){
    return Commands.deadline(Commands.waitSeconds(seconds), commandDuring).andThen(commandAfter);
  }


   /**
   * Move a motor to a relative positon 

   * @author Hudson Strub
   * @since 2025
   */
  public void moveToPosition(SparkFlex motor, double targetPosition) {
      // Set the target position using the built-in PID controller
      motor.getClosedLoopController().setReference(targetPosition, ControlType.kPosition);
  }
}
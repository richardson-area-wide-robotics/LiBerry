package frc.robot.common.interfaces;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Interface for a team's RobotContainer, 
 * ideally each team that needs different functionality would implement this
 * 
 * @author Hudson Strub
 * @since 2025
 */
public interface IRobotContainer {

    static IRobotContainer createContainer() {
        throw new UnsupportedOperationException("createContainer must be implemented in the specific RobotContainer class");
    }

    /**Get the command to use during auto */
    Command getAutonomousCommand();

    /**Ran periodicly during simulation */
    void simulationPeriodic();

    /**Ran periodicly when the robot is disabled. (Dont try it lmao) */
    void disabledPeriodic();

    /**Ran periodicly during auto */
    void autonomousPeriodic();

    /**Ran periodicly during teleop */
    void teleopPeriodic();
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.rawrobotics;

import org.rawrobotics.common.LocalADStarAK;
import org.rawrobotics.common.components.RobotContainerRegistry;
import org.rawrobotics.common.components.RobotExceptionHandler;
import org.rawrobotics.common.components.RobotUtils;
import java.nio.file.Path;

import org.lasarobotics.hardware.PurpleManager;
import org.littletonrobotics.junction.LoggedRobot;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.rawrobotics.common.interfaces.IRobotContainer;


/**
 * "Starting point" of the robot, nothing in here should need to be touched.
 * 
 * This sets up the ExceptionHandler, PathPlanner, Logging, and then creates a IRobotContainer based off the team number  
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;

  private IRobotContainer robotContainer;
  public Robot() {
super();
  }

  @Override
  @SuppressWarnings("resource")
  public void robotInit() {
    try{
      PurpleManager.initialize(
        this,
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField),
        Path.of("/media/sda1"),
        BuildConstants.MAVEN_NAME,
        BuildConstants.GIT_SHA,
        BuildConstants.BUILD_DATE,
        false
      );
    }
    catch (Exception e){
      System.out.println("Error loading PurpleManager" + e.getMessage() + e.getCause());
    }
    

    Thread.setDefaultUncaughtExceptionHandler(new RobotExceptionHandler());


    // Set pathfinding algorithm to be AdvantageKit compatible
    Pathfinding.setPathfinder(new LocalADStarAK());

    RobotUtils.loadRobotConfig();
    robotContainer = RobotContainerRegistry.createContainerForTeam(RobotUtils.getTeamNumber());
}


  @Override
  public void robotPeriodic() {
    PurpleManager.update();
    CommandScheduler.getInstance().run();
  }

  
  @Override
  public void disabledPeriodic() {
    robotContainer.disabledPeriodic();
  }

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    robotContainer.autonomousPeriodic();
    //PurpleManager.update();

  }

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    robotContainer.teleopPeriodic();
    //PurpleManager.update();

  }

  @Override
  public void simulationPeriodic() {
    robotContainer.simulationPeriodic();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
    //PurpleManager.update();
  }


}

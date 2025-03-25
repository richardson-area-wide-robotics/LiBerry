// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.common.subsystems.drive;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import lombok.Getter;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.lasarobotics.drive.swerve.AdvancedSwerveKinematics;
import org.lasarobotics.drive.swerve.AdvancedSwerveKinematics.ControlCentricity;
import org.lasarobotics.drive.swerve.SwerveModule;
import org.lasarobotics.drive.RotatePIDController;
import org.lasarobotics.drive.ThrottleMap;
import org.lasarobotics.utils.PIDConstants;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Rotation2d;      // For handling rotations
import edu.wpi.first.math.geometry.Translation2d;    // For handling 2D translations
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; // For dashboard integration
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.common.components.RobotUtils;
import frc.robot.common.swerve.RAWRNavX2;
import frc.robot.common.swerve.RAWRSwerveModule;
import frc.robot.common.components.SwerveHardware;

/**
 * Drive Subsystem for Swerve Drive bots with 4 motors in each corner
 *
 * @author PurpleLib
 * @author Alan Trinh
 * @author Hudson Strub
 *
 * @since 2025
 */
public class SwerveDriveSubsystem extends SubsystemBase implements AutoCloseable {

  /**FOR TESTING ONLY - Skip loading the drive train*/
  public final boolean enableDrive = true;

  public final SwerveHardware DRIVETRAIN_HARDWARE;

  private final ThrottleMap THROTTLE_MAP;
  private final RotatePIDController ROTATE_PID_CONTROLLER;
  @Getter
  private final SwerveDriveKinematics KINEMATICS;
  private final SwerveDrivePoseEstimator POSE_ESTIMATOR;

  public final LinearVelocity DRIVE_MAX_LINEAR_SPEED;
  public final LinearAcceleration DRIVE_AUTO_ACCELERATION;
  private AdvancedSwerveKinematics advancedKinematics;
  private final ProfiledPIDController AUTO_AIM_PID_CONTROLLER_FRONT;
  private final ProfiledPIDController AUTO_AIM_PID_CONTROLLER_BACK;
  private final MedianFilter X_VELOCITY_FILTER;
  private final MedianFilter Y_VELOCITY_FILTER;
  private final PPHolonomicDriveController PATH_FOLLOWER_CONFIG;

  private ControlCentricity controlCentricity;
  private ChassisSpeeds desiredChassisSpeeds;
  private Pose2d m_previousPose;
  private Rotation2d currentHeading;
  private final Field2d FIELD;



  /**
   * SwerveDriveSubsystem constructor for managing a swerve drivetrain.
   *
   * @param drivetrainHardware Hardware devices required by drivetrain.
   * @param pidf PID constants for the drive system.
   * @param controlCentricity Control centricity configuration.
   * @param throttleInputCurve Spline function for throttle input.
   * @param turnInputCurve Spline function for turn input.
   * @param turnScalar Scalar for turn input (degrees).
   * @param deadband Deadband for controller input [+0.001, +0.2].
   * @param lookAhead Rotate PID lookahead, in number of loops.
   */
  public SwerveDriveSubsystem(SwerveHardware drivetrainHardware, PIDConstants pidf, ControlCentricity controlCentricity,
                              PolynomialSplineFunction throttleInputCurve, PolynomialSplineFunction turnInputCurve,
                              Angle turnScalar, Dimensionless deadband, Time lookAhead) {
  
      if(enableDrive){
        // Initialize subsystem name
        setSubsystem(this.getClass().getSimpleName());
      
        this.DRIVETRAIN_HARDWARE = drivetrainHardware;

        // Drivetrain constants
        DRIVE_MAX_LINEAR_SPEED = DRIVETRAIN_HARDWARE.lFrontModule.getMaxLinearVelocity();
        DRIVE_AUTO_ACCELERATION = DRIVE_MAX_LINEAR_SPEED
            .per(Units.Second)
            .minus(Units.MetersPerSecondPerSecond.of(1.0));
    
        // Input curves and controllers
        this.controlCentricity = controlCentricity;
        this.THROTTLE_MAP = new ThrottleMap(throttleInputCurve, DRIVE_MAX_LINEAR_SPEED, deadband);
        this.ROTATE_PID_CONTROLLER = new RotatePIDController(turnInputCurve, pidf, turnScalar, deadband, lookAhead);
    
        // Path follower configuration
        this.PATH_FOLLOWER_CONFIG = new PPHolonomicDriveController(
            new com.pathplanner.lib.config.PIDConstants(3.1, 0.0, 0.0),
            new com.pathplanner.lib.config.PIDConstants(5.0, 0.0, 0.1),
            DRIVE_MAX_LINEAR_SPEED.in(Units.MetersPerSecond)
        );
    
        // NavX calibration
        while (DRIVETRAIN_HARDWARE.navx.isCalibrating()) DRIVETRAIN_HARDWARE.stop();

        DRIVETRAIN_HARDWARE.navx.reset();
    
        // Swerve drive kinematics and pose estimator
        KINEMATICS = new SwerveDriveKinematics(
            DRIVETRAIN_HARDWARE.lFrontModule.getModuleCoordinate(),
            DRIVETRAIN_HARDWARE.rFrontModule.getModuleCoordinate(),
            DRIVETRAIN_HARDWARE.lRearModule.getModuleCoordinate(),
            DRIVETRAIN_HARDWARE.rRearModule.getModuleCoordinate()
        );
    
        advancedKinematics = new AdvancedSwerveKinematics(DRIVETRAIN_HARDWARE.lFrontModule.getModuleCoordinate(),
                                                          DRIVETRAIN_HARDWARE.rFrontModule.getModuleCoordinate(),
                                                          DRIVETRAIN_HARDWARE.lRearModule.getModuleCoordinate(),
                                                          DRIVETRAIN_HARDWARE.rRearModule.getModuleCoordinate());

      POSE_ESTIMATOR = new SwerveDrivePoseEstimator(
          KINEMATICS,
          getRotation2d(),
          getModulePositions(),
          new Pose2d(),
          Constants.DriveConstants.ODOMETRY_STDDEV,
          Constants.DriveConstants.VISION_STDDEV
      );

      // Chassis speeds
      desiredChassisSpeeds = new ChassisSpeeds();

      // Field2d visualization
      FIELD = new Field2d();
      SmartDashboard.putData(FIELD);

      // Path logging for PathPlanner
      PathPlannerLogging.setLogActivePathCallback((poses) -> {
          if (poses.isEmpty()) return;
          var trajectory = TrajectoryGenerator.generateTrajectory(
              poses,
              new TrajectoryConfig(DRIVE_MAX_LINEAR_SPEED, DRIVE_AUTO_ACCELERATION)
          );
          FIELD.getObject("currentPath").setTrajectory(trajectory);
      });

      this.AUTO_AIM_PID_CONTROLLER_FRONT = new ProfiledPIDController(10.0, 0.0, 0.5, new TrapezoidProfile.Constraints(2160.0, 4320.0), 0.02);
      this.AUTO_AIM_PID_CONTROLLER_FRONT.enableContinuousInput(-180.0, +180.0);
      this.AUTO_AIM_PID_CONTROLLER_FRONT.setTolerance(1.5);
      this.AUTO_AIM_PID_CONTROLLER_BACK = new ProfiledPIDController(10.0, 0.0, 0.5, new TrapezoidProfile.Constraints(2160.0, 4320.0), 0.02);
      this.AUTO_AIM_PID_CONTROLLER_BACK.enableContinuousInput(-180.0, +180.0);
      this.AUTO_AIM_PID_CONTROLLER_BACK.setTolerance(1.5);
      this.X_VELOCITY_FILTER = new MedianFilter(100);
      this.Y_VELOCITY_FILTER = new MedianFilter(100);
    }                          
}

/**
 * Initialize hardware devices for drive subsystem
 * 
 * @return A Hardware object containing all necessary devices for this subsystem
 */
public static SwerveHardware initializeHardware() {
  RAWRNavX2 navx = new RAWRNavX2(Constants.DriveHardwareConstants.NAVX_ID);

  RAWRSwerveModule lFrontModule = RAWRSwerveModule.createSwerve(
          Constants.DriveHardwareConstants.LEFT_FRONT_DRIVE_MOTOR_ID,
          Constants.DriveHardwareConstants.LEFT_FRONT_ROTATE_MOTOR_ID,
          SwerveModule.Location.LeftFront);

  RAWRSwerveModule rFrontModule = RAWRSwerveModule.createSwerve(
          Constants.DriveHardwareConstants.RIGHT_FRONT_DRIVE_MOTOR_ID,
          Constants.DriveHardwareConstants.RIGHT_FRONT_ROTATE_MOTOR_ID,
          SwerveModule.Location.RightFront);

  RAWRSwerveModule lRearModule = RAWRSwerveModule.createSwerve(
          Constants.DriveHardwareConstants.LEFT_REAR_DRIVE_MOTOR_ID,
          Constants.DriveHardwareConstants.LEFT_REAR_ROTATE_MOTOR_ID,
          SwerveModule.Location.LeftRear);

  RAWRSwerveModule rRearModule = RAWRSwerveModule.createSwerve(
          Constants.DriveHardwareConstants.RIGHT_REAR_DRIVE_MOTOR_ID,
          Constants.DriveHardwareConstants.RIGHT_REAR_ROTATE_MOTOR_ID,
          SwerveModule.Location.RightRear);

    return new SwerveHardware(navx, lFrontModule, rFrontModule, lRearModule, rRearModule);
}

  /**
   * Set swerve modules
   * @param moduleStates Array of calculated module states
   */
  private void setSwerveModules(SwerveModuleState[] moduleStates) {
    DRIVETRAIN_HARDWARE.lFrontModule.set(moduleStates);
    DRIVETRAIN_HARDWARE.rFrontModule.set(moduleStates);
    DRIVETRAIN_HARDWARE.lRearModule.set(moduleStates);
    DRIVETRAIN_HARDWARE.rRearModule.set(moduleStates);
    Logger.recordOutput(getName() + Constants.DriveConstants.DESIRED_SWERVE_STATE_LOG_ENTRY, moduleStates);
  }

  /**
   * Drive the robot 
   *
   * @param xRequest         Desired X (forward) velocity
   * @param yRequest         Desired Y (sideways) velocity
   * @param rotateRequest    Desired rotate rate
   * @param controlCentricity Control centricity 
   * @param inertialVelocity Current robot inertial velocity (null if traction control is disabled)
   */
  private void drive(LinearVelocity xRequest,
                    LinearVelocity yRequest,
                    AngularVelocity rotateRequest,
                    ControlCentricity controlCentricity,
                    LinearVelocity inertialVelocity) {

      if (controlCentricity == null){
        controlCentricity = ControlCentricity.FIELD_CENTRIC;
      }
      
      // Get requested chassis speeds, correcting for second order kinematics
      desiredChassisSpeeds = AdvancedSwerveKinematics.correctForDynamics(
          new ChassisSpeeds(xRequest, yRequest, rotateRequest)
      );

      // Convert speeds to module states, correcting for 2nd order kinematics
      SwerveModuleState[] moduleStates = advancedKinematics.toSwerveModuleStates(
          desiredChassisSpeeds,
          getRotation2d(),
          controlCentricity
      );

      // Desaturate drive speeds
      SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DRIVE_MAX_LINEAR_SPEED);

      // Set modules to calculated states, applying traction control if enabled
      setSwerveModules(moduleStates);
  }

  /**
   * Get current module states
   * @return Array of swerve module states
   */
  private SwerveModuleState[] getModuleStates() {
     return new SwerveModuleState[] {
      DRIVETRAIN_HARDWARE.lFrontModule.getState(),
      DRIVETRAIN_HARDWARE.rFrontModule.getState(),
      DRIVETRAIN_HARDWARE.lRearModule.getState(),
      DRIVETRAIN_HARDWARE.rRearModule.getState()
    };
  }

  /**
   * Get current module positions
   * @return Array of swerve module positions
   */
  private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      DRIVETRAIN_HARDWARE.lFrontModule.getPosition(),
      DRIVETRAIN_HARDWARE.rFrontModule.getPosition(),
      DRIVETRAIN_HARDWARE.lRearModule.getPosition(),
      DRIVETRAIN_HARDWARE.rRearModule.getPosition()
    };
  }

  /**
   * Update robot pose
   */
  private void updatePose() {
    // Save previous pose
    m_previousPose = getPose();

    // Update pose based on odometry
    POSE_ESTIMATOR.update(getRotation2d(), getModulePositions());

    // Update current heading
    currentHeading = new Rotation2d(getPose().getX() - m_previousPose.getX(), getPose().getY() - m_previousPose.getY());
  }

  /**
   * Log SwerveDriveSubsystem outputs
   */
  private void logOutputs() {
    Logger.recordOutput(getName() + Constants.DriveConstants.POSE_LOG_ENTRY, getPose());
    Logger.recordOutput(getName() + Constants.DriveConstants.ACTUAL_SWERVE_STATE_LOG_ENTRY, getModuleStates());
  }

  /**
   * SmartDashboard indicators
   */
  private void smartDashboard() {
    FIELD.setRobotPose(getPose());
    SmartDashboard.putBoolean("FC", controlCentricity.equals(ControlCentricity.FIELD_CENTRIC));
  }

  /**
   * Start calling this repeatedly when robot is in danger of tipping over
   */
  private void antiTip() {
    // Calculate direction of tip
    double direction = Math.atan2(getRoll().in(Units.Degrees), getPitch().in(Units.Degrees));

    // Drive to counter tipping motion
    drive(
            DRIVE_MAX_LINEAR_SPEED.div(4).times(Math.cos(direction)),
      DRIVE_MAX_LINEAR_SPEED.div(4).times(Math.sin(direction)),
      Units.DegreesPerSecond.of(0.0), null, null
    );
  }

  /**
   * Aim robot at a desired point on the field
   * @param xRequest Desired X axis (forward) speed [-1.0, +1.0]
   * @param yRequest Desired Y axis (sideways) speed [-1.0, +1.0]
   * @param rotateRequest Desired rotate speed (ONLY USED IF POINT IS NULL) [-1.0, +1.0]
   * @param point Target point, pass in null to signify invalid point
   * @param controlCentricity True to point back of robot to target
   * @param velocityCorrection True to compensate for robot's own velocity
   */
  private void aimAtPoint(ControlCentricity controlCentricity, double xRequest, double yRequest, double rotateRequest, Translation2d point, boolean reversed, boolean velocityCorrection) {
    // Calculate desired robot velocity
    double moveRequest = Math.hypot(xRequest, yRequest);
    double moveDirection = Math.atan2(yRequest, xRequest);
    LinearVelocity velocityOutput = THROTTLE_MAP.throttleLookup(moveRequest);

    // Drive normally and return if invalid point
    if (point == null) {
      AngularVelocity rotateOutput = ROTATE_PID_CONTROLLER.calculate(getAngle(), getRotateRate(), rotateRequest).unaryMinus();
      drive(
        velocityOutput.unaryMinus().times(Math.cos(moveDirection)),
        velocityOutput.unaryMinus().times(Math.sin(moveDirection)),
        rotateOutput,
        controlCentricity,
        getInertialVelocity()
      );
      return;
    }

    // Adjust point
    point = point.plus(Constants.DriveConstants.AIM_OFFSET);
    // Get current pose
    Pose2d currentPose = getPose();
    // Angle to target point
    Rotation2d targetAngle = new Rotation2d(point.getX() - currentPose.getX(), point.getY() - currentPose.getY());
    // Movement vector of robot
    Vector2D robotVector = new Vector2D(velocityOutput.times(currentHeading.getCos()).magnitude(), velocityOutput.times(currentHeading.getSin()).magnitude());
    // Aim point
    Translation2d aimPoint = point.minus(new Translation2d(robotVector.getX(), robotVector.getY()));
    // Vector from robot to target
    Vector2D targetVector = new Vector2D(currentPose.getTranslation().getDistance(point) * targetAngle.getCos(), currentPose.getTranslation().getDistance(point) * targetAngle.getSin());
    // Parallel component of robot's motion to target vector
    Vector2D parallelRobotVector = targetVector.scalarMultiply(robotVector.dotProduct(targetVector) / targetVector.getNormSq());
    // Perpendicular component of robot's motion to target vector
    Vector2D perpendicularRobotVector = robotVector.subtract(parallelRobotVector).scalarMultiply(velocityCorrection ?Constants.DriveConstants. AIM_VELOCITY_COMPENSATION_FUDGE_FACTOR : 0.0);
    // Adjust aim point using calculated vector
    Translation2d adjustedPoint = point.minus(new Translation2d(perpendicularRobotVector.getX(), perpendicularRobotVector.getY()));
    // Calculate new angle using adjusted point
    Rotation2d adjustedAngle = new Rotation2d(adjustedPoint.getX() - currentPose.getX(), adjustedPoint.getY() - currentPose.getY());
    // Calculate necessary rotate rate
    double rotateOutput = reversed
      ? AUTO_AIM_PID_CONTROLLER_BACK.calculate(currentPose.getRotation().plus(Rotation2d.fromRadians(Math.PI)).getDegrees(), adjustedAngle.getDegrees())
      : AUTO_AIM_PID_CONTROLLER_FRONT.calculate(currentPose.getRotation().getDegrees(), adjustedAngle.getDegrees());

    // Log aim point
    Logger.recordOutput(getName() + "/AimPoint", new Pose2d(aimPoint, new Rotation2d()));
    double aimError = currentPose.getRotation().getDegrees() - adjustedAngle.getDegrees();
    Logger.recordOutput(getName() + "/AimError", Math.copySign(((180 - Math.abs(aimError)) % 180), (aimError)));

    // Drive robot accordingly
    drive(
      velocityOutput.unaryMinus().times(Math.cos(moveDirection)),
      velocityOutput.unaryMinus().times(Math.sin(moveDirection)),
      Units.DegreesPerSecond.of(rotateOutput),
      controlCentricity,
      getInertialVelocity()
    );
  }

  /**
   * Configure the auto builder
   */
  public void configureAutoBuilder() {
    AutoBuilder.configure(
            this::getPose,
            this::resetPose,
            this::getChassisSpeeds,
            (speeds, feedforwards) -> autoDrive(speeds),
            PATH_FOLLOWER_CONFIG,
            RobotUtils.robotConfig,
            () -> {
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this
    );
  }

  /**
   * Call this repeatedly to drive using PID during teleoperation
   * @param xRequest Desired X axis (forward) speed [-1.0, +1.0]
   * @param yRequest Desired Y axis (sideways) speed [-1.0, +1.0]
   * @param rotateRequest Desired rotate speed [-1.0, +1.0]
   */
  private void teleopPID(double xRequest, double yRequest, double rotateRequest) {
    // Calculate move request and direction
    double moveRequest = Math.hypot(xRequest, yRequest);
    double moveDirection = Math.atan2(yRequest, xRequest);

    // Get throttle and rotate output
    LinearVelocity velocityOutput = THROTTLE_MAP.throttleLookup(moveRequest);
    AngularVelocity rotateOutput = ROTATE_PID_CONTROLLER.calculate(getAngle(), getRotateRate(), rotateRequest).unaryMinus();

    // Drive robot
    drive(
      velocityOutput.unaryMinus().times(Math.cos(moveDirection)),
      velocityOutput.unaryMinus().times(Math.sin(moveDirection)),
      rotateOutput,
      controlCentricity,
      getInertialVelocity()
    );
  }

  /**
   * Reset pose estimator
   * @param pose Pose to set robot to
   */
  private void resetPose(Pose2d pose) {
    POSE_ESTIMATOR.resetPosition(
      getRotation2d(),
      getModulePositions(),
      pose
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    DRIVETRAIN_HARDWARE.navx.updateInputs();


    // Filter inertial velocity
    DRIVETRAIN_HARDWARE.navx.getInputs().velocityX = (Units.MetersPerSecond.of(
      X_VELOCITY_FILTER.calculate(DRIVETRAIN_HARDWARE.navx.getInputs().velocityX.in(Units.MetersPerSecond))
    )).mutableCopy();
    DRIVETRAIN_HARDWARE.navx.getInputs().velocityY = (Units.MetersPerSecond.of(
      Y_VELOCITY_FILTER.calculate(DRIVETRAIN_HARDWARE.navx.getInputs().velocityY.in(Units.MetersPerSecond))

    )).mutableCopy();

    

    updatePose();
    smartDashboard();
    logOutputs();
  }

  @Override
  public void simulationPeriodic() {

    updatePose();
    smartDashboard();
    logOutputs();
  }

  /**
   * Call this repeatedly to drive during autonomous
   * @param speeds Calculated swerve module states
   */
  public void autoDrive(ChassisSpeeds speeds) {
    // Get requested chassis speeds, correcting for second order kinematics
    desiredChassisSpeeds = AdvancedSwerveKinematics.correctForDynamics(speeds);

    // Convert speeds to module states, correcting for 2nd order kinematics
    SwerveModuleState[] moduleStates = advancedKinematics.toSwerveModuleStates(
            desiredChassisSpeeds,
      getRotation2d(),
      ControlCentricity.ROBOT_CENTRIC
    );

    // Desaturate drive speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DRIVE_MAX_LINEAR_SPEED);

    // Set modules to calculated states, WITHOUT traction control
    setSwerveModules(moduleStates);

    // Update turn PID
    ROTATE_PID_CONTROLLER.calculate(getAngle(), getRotateRate(), 0.0);

    // Update auto-aim controllers
    AUTO_AIM_PID_CONTROLLER_FRONT.calculate(
      getRotation2d().getDegrees(),
      getRotation2d().getDegrees()
    );
    AUTO_AIM_PID_CONTROLLER_BACK.calculate(
      getRotation2d().plus(Rotation2d.fromRadians(Math.PI)).getDegrees(),
      getRotation2d().plus(Rotation2d.fromRadians(Math.PI)).getDegrees()
    );
  }

  /**
   * Toggles between field centric and robot centric drive control
   */
  private void toggleControlCentricity() {
    if (controlCentricity == ControlCentricity.FIELD_CENTRIC) {
      this.controlCentricity = ControlCentricity.ROBOT_CENTRIC;
    } else {
      this.controlCentricity = ControlCentricity.FIELD_CENTRIC;
    }
  }

  /**
   * Aim robot at desired point on the field, while strafing
   * @param xRequestSupplier X axis speed supplier [-1.0, +1.0]
   * @param yRequestSupplier Y-axis speed supplier [-1.0, +1.0]
   * @param rotateRequestSupplier Rotate speed supplier (ONLY USED IF POINT IS NULL) [-1.0, +1.0]
   * @param pointSupplier Desired point supplier
   * @param reversed True to point rear of robot toward point
   * @param velocityCorrection True to compensate for robot's own velocity
   * @return Command that will aim at point while strafing
   */
  public Command aimAtPointCommand(DoubleSupplier xRequestSupplier, DoubleSupplier yRequestSupplier, DoubleSupplier rotateRequestSupplier,
                                   Supplier<Translation2d> pointSupplier, boolean reversed, boolean velocityCorrection) {
    return runEnd(() -> aimAtPoint(
            controlCentricity,
      xRequestSupplier.getAsDouble(),
      yRequestSupplier.getAsDouble(),
      rotateRequestSupplier.getAsDouble(),
      pointSupplier.get(),
      reversed,
      velocityCorrection
    ),
            this::resetRotatePID
    );
  }

  /**
   * Aim robot at desired point on the field, while strafing
   * @param xRequestSupplier X axis speed supplier [-1.0, +1.0]
   * @param yRequestSupplier Y axis speed supplier [-1.0, +1.0]
   * @param rotateRequestSupplier Rotate speed supplier (ONLY USED IF POINT IS NULL) [-1.0, +1.0]
   * @param point Desired point
   * @param reversed True to point rear of robot toward point
   * @param velocityCorrection True to compensate for robot's own velocity
   * @return Command that will aim at point while strafing
   */
  public Command aimAtPointCommand(DoubleSupplier xRequestSupplier, DoubleSupplier yRequestSupplier, DoubleSupplier rotateRequestSupplier,
                                   Translation2d point, boolean reversed, boolean velocityCorrection) {
    return aimAtPointCommand(xRequestSupplier, yRequestSupplier, rotateRequestSupplier, () -> point, reversed, velocityCorrection);
  }

  /**
   * Aim robot at desired point on the field
   * @param point Desired point
   * @param reversed True to point rear of robot toward point
   * @param velocityCorrection True to compensate for robot's own velocity
   * @return Command that will aim robot at point while strafing
   */
  public Command aimAtPointCommand(Translation2d point, boolean reversed, boolean velocityCorrection) {
    return aimAtPointCommand(() -> 0.0, () -> 0.0, () -> 0.0, () -> point, reversed, velocityCorrection);
  }


  /**
   * Drive the robot
   * @param xRequestSupplier X axis speed supplier
   * @param yRequestSupplier Y axis speed supplier
   * @param rotateRequestSupplier Rotate speed supplier
   * @return Command that will drive robot
   */
  public Command driveCommand(DoubleSupplier xRequestSupplier, DoubleSupplier yRequestSupplier, DoubleSupplier rotateRequestSupplier) {
    return run(() -> teleopPID(xRequestSupplier.getAsDouble(), yRequestSupplier.getAsDouble(), rotateRequestSupplier.getAsDouble()));
  }

  /**
   * Stop robot
   * @return Command to stop robot
   */
  public Command stopCommand() {
    return runOnce(() -> {
      DRIVETRAIN_HARDWARE.stop();
      resetRotatePID();
    });
  }

  /**
   * Toggle traction control
   * @return Command to toggle traction control
   */
  public Command toggleTractionControlCommand() {
    return runOnce(() -> DRIVETRAIN_HARDWARE.toggleTractionControl());  }

  /**
   * Toggles between field and robot oriented drive control
   * @return Command to toggle control centricity between robot and field centric drive control
   */
  public Command toggleCentricityCommand() {
    return runOnce(this::toggleControlCentricity);
  }

  /**
   * Enable traction control
   * @return Command to enable traction control
   */
  public Command enableTractionControlCommand() {
    return runOnce(() -> DRIVETRAIN_HARDWARE.enableTractionControl());  }

  /**
   * Disable traction control
   * @return Command to disable traction control
   */
  public Command disableTractionControlCommand() {
    return runOnce(() -> DRIVETRAIN_HARDWARE.disableTractionControl());  }

  /**
   * Reset pose estimator
   * @param poseSupplier Pose supplier
   * @return Command to reset pose
   */
  public Command resetPoseCommand(Supplier<Pose2d> poseSupplier) {
    return runOnce(() -> resetPose(poseSupplier.get()));
  }

  /**
   * @return Command to aim a point on the field in robot centric mode
   */
  public Command aimAtPointRobotCentric(DoubleSupplier xRequestSupplier, DoubleSupplier yRequestSupplier, DoubleSupplier rotateRequestSupplier,
                                        Supplier<Translation2d> pointSupplier, boolean reversed, boolean velocityCorrection) {
    return runEnd(() ->
      aimAtPoint(
        ControlCentricity.ROBOT_CENTRIC,
        xRequestSupplier.getAsDouble(),
        yRequestSupplier.getAsDouble(),
        rotateRequestSupplier.getAsDouble(),
        pointSupplier.get(),
        reversed,
        velocityCorrection
      ),
            this::resetRotatePID
    );

  }

  /**
   * Reset SwerveDriveSubsystem turn PID
   */
  public void resetRotatePID() {
    ROTATE_PID_CONTROLLER.setSetpoint(getAngle());
    ROTATE_PID_CONTROLLER.reset();
  }

    /**
   * Get constraints for path following
   * @return Path following constraints
   */
  public PathConstraints getPathConstraints() {
    return new PathConstraints(
      3.0,
      1.0,
      Constants.DriveConstants.DRIVE_ROTATE_VELOCITY.in(Units.RadiansPerSecond),
      Constants.DriveConstants.DRIVE_ROTATE_ACCELERATION.magnitude()
    );
  }

  /**
   * Get robot relative speeds
   * @return Robot relative speeds
   */
  public ChassisSpeeds getChassisSpeeds() {
    return KINEMATICS.toChassisSpeeds(getModuleStates());
  }

  /**
   * Get estimated robot pose
   * @return Currently estimated robot pose
   */
  public Pose2d getPose() {
    return POSE_ESTIMATOR.getEstimatedPosition();
  }

  /**
   * Get whether robot is tipping over
   * @return True if robot is tipping
   */
  public boolean isTipping() {
    return Math.abs(getPitch().in(Units.Degrees)) > Constants.DriveConstants.TIP_THRESHOLD ||
           Math.abs(getRoll().in(Units.Degrees)) > Constants.DriveConstants.TIP_THRESHOLD;
  }

  /**
   * Get whether robot is nearly balanced
   * @return True if robot is (nearly) balanced
   */
  public boolean isBalanced() {
    return Math.abs(getPitch().in(Units.Degrees)) < Constants.DriveConstants.BALANCED_THRESHOLD &&
           Math.abs(getRoll().in(Units.Degrees)) < Constants.DriveConstants.BALANCED_THRESHOLD;
  }

  /**
   * Get if robot is aimed at desired target
   * @return True if aimed
   */
  public boolean isAimed() {
    return (AUTO_AIM_PID_CONTROLLER_FRONT.atGoal() || AUTO_AIM_PID_CONTROLLER_BACK.atGoal()) && getRotateRate().lt(Constants.DriveConstants.AIM_VELOCITY_THRESHOLD);
  }

  /**
   * Get inertial velocity of robot
   * @return Inertial velocity of robot in m/s
   */
  public LinearVelocity getInertialVelocity() {
    return Units.MetersPerSecond.of(
      Math.hypot(DRIVETRAIN_HARDWARE.navx.getInputs().velocityX.in(Units.MetersPerSecond), DRIVETRAIN_HARDWARE.navx.getInputs().velocityY.in(Units.MetersPerSecond))
    );
  }

  /**
   * Get pitch of robot
   * @return Current pitch angle of robot in degrees
   */
  public Angle getPitch() {
    // Robot pitch axis is navX pitch axis
    return DRIVETRAIN_HARDWARE.navx.getInputs().pitchAngle;
  }

  /**
   * Get roll of robot
   * @return Current roll angle of robot in degrees
   */
  public Angle getRoll() {
    // Robot roll axis is navX roll axis
    return DRIVETRAIN_HARDWARE.navx.getInputs().rollAngle;
  }

  /**
   * Return the heading of the robot in degrees
   * @return Current heading of the robot in degrees
   */
  public Angle getAngle() {
    //System.out.println("NAVX ANGLE: " + DRIVETRAIN_HARDWARE.navx.getYaw().magnitude());

    return DRIVETRAIN_HARDWARE.navx.getYaw();
  }

  /**
   * Get rotate rate of robot
   * @return Current rotate rate of robot
   */
  public AngularVelocity getRotateRate() {
    return DRIVETRAIN_HARDWARE.navx.getInputs().yawRate;
  }

  /**
   * Return the heading of the robot as a Rotation2d.
   *
   * <p>The angle is expected to increase as the gyro turns counterclockwise when looked at from the
   * top. It needs to follow the NWU axis convention.
   *
   * @return Current heading of the robot as a Rotation2d.
   */
  public Rotation2d getRotation2d() {
    return DRIVETRAIN_HARDWARE.navx.getInputs().rotation2d;

  }

  @Override
  public void close() {
    DRIVETRAIN_HARDWARE.navx.close();
    DRIVETRAIN_HARDWARE.lFrontModule.close();
    DRIVETRAIN_HARDWARE.rFrontModule.close();
    DRIVETRAIN_HARDWARE.lRearModule.close();
    DRIVETRAIN_HARDWARE.rRearModule.close();

  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.rawrobotics;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.lasarobotics.drive.swerve.AdvancedSwerveKinematics.ControlCentricity;
import org.lasarobotics.drive.swerve.child.MAXSwerveModule;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.utils.PIDConstants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.rawrobotics.common.swerve.RAWRNavX2;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  /**
   * Constants for controllers
   * <br>
   * {@link HIDConstants#PRIMARY_CONTROLLER_PORT} is for the driver,
   * <br> <br>
   * {@link HIDConstants#SECONDARY_CONTROLLER_PORT} is for the operator
   */
  public static class HIDConstants {

  public static final int PRIMARY_CONTROLLER_PORT = 0;
  public static final int SECONDARY_CONTROLLER_PORT = 1;
  public static final double CONTROLLER_DEADBAND = 0.6;

  public static final CommandXboxController DRIVER_CONTROLLER = new CommandXboxController(
    PRIMARY_CONTROLLER_PORT);
  public static final CommandXboxController OPERATOR_CONTROLLER = new CommandXboxController(
    SECONDARY_CONTROLLER_PORT);

  }

  public static class SwerveConstants {
    public static final Time DEFAULT_SIGNAL_PERIOD = Units.Milliseconds.of(10.0);

    public static final double EPSILON = 5e-3;
    /**In Amps, the max current a Drive Motor can use*/
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;
    /**In Amps, the max current a Rotate Motor can use*/
    public static final int ROTATE_MOTOR_CURRENT_LIMIT = 20;

    /**The Gear Ratio used for our swerve modules*/
    public static final MAXSwerveModule.GearRatio GEAR_RATIO = MAXSwerveModule.GearRatio.L3;

    // Log
    public static final String ROTATE_ERROR_LOG_ENTRY = "/RotateError";
    public static final String MAX_LINEAR_VELOCITY_LOG_ENTRY = "/MaxLinearVelocity";
    public static final double MAX_AUTO_LOCK_TIME = 10.0;
    
  }

  public static class DriveConstants {
    // Drive specs
    public static final double DRIVETRAIN_EFFICIENCY = 0.90;
    public static final PIDConstants DRIVE_ROTATE_PID = PIDConstants.of(4.0, 0.0, 0.05, 0.0, 0.0);
    public static final double DRIVE_SLIP_RATIO = 0.05;
    public static final double DRIVE_TURN_SCALAR = 60.0;
    public static final double DRIVE_LOOKAHEAD = 0.0;
    public static final double DRIVE_WHEELBASE = 0.5588;
    public static final double DRIVE_TRACK_WIDTH = 0.5588;
    public static final double AUTO_LOCK_TIME = 3.0;
    public static final AngularVelocity DRIVE_ROTATE_VELOCITY = Units.RadiansPerSecond.of(12.0 * Math.PI);
    public static final AngularVelocity AIM_VELOCITY_THRESHOLD = Units.DegreesPerSecond.of(5.0);
    public static final AngularAcceleration DRIVE_ROTATE_ACCELERATION = Units.RadiansPerSecond.of(4.0 * Math.PI).per(Units.Second);
    public static final Translation2d AIM_OFFSET = new Translation2d(0.0, -0.5);

    // Other settings
    public static final double TIP_THRESHOLD = 35.0;
    public static final double BALANCED_THRESHOLD = 10.0;
    public static final double AIM_VELOCITY_COMPENSATION_FUDGE_FACTOR = 0.1;
    public static final Matrix<N3, N1> ODOMETRY_STDDEV = VecBuilder.fill(0.03, 0.03, Math.toRadians(1.0));
    public static final Matrix<N3, N1> VISION_STDDEV = VecBuilder.fill(1.0, 1.0, Math.toRadians(3.0));

    // Log
    public static final String POSE_LOG_ENTRY = "/Pose";
    public static final String ACTUAL_SWERVE_STATE_LOG_ENTRY = "/ActualSwerveState";
    public static final String DESIRED_SWERVE_STATE_LOG_ENTRY = "/DesiredSwerveState";
    
    public static final ControlCentricity DRIVE_CONTROL_CENTRICITY = ControlCentricity.FIELD_CENTRIC;

    // Input Curves
    private static final double[] m_DriveThrottleInputCurveX = { 0.0, 0.100, 0.200, 0.300, 0.400, 0.500, 0.600, 0.700, 0.800, 0.900, 1.000 };
    private static final double[] m_DriveThrottleInputCurveY = { 0.0, 0.052, 0.207, 0.465, 0.827, 1.293, 1.862, 2.534, 3.310, 4.189, 5.172 };
    private static final double[] m_DriveTurnInputCurveX = { 0.0, 0.100, 0.200, 0.300, 0.400, 0.500, 0.600, 0.700, 0.800, 0.900, 1.0 };
    private static final double[] m_DriveTurnInputCurveY = { 0.0, 0.010, 0.050, 0.100, 0.150, 0.200, 0.250, 0.300, 0.400, 0.600, 1.0 };

    private static final SplineInterpolator SPLINE_INTERPOLATOR = new SplineInterpolator();


    public static final PolynomialSplineFunction DRIVE_THROTTLE_INPUT_CURVE = SPLINE_INTERPOLATOR
        .interpolate(m_DriveThrottleInputCurveX, m_DriveThrottleInputCurveY);
    public static final PolynomialSplineFunction DRIVE_TURN_INPUT_CURVE = SPLINE_INTERPOLATOR
        .interpolate(m_DriveTurnInputCurveX, m_DriveTurnInputCurveY);
  }


  /** Constants for the physical hardware for drive 
   * (Swerve Modules, etc)
   * 
   * @author PurpleLib
   * @since 2024
   */
  public static class DriveHardwareConstants {
    public static final RAWRNavX2.ID NAVX_ID = new RAWRNavX2.ID("DriveHardware/NavX2");

    public static final Spark.ID LEFT_FRONT_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/LeftFront/Drive", 5);
    public static final Spark.ID LEFT_FRONT_ROTATE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/LeftFront/Rotate", 6);

    public static final Spark.ID RIGHT_FRONT_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/RightFront/Drive", 3);
    public static final Spark.ID RIGHT_FRONT_ROTATE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/RightFront/Rotate", 4);

    public static final Spark.ID LEFT_REAR_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/LeftRear/Drive", 7);
    public static final Spark.ID LEFT_REAR_ROTATE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/LeftRear/Rotate", 8);
    
    public static final Spark.ID RIGHT_REAR_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/RightRear/Drive", 1);
    public static final Spark.ID RIGHT_REAR_ROTATE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/RightRear/Rotate", 2);
  }


  public static class SmartDashboardConstants {
    public static final String SMARTDASHBOARD_DEFAULT_TAB = "SmartDashboard";
    public static final String SMARTDASHBOARD_AUTO_MODE = "Auto Mode";
  }

}
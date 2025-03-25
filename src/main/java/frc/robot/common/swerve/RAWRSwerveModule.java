package frc.robot.common.swerve;

// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

import java.time.Duration;
import java.time.Instant;
import java.util.Map;
import java.lang.Runtime;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

import frc.robot.Constants;
import org.lasarobotics.drive.TractionControlController;
import org.lasarobotics.drive.swerve.DriveWheel;
import org.lasarobotics.drive.swerve.SwerveModule;
import org.lasarobotics.drive.swerve.SwerveModuleSim;
import org.lasarobotics.drive.swerve.parent.REVSwerveModule;
import org.lasarobotics.hardware.PurpleManager;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;
import org.lasarobotics.utils.FFConstants;
import org.lasarobotics.utils.GlobalConstants;
import org.lasarobotics.utils.PIDConstants;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.common.components.RobotUtils;
import lombok.AllArgsConstructor;

/** REV MAXSwerve module */
public class RAWRSwerveModule extends SwerveModule implements Sendable {
  /**
   * REV swerve module hardware
   */
  @AllArgsConstructor
  public static class Hardware {
    Spark driveMotor;
    Spark rotateMotor;
  }

  private final double DRIVE_TICKS_PER_METER;
  private final double DRIVE_METERS_PER_TICK;
  private final double DRIVE_METERS_PER_ROTATION;
  private final double DRIVE_MAX_LINEAR_SPEED;

  private final Spark m_driveMotor;
  private final Spark m_rotateMotor;
  private final SwerveModuleSim m_moduleSim;
  private SimpleMotorFeedforward m_driveFF;
  private final Rotation2d m_zeroOffset;
  
  private final SwerveModule.Location m_location;
  private Rotation2d m_previousRotatePosition;

  private volatile double m_simDrivePosition;
  private volatile SwerveModulePosition m_simModulePosition;
  private volatile SwerveModuleState m_desiredState;

  private final double m_autoLockTime;

  private Instant m_autoLockTimer;

  private final ExecutorService EXECUTOR_SERVICE = Executors.newFixedThreadPool(2);  
    
  public static final Map<SwerveModule.Location, Angle> ZERO_OFFSET = Map.ofEntries(
    Map.entry(SwerveModule.Location.LeftFront, Units.Radians.of(Math.PI / 2)),
    Map.entry(SwerveModule.Location.RightFront, Units.Radians.zero()),
    Map.entry(SwerveModule.Location.LeftRear, Units.Radians.of(Math.PI)),
    Map.entry(SwerveModule.Location.RightRear, Units.Radians.of(Math.PI / 2))
  );
  
  
    /** Make a {@link REVSwerveModule}
     * 
     * @param driveMotor the drive motor (Ex: forward-backword)
     * @param rotateMotor the rotate motor (Ex: left-right)
     * @param location the location of the swerve module (Ex: front left)
     * 
     * @author Hudson Strub
     * @since 2025
     */
    public static RAWRSwerveModule createSwerve(Spark.ID driveMotor, Spark.ID rotateMotor, SwerveModule.Location location){
      
      RAWRSwerveModule.Hardware hardware = new RAWRSwerveModule.Hardware(
                    new Spark(driveMotor, Spark.MotorKind.NEO_VORTEX),
                    new Spark(rotateMotor, Spark.MotorKind.NEO_550)
            );
    
      
      RAWRSwerveModule swerveModule = new RAWRSwerveModule(
            hardware,
            location,
            SwerveModule.MountOrientation.STANDARD,
            SwerveModule.MountOrientation.INVERTED,
            Constants.SwerveConstants.GEAR_RATIO,
            DriveWheel.create(
              Distance.ofRelativeUnits(75, Units.Millimeter), 
              Dimensionless.ofBaseUnits(1.6, Units.Value),
              Dimensionless.ofBaseUnits(1.3, Units.Value)), 
            ZERO_OFFSET.get(location),
            PIDConstants.of(0.18, 0, 0.001, 0.174, 0), // Replace with actual PID constants
          FFConstants.of(0, 0, 0, 0),  // Replace with actual feed-forward constants
          PIDConstants.of(2.1, 0, 0.2, 0, 0), // The PID for the rotate Motor
          FFConstants.of(0, 0, 0, 0),  // Replace with actual feed-forward constants
          Dimensionless.ofBaseUnits(DriveConstants.DRIVE_SLIP_RATIO, Units.Value),
          Mass.ofRelativeUnits(RobotUtils.robotConfig.massKG, Units.Kilograms),
          Distance.ofRelativeUnits(23, Units.Inches),
          Distance.ofRelativeUnits(24.5, Units.Inches),
          Time.ofBaseUnits(Constants.DriveConstants.AUTO_LOCK_TIME, Units.Second));
    
    
    return swerveModule;
  }

  /**
   * Create an instance of a REV swerve module
   * @param swerveHardware Hardware devices required by swerve module
   * @param location Module location
   * @param motorOrientation Motor mount orientation
   * @param encoderOrientation Encoder orientation
   * @param gearRatio Module gear ratio
   * @param driveWheel Wheel installed in swerve module
   * @param zeroOffset Chassis zero offset due to module calibration position
   * @param drivePID Drive motor PID gains
   * @param driveFF Drive motor feed forward gains
   * @param rotatePID Rotate motor PID gains
   * @param rotateFF Rotate motor feed forward gains
   * @param slipRatio Desired slip ratio [1%, 40%]
   * @param mass Robot mass
   * @param wheelbase Robot wheelbase
   * @param trackWidth Robot track width
   * @param autoLockTime Time before automatically rotating module to locked position (10 seconds max)
   */
  private RAWRSwerveModule(Hardware swerveHardware,
                         SwerveModule.Location location,
                         SwerveModule.MountOrientation motorOrientation,
                         SwerveModule.MountOrientation encoderOrientation,
                         SwerveModule.GearRatio gearRatio,
                         DriveWheel driveWheel, Angle zeroOffset,
                         PIDConstants drivePID, FFConstants driveFF,
                         PIDConstants rotatePID, FFConstants rotateFF,
                         Dimensionless slipRatio, Mass mass,
                         Distance wheelbase, Distance trackWidth,
                         Time autoLockTime) {
    super(location, gearRatio, driveWheel, zeroOffset, wheelbase, trackWidth, swerveHardware.driveMotor.getID().name);
    int encoderTicksPerRotation = swerveHardware.driveMotor.getKind().equals(MotorKind.NEO_VORTEX)
      ? GlobalConstants.VORTEX_ENCODER_TICKS_PER_ROTATION
      : GlobalConstants.NEO_ENCODER_TICKS_PER_ROTATION;
  
    DRIVE_TICKS_PER_METER =
      (encoderTicksPerRotation * gearRatio.getDriveRatio())
      * (1 / (driveWheel.diameter.in(Units.Meters) * Math.PI));
    DRIVE_METERS_PER_TICK = 1 / DRIVE_TICKS_PER_METER;
    DRIVE_METERS_PER_ROTATION = DRIVE_METERS_PER_TICK * encoderTicksPerRotation;
    DRIVE_MAX_LINEAR_SPEED = (swerveHardware.driveMotor.getKind().getMaxRPM() / 60) * DRIVE_METERS_PER_ROTATION * DriveConstants.DRIVETRAIN_EFFICIENCY;

    // Set traction control controller
    super.setTractionControlController(new TractionControlController(driveWheel, slipRatio, mass, Units.MetersPerSecond.of(DRIVE_MAX_LINEAR_SPEED)));

    this.m_driveMotor = swerveHardware.driveMotor;
    this.m_rotateMotor = swerveHardware.rotateMotor;
    this.m_moduleSim = new SwerveModuleSim(
      m_driveMotor.getKind().motor,
      driveFF.withKA((driveFF.kA <= 0.0) ? SwerveModule.MIN_SIM_kA : driveFF.kA),
      m_rotateMotor.getKind().motor, rotateFF.withKA((rotateFF.kA <= 0.0) ? SwerveModule.MIN_SIM_kA : rotateFF.kA)
    );
    this.m_driveFF = new SimpleMotorFeedforward(driveFF.kS, driveFF.kV, driveFF.kA);
    this.m_location = location;
    this.m_zeroOffset = Rotation2d.fromRadians(zeroOffset.in(Units.Radians));
    this.m_simDrivePosition = 0.0;
    this.m_simModulePosition = new SwerveModulePosition();
    this.m_desiredState = new SwerveModuleState(Units.MetersPerSecond.of(0.0), m_zeroOffset.plus(m_location.getLockPosition()));
    this.m_autoLockTime = MathUtil.clamp(autoLockTime.in(Units.Milliseconds), 0.0, SwerveConstants.MAX_AUTO_LOCK_TIME * 1000);
    this.m_previousRotatePosition = m_zeroOffset.plus(m_location.getLockPosition());
    this.m_autoLockTimer = Instant.now();

    Logger.recordOutput(m_driveMotor.getID().name + SwerveConstants.MAX_LINEAR_VELOCITY_LOG_ENTRY, DRIVE_MAX_LINEAR_SPEED);

    //Config Drive Motor 
    EXECUTOR_SERVICE.submit(() -> configDrive(driveWheel, motorOrientation, drivePID));

    // Reset the Drive Encoder
    EXECUTOR_SERVICE.submit(() -> resetDriveEncoder());

    //Config Rotate Motor
    EXECUTOR_SERVICE.submit(() -> configRotate(motorOrientation, encoderOrientation, rotatePID));

    // Add callbacks to PurpleManager
    PurpleManager.addCallback(() -> periodic());
    PurpleManager.addCallbackSim(() -> simulationPeriodic());

    Runtime.getRuntime().addShutdownHook(new Thread(() -> shutdownExecutorService()));
  }

  public void configDrive(DriveWheel driveWheel, SwerveModule.MountOrientation motorOrientation,  PIDConstants drivePID) {
    SparkBaseConfig m_driveMotorConfig;
    
    // Configure the drive motor
    m_driveMotorConfig = (m_driveMotor.getKind().equals(MotorKind.NEO_VORTEX)) ? new SparkFlexConfig() : new SparkMaxConfig();

    // Set drive encoder config
    double m_driveConversionFactor = driveWheel.diameter.in(Units.Meters) * Math.PI / super.getGearRatio().getDriveRatio();
    m_driveMotorConfig.encoder.positionConversionFactor(m_driveConversionFactor);
    m_driveMotorConfig.encoder.velocityConversionFactor(m_driveConversionFactor / 60);

    // Invert drive motor if necessary
    m_driveMotorConfig.inverted(motorOrientation.equals(MountOrientation.INVERTED));

    // Set sensor to use for closed loop control
    m_driveMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    // Set gains for drive PID
    m_driveMotorConfig.closedLoop.pidf(
      drivePID.kP,
      drivePID.kI,
      drivePID.kD,
      drivePID.kF
    );

    // Set drive motor to coast
    m_driveMotorConfig.idleMode(IdleMode.kCoast);

    // Set current limits
    m_driveMotorConfig.smartCurrentLimit(SwerveConstants.DRIVE_MOTOR_CURRENT_LIMIT);

    // Set status frame rates
    m_driveMotorConfig.signals.primaryEncoderPositionPeriodMs(23);
    m_driveMotorConfig.signals.primaryEncoderVelocityPeriodMs(20);
    m_driveMotorConfig.signals.absoluteEncoderPositionPeriodMs(20);
    m_driveMotorConfig.signals.absoluteEncoderVelocityPeriodMs(20);
    m_driveMotorConfig.signals.analogPositionPeriodMs(20);
    m_driveMotorConfig.signals.analogVelocityPeriodMs(20);
    m_driveMotorConfig.signals.limitsPeriodMs(10);

    // Configure the drive motor with the desired config
    m_driveMotor.configure(m_driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
}

public void configRotate(SwerveModule.MountOrientation motorOrientation, SwerveModule.MountOrientation encoderOrientation, PIDConstants rotatePID) {
  SparkBaseConfig m_rotateMotorConfig;

  // Configure the rotate motor
  m_rotateMotorConfig = (m_rotateMotor.getKind().equals(MotorKind.NEO_VORTEX)) ? new SparkFlexConfig() : new SparkMaxConfig();

  // Set rotate encoder config
  double m_rotateConversionFactor = 6.28318530718;
  m_rotateMotorConfig.absoluteEncoder.positionConversionFactor(m_rotateConversionFactor);
  m_rotateMotorConfig.absoluteEncoder.velocityConversionFactor(m_rotateConversionFactor / 60);
  m_rotateMotorConfig.absoluteEncoder.inverted(!encoderOrientation.equals(motorOrientation));

  // Set sensor to use for closed loop control
  m_rotateMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

  // Set gains for rotate PID and enable wrapping
  m_rotateMotorConfig.closedLoop.pid(rotatePID.kP, rotatePID.kI, rotatePID.kD);
  m_rotateMotorConfig.closedLoop.positionWrappingEnabled(true);
  m_rotateMotorConfig.closedLoop.positionWrappingInputRange(0.0, m_rotateConversionFactor);

  // Set rotate motor to brake
  m_rotateMotorConfig.idleMode(IdleMode.kBrake);

  // Set current limits
  m_rotateMotorConfig.smartCurrentLimit(SwerveConstants.ROTATE_MOTOR_CURRENT_LIMIT);

  // Set status frame rates
  m_rotateMotorConfig.signals.primaryEncoderPositionPeriodMs(23);
  m_rotateMotorConfig.signals.primaryEncoderVelocityPeriodMs(20);
  m_rotateMotorConfig.signals.absoluteEncoderPositionPeriodMs(20);
  m_rotateMotorConfig.signals.absoluteEncoderVelocityPeriodMs(20);
  m_rotateMotorConfig.signals.analogPositionPeriodMs(20);
  m_rotateMotorConfig.signals.analogVelocityPeriodMs(20);
  m_rotateMotorConfig.signals.limitsPeriodMs(10);

  // Configure the rotate motor with the desired config
  m_rotateMotor.configure(m_rotateMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
}


  private void shutdownExecutorService() {
    try {
        EXECUTOR_SERVICE.shutdown();
        if (!EXECUTOR_SERVICE.awaitTermination(60, TimeUnit.SECONDS)) {
          EXECUTOR_SERVICE.shutdownNow();
        }
    } catch (InterruptedException e) {
      EXECUTOR_SERVICE.shutdownNow();
    }
  }

  /**
   * Update position in simulation
   */
  void updateSimPosition() {
    m_simDrivePosition += m_desiredState.speedMetersPerSecond * GlobalConstants.ROBOT_LOOP_HZ.asPeriod().in(Units.Seconds);
    synchronized (m_driveMotor.getInputs()) {
      m_driveMotor.getInputs().encoderPosition = m_simDrivePosition;
      m_driveMotor.getInputs().encoderVelocity = m_desiredState.speedMetersPerSecond;
      synchronized (m_rotateMotor.getInputs()) {
        m_rotateMotor.getInputs().absoluteEncoderPosition = m_desiredState.angle.getRadians();
        m_simModulePosition = new SwerveModulePosition(m_simDrivePosition, m_desiredState.angle);
      }
    }
  }

  /**
   * Call this method periodically
   */
  private void periodic() {
    super.logOutputs();
    Logger.recordOutput(m_rotateMotor.getID().name + SwerveConstants.ROTATE_ERROR_LOG_ENTRY, m_desiredState.angle.minus(Rotation2d.fromRadians(m_rotateMotor.getInputs().absoluteEncoderPosition)));
  }

 /**
   * Call this method periodically in simulation
   */
  private void simulationPeriodic() {
    var vbus = Units.Volts.of(RobotController.getBatteryVoltage());
    m_driveMotor.getSim().enable();
    m_rotateMotor.getSim().enable();

    m_driveMotor.getSim().iterate(m_moduleSim.getDriveMotorVelocity().in(Units.RPM), vbus.in(Units.Volts), GlobalConstants.ROBOT_LOOP_HZ.asPeriod().in(Units.Seconds));
    m_rotateMotor.getSim().iterate(m_moduleSim.getRotateMotorVelocity().in(Units.RPM), vbus.in(Units.Volts), GlobalConstants.ROBOT_LOOP_HZ.asPeriod().in(Units.Seconds));

    m_moduleSim.update(
      vbus.times(m_driveMotor.getSim().getAppliedOutput()),
      vbus.times(m_rotateMotor.getSim().getAppliedOutput())
    );

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_moduleSim.getTotalCurrentDraw().in(Units.Amps)));

    m_driveMotor.getSim().setMotorCurrent(m_moduleSim.getDriveMotorCurrentDraw().in(Units.Amps));
    m_rotateMotor.getSim().setMotorCurrent(m_moduleSim.getRotateMotorCurrentDraw().in(Units.Amps));

    updateSimPosition();
  }

  /**
   * Allow for adding swerve module as a sendable object for dashboard interactivity
   * @param builder
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSafeState(this::lock);
    builder.setActuator(true);
    // Control drive velocity
    builder.addDoubleProperty(
      "Velocity",
      () -> m_desiredState.speedMetersPerSecond,
      (value) -> set(new SwerveModuleState(Units.MetersPerSecond.of(value), m_desiredState.angle))
    );
    // Control rotation
    builder.addDoubleProperty(
      "Orientation",
      () -> m_desiredState.angle.getRadians(),
      (value) -> set(new SwerveModuleState(m_desiredState.speedMetersPerSecond, Rotation2d.fromRadians(value)))
    );
    // Configure drive kS
    builder.addDoubleProperty(
      "Drive kS",
      () -> m_driveFF.getKs(),
      (value) -> {
        m_driveFF = new SimpleMotorFeedforward(value, m_driveFF.getKv(), m_driveFF.getKa());
      }
    );
    // Configure drive kV
    builder.addDoubleProperty(
      "Drive kV",
      () -> m_driveFF.getKv(),
      (value) -> {
        m_driveFF = new SimpleMotorFeedforward(m_driveFF.getKs(), value, m_driveFF.getKa());
      }
    );
    // Configure drive kA
    builder.addDoubleProperty(
      "Drive kA",
      () -> m_driveFF.getKa(),
      (value) -> {
        m_driveFF = new SimpleMotorFeedforward(m_driveFF.getKs(), m_driveFF.getKv(), value);
      }
    );
  }

  @Override
  public void enableBrakeMode(boolean enable) {
    m_driveMotor.setIdleMode((enable) ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void set(SwerveModuleState state) {
    // Auto lock modules if auto lock enabled, speed not requested, and time has elapsed
    if (super.isAutoLockEnabled() && state.speedMetersPerSecond < SwerveConstants.EPSILON) {
      state.speedMetersPerSecond = 0.0;
      // Time's up, lock now...
      if (Duration.between(m_autoLockTimer, Instant.now()).toMillis() > m_autoLockTime)
        state.angle = m_location.getLockPosition();
      // Waiting to lock...
      else state.angle = m_previousRotatePosition.minus(m_zeroOffset);
    } else {
      // Not locking this loop, restart timer...
      m_autoLockTimer = Instant.now();
    }

    // Save previous state
    var oldState = m_desiredState;

    // Get desired state
    m_desiredState = super.getDesiredState(state, Rotation2d.fromRadians(m_rotateMotor.getInputs().absoluteEncoderPosition));

    // Set rotate motor position
    m_rotateMotor.set(m_desiredState.angle.getRadians(), ControlType.kPosition);

    // Calculate drive FF
    var driveFF = m_driveFF.calculateWithVelocities(
      oldState.speedMetersPerSecond,
      m_desiredState.speedMetersPerSecond
    );

    // Set drive motor speed
    m_driveMotor.set(
      m_desiredState.speedMetersPerSecond, ControlType.kVelocity,
      driveFF, ArbFFUnits.kVoltage
    );

    // Save rotate position
    m_previousRotatePosition = m_desiredState.angle;

    // Increment odometer
    super.incrementOdometer(Math.abs(m_desiredState.speedMetersPerSecond) * GlobalConstants.ROBOT_LOOP_HZ.asPeriod().in(Units.Seconds));
  }

  @Override
  public LinearVelocity getDriveVelocity() {
    return Units.MetersPerSecond.of(m_driveMotor.getInputs().encoderVelocity);
  }

  @Override
  public SwerveModuleState getState() {
    return new SwerveModuleState(
      getDriveVelocity(),
      Rotation2d.fromRadians(m_rotateMotor.getInputs().absoluteEncoderPosition).minus(m_zeroOffset)
    );
  }

  @Override
  public SwerveModulePosition getPosition() {
    if (RobotBase.isSimulation()) return m_simModulePosition;
    return new SwerveModulePosition(
      m_driveMotor.getInputs().encoderPosition,
      Rotation2d.fromRadians(m_rotateMotor.getInputs().absoluteEncoderPosition).minus(m_zeroOffset)
    );
  }

  @Override
  public void resetDriveEncoder() {
    m_driveMotor.resetEncoder();
    m_simDrivePosition = 0.0;
    m_simModulePosition = new SwerveModulePosition(m_simDrivePosition, m_previousRotatePosition);
  }

  @Override
  public void stop() {
    m_rotateMotor.stopMotor();
    m_driveMotor.stopMotor();
  }

  @Override
  public void close() {
    m_driveMotor.close();
    m_rotateMotor.close();
  }

  @Override
  public void setDriveSysID(Voltage volts) {
    throw new UnsupportedOperationException("Unimplemented method 'setDriveSysID'");
  }

  @Override
  public void setRotateSysID(Voltage volts) {
    throw new UnsupportedOperationException("Unimplemented method 'setRotateSysID'");
  }

  @Override
  public Frequency getUpdateRate() {
    throw new UnsupportedOperationException("Unimplemented method 'getUpdateRate'");
  }
}

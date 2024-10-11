// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class MAXSwerveModule {
  private final TalonFX m_drivingKraken;
  private final CANSparkMax m_turningSparkMax;

 // private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkPIDController m_turningPIDController;
  private VelocityDutyCycle m_velocityPID = new VelocityDutyCycle(0);
  public  Orchestra  m_music = new Orchestra(); 
  private DutyCycleOut m_openLoop = new DutyCycleOut(0);
  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());
  private boolean velocityControlBoolean = false;
  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingKraken = new TalonFX(drivingCANId, "rio");
    m_turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);


    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    m_turningSparkMax.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);

    m_turningPIDController = m_turningSparkMax.getPIDController();
    m_turningPIDController.setFeedbackDevice(m_turningEncoder);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    m_turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
    m_turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    m_turningEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    m_turningPIDController.setPositionPIDWrappingEnabled(true);
    m_turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
    m_turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

    // m_velocityPID = new VelocityDutyCycle(0);

    m_drivingKraken.setInverted(false);
  
    // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!

    // var slot0Configs = new Slot0Configs();
    //   slot0Configs.kS = ModuleConstants.kDrivingFF; // Add 0.25 V output to overcome static friction
    //   slot0Configs.kP = ModuleConstants.kDrivingP; // A position error of 2.5 rotations results in 12 V output
    //   slot0Configs.kI = ModuleConstants.kDrivingI; // no output for integrated error
    //   slot0Configs.kD = ModuleConstants.kDrivingD; // A velocity error of 1 rps results in 0.1 V output

      

    // var currentConfig = new CurrentLimitsConfigs();
    //   currentConfig.StatorCurrentLimit = ModuleConstants.kDrivingMotorCurrentLimit;
    //   currentConfig.StatorCurrentLimitEnable = true;

    // m_drivingKraken.getConfigurator().apply(slot0Configs);
    // m_drivingKraken.getConfigurator().apply(currentConfig);

    var talonFxConfigs = new TalonFXConfiguration();
    talonFxConfigs.Slot0.kP = ModuleConstants.kDrivingP;
    talonFxConfigs.Slot0.kI = ModuleConstants.kDrivingI;
    talonFxConfigs.Slot0.kD = ModuleConstants.kDrivingD;
    // talonFxConfigs.CurrentLimits.SupplyCurrentLimit = ModuleConstants.kDrivingMotorCurrentLimit;
    // talonFxConfigs.MotorOutput.Inverted = false;
    talonFxConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonFxConfigs.Feedback.SensorToMechanismRatio = ModuleConstants.kDrivingMotorReduction / ModuleConstants.kWheelCircumferenceMeters;
    // talonFxConfigs.Feedback.SensorToMechanismRatio = ;
    m_drivingKraken.getConfigurator().apply(talonFxConfigs);


    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_turningPIDController.setP(ModuleConstants.kTurningP);
    m_turningPIDController.setI(ModuleConstants.kTurningI);
    m_turningPIDController.setD(ModuleConstants.kTurningD);
    m_turningPIDController.setFF(ModuleConstants.kTurningFF);
    m_turningPIDController.setOutputRange(ModuleConstants.kTurningMinOutput,
        ModuleConstants.kTurningMaxOutput);

    m_turningSparkMax.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
    m_turningSparkMax.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_turningSparkMax.burnFlash();

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingKraken.setPosition(0);
    m_drivingKraken.setNeutralMode(NeutralModeValue.Brake);

    m_music.addInstrument(m_drivingKraken);


  }

  public void periodic() {
    // Update the state of the PID controller.
    SmartDashboard.putNumber("velocity", m_drivingKraken.getVelocity().getValueAsDouble());
  }

  public void playSong(String pathname){
    m_music.loadMusic(pathname);
    m_music.play();

  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingKraken.getVelocity().getValue(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  public double cosineScale(Rotation2d currentAngle, Rotation2d desiredAngle) {
    return desiredAngle.minus(currentAngle).getCos();
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingKraken.getPosition().getValue(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(m_turningEncoder.getPosition()));

    SmartDashboard.putNumber("Speed",  optimizedDesiredState.speedMetersPerSecond);
    
  
    m_drivingKraken.setControl(
      m_velocityPID
          .withVelocity(
              optimizedDesiredState.speedMetersPerSecond * 
              Math.abs(cosineScale(Rotation2d.fromRadians(m_turningEncoder.getPosition()), correctedDesiredState.angle))
          )
          .withEnableFOC(true)
    );    
    // m_drivingKraken.setControl(m_velocityPID.withVelocity(optimizedDesiredState.speedMetersPerSecond));

    // m_drivingKraken.setControl(m_velocityPID.withVelocity((optimizedDesiredState.speedMetersPerSecond / 0.1016 / Math.PI) * 2048 * 3.56));
    m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    m_desiredState = desiredState;
  }

  public void velocityControlEnabled(boolean m_state) {
    velocityControlBoolean = m_state;
  }



  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingKraken.setPosition(0);
  }
}

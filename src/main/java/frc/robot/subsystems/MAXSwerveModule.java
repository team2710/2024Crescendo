// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ModuleConstants;

/**
 * MAXSwerveModule manages a single swerve drive module, including dynamic Field-Oriented Control (FOC)
 * toggling to simulate gear shifting based on speed and acceleration, along with smoothing mechanisms
 * to ensure seamless transitions without jerking or slippage.
 */
public class MAXSwerveModule {
    // === Motor Controllers ===
    private final TalonFX m_drivingKraken;
    private final CANSparkMax m_turningSparkMax;

    // === Encoders ===
    private final AbsoluteEncoder m_turningEncoder;

    // === PID Controllers ===
    private final SparkPIDController m_turningPIDController;
    private final VelocityDutyCycle m_velocityPID = new VelocityDutyCycle(0);

    // === Orchestra (for music playback, unrelated to gear shifting) ===
    public final Orchestra m_music = new Orchestra(); 

    // === Control Variables ===
    private final DutyCycleOut m_openLoop = new DutyCycleOut(0);
    private final double m_chassisAngularOffset;
    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());
    private boolean velocityControlBoolean = false;

    // === Gear Shifting Parameters ===
    // Speed thresholds in meters per second
    private static final double HIGH_GEAR_SPEED_THRESHOLD = 5.5;    // Shift to High Gear above this speed
    private static final double LOW_GEAR_SPEED_THRESHOLD = 4.5;     // Shift to Low Gear below this speed

    // Acceleration/Deceleration thresholds in meters per second squared
    private static final double ACCELERATION_THRESHOLD = 2.0;      // Engage FOC during acceleration above this rate
    private static final double DECELERATION_THRESHOLD = -2.0;     // Engage FOC during deceleration below this rate

    // Smoothing parameters
    private static final double TRANSITION_TIME = 0.5;             // Transition duration in seconds
    private static final double UPDATE_RATE = 0.02;                // Control loop update rate (50Hz)
    private static final double LOW_PASS_ALPHA = 0.2;              // Alpha for low-pass filter

    // === Gear States ===
    private enum GearState {
        HIGH_GEAR,            // FOC Disabled
        LOW_GEAR,             // FOC Enabled
        TRANSITION_TO_HIGH,   // Transitioning to High Gear
        TRANSITION_TO_LOW     // Transitioning to Low Gear
    }

    private GearState currentGearState = GearState.LOW_GEAR;     // Initial gear state

    // === Transition Tracking ===
    private double transitionTimer = 0.0;

    // === Acceleration Tracking ===
    private double previousSpeed = 0.0;

    // === Debounce Counters ===
    private static final int DEBOUNCE_COUNT = 5; // Number of consistent cycles required
    private int highGearDebounce = 0;
    private int lowGearDebounce = 0;

    // === Low-Pass Filter Variables ===
    private double filteredSpeed = 0.0;
    private double filteredAcceleration = 0.0;

    // === Module Identifier ===
    private final int m_drivingCANId; // Stored CAN ID for SmartDashboard labeling

    /**
     * Constructs a MAXSwerveModule and configures the driving and turning motor,
     * encoder, and PID controller. This configuration is specific to the REV
     * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
     * Encoder.
     *
     * @param drivingCANId          CAN ID for the driving TalonFX motor controller
     * @param turningCANId          CAN ID for the turning CANSparkMax motor controller
     * @param chassisAngularOffset  Angular offset for the chassis (radians)
     */
    public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
        // Store the drivingCANId for SmartDashboard labeling
        this.m_drivingCANId = drivingCANId;

        // Initialize motor controllers
        m_drivingKraken = new TalonFX(drivingCANId, "rio");
        m_turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

        // Factory reset for turning SparkMax
        m_turningSparkMax.restoreFactoryDefaults();

        // Setup encoders and PID controllers for the turning SparkMax
        m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
        m_turningPIDController = m_turningSparkMax.getPIDController();
        m_turningPIDController.setFeedbackDevice(m_turningEncoder);

        // Apply position and velocity conversion factors for the turning encoder
        m_turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
        m_turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

        // Invert the turning encoder if necessary
        m_turningEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);

        // Enable PID wrap around for the turning motor
        m_turningPIDController.setPositionPIDWrappingEnabled(true);
        m_turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
        m_turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

        // Configure the driving TalonFX motor
        TalonFXConfiguration talonFxConfigs = new TalonFXConfiguration();
        talonFxConfigs.Slot0.kP = ModuleConstants.kDrivingP;
        talonFxConfigs.Slot0.kI = ModuleConstants.kDrivingI;
        talonFxConfigs.Slot0.kD = ModuleConstants.kDrivingD;
        talonFxConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFxConfigs.Feedback.SensorToMechanismRatio = ModuleConstants.kDrivingMotorReduction / ModuleConstants.kWheelCircumferenceMeters;
        m_drivingKraken.getConfigurator().apply(talonFxConfigs);

        // Set PID gains for the turning motor
        m_turningPIDController.setP(ModuleConstants.kTurningP);
        m_turningPIDController.setI(ModuleConstants.kTurningI);
        m_turningPIDController.setD(ModuleConstants.kTurningD);
        m_turningPIDController.setFF(ModuleConstants.kTurningFF);
        m_turningPIDController.setOutputRange(ModuleConstants.kTurningMinOutput,
                ModuleConstants.kTurningMaxOutput);

        // Configure the turning SparkMax motor
        m_turningSparkMax.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
        m_turningSparkMax.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

        // Save the SparkMax configurations
        m_turningSparkMax.burnFlash();

        // Initialize chassis angular offset and reset motor positions
        m_chassisAngularOffset = chassisAngularOffset;
        m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
        m_drivingKraken.setPosition(0);
        m_drivingKraken.setNeutralMode(NeutralModeValue.Brake);

        // Add the driving Kraken to the orchestra for music playback (if applicable)
        m_music.addInstrument(m_drivingKraken);
    }

    /**
     * Periodic method called once per scheduler run.
     */
    public void periodic() {
        // Update the state of the PID controller on the SmartDashboard
        SmartDashboard.putNumber("Module " + m_drivingCANId + " Driving Velocity (m/s)", m_drivingKraken.getVelocity().getValueAsDouble());
        Logger.recordOutput("Module " + m_drivingCANId + " Gear State", currentGearState.toString());

        // Optional: Update additional telemetry or perform regular checks here
    }

    /**
     * Plays a song using the Orchestra.
     *
     * @param pathname Path to the song file.
     */
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

    /**
     * Returns the cosine scale based on the angle difference.
     *
     * @param currentAngle The current angle of the module.
     * @param desiredAngle The desired angle of the module.
     * @return Cosine of the angle difference.
     */
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
     * Sets the desired state for the module with integrated gear-shifting logic.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // === 1. Apply Chassis Angular Offset ===
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

        // === 2. Optimize Reference State ===
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
                new Rotation2d(m_turningEncoder.getPosition()));

        SmartDashboard.putNumber("Module " + m_drivingCANId + " Optimized Speed (m/s)", optimizedDesiredState.speedMetersPerSecond);

        // === 3. Calculate Acceleration with Low-Pass Filter ===
        double currentSpeed = optimizedDesiredState.speedMetersPerSecond;
        // Apply low-pass filter to speed
        filteredSpeed = LOW_PASS_ALPHA * currentSpeed + (1 - LOW_PASS_ALPHA) * filteredSpeed;
        // Calculate acceleration based on filtered speed
        filteredAcceleration = (filteredSpeed - previousSpeed) / UPDATE_RATE;
        previousSpeed = filteredSpeed;
        SmartDashboard.putNumber("Module " + m_drivingCANId + " Acceleration (m/s²)", filteredAcceleration);

        // === 4. Gear Shifting Logic with Debouncer ===
        switch (currentGearState) {
            case HIGH_GEAR:
                // Shift to Low Gear if speed below threshold or decelerating rapidly
                if (filteredSpeed < LOW_GEAR_SPEED_THRESHOLD || filteredAcceleration < DECELERATION_THRESHOLD) {
                    lowGearDebounce++;
                    if (lowGearDebounce >= DEBOUNCE_COUNT) {
                        initiateShift(GearState.TRANSITION_TO_LOW);
                        lowGearDebounce = 0;
                    }
                } else {
                    lowGearDebounce = 0;
                }
                break;

            case LOW_GEAR:
                // Shift to High Gear if speed above threshold or accelerating rapidly
                if (filteredSpeed > HIGH_GEAR_SPEED_THRESHOLD || filteredAcceleration > ACCELERATION_THRESHOLD) {
                    highGearDebounce++;
                    if (highGearDebounce >= DEBOUNCE_COUNT) {
                        initiateShift(GearState.TRANSITION_TO_HIGH);
                        highGearDebounce = 0;
                    }
                } else {
                    highGearDebounce = 0;
                }
                break;

            case TRANSITION_TO_HIGH:
            case TRANSITION_TO_LOW:
                // Handle ongoing transition
                handleTransition();
                break;
        }

        // === 5. Determine FOC State ===
        boolean enableFOC = false;
        switch (currentGearState) {
            case LOW_GEAR:
            case TRANSITION_TO_LOW:
                enableFOC = true;
                break;
            case HIGH_GEAR:
            case TRANSITION_TO_HIGH:
                enableFOC = false;
                break;
        }

        // === 6. Apply Velocity Control with FOC ===
        m_drivingKraken.setControl(
                m_velocityPID
                        .withVelocity(
                                optimizedDesiredState.speedMetersPerSecond * 
                                Math.abs(cosineScale(new Rotation2d(m_turningEncoder.getPosition()), correctedDesiredState.angle))
                        )
                        .withEnableFOC(enableFOC)
        );

        // === 7. Set Reference for Turning ===
        m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

        // === 8. Update Desired State ===
        m_desiredState = desiredState;
    }

    /**
     * Initiates a gear shift to the target state.
     *
     * @param targetState The gear state to shift to.
     */
    private void initiateShift(GearState targetState) {
        if (currentGearState == targetState ||
                (currentGearState == GearState.TRANSITION_TO_HIGH && targetState == GearState.TRANSITION_TO_HIGH) ||
                (currentGearState == GearState.TRANSITION_TO_LOW && targetState == GearState.TRANSITION_TO_LOW)) {
            return; // Already in target state or transitioning
        }

        currentGearState = targetState;
        transitionTimer = 0.0;

        SmartDashboard.putString("Module " + m_drivingCANId + " Gear", 
                targetState == GearState.TRANSITION_TO_HIGH ? "Transitioning to High Gear" : "Transitioning to Low Gear");
        System.out.println("Module " + m_drivingCANId + ": Initiating shift to " + 
                (targetState == GearState.TRANSITION_TO_HIGH ? "High Gear" : "Low Gear"));
    }

    /**
     * Handles the gear transition by updating the transition timer and completing the shift when appropriate.
     */
    private void handleTransition() {
        transitionTimer += UPDATE_RATE;
        double transitionProgress = transitionTimer / TRANSITION_TIME;

        if (transitionProgress >= 1.0) {
            // Complete the transition
            if (currentGearState == GearState.TRANSITION_TO_HIGH) {
                completeShift(GearState.HIGH_GEAR);
            } else if (currentGearState == GearState.TRANSITION_TO_LOW) {
                completeShift(GearState.LOW_GEAR);
            }
        }
    }

    /**
     * Completes the gear shift by setting the current gear state to the target state.
     *
     * @param targetState The gear state to complete the shift to.
     */
    private void completeShift(GearState targetState) {
        currentGearState = targetState;
        SmartDashboard.putString("Module " + m_drivingCANId + " Gear", 
                targetState == GearState.HIGH_GEAR ? "High Gear" : "Low Gear");
        System.out.println("Module " + m_drivingCANId + ": Shifted to " + 
                (targetState == GearState.HIGH_GEAR ? "High Gear" : "Low Gear"));
    }

    /**
     * Enables or disables velocity control.
     *
     * @param m_state True to enable velocity control, false to disable.
     */
    public void velocityControlEnabled(boolean m_state) {
        velocityControlBoolean = m_state;
    }

    /** 
     * Zeroes all the SwerveModule encoders.
     */
    public void resetEncoders() {
        m_drivingKraken.setPosition(0);
    }

}

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.subsystems.LEDSubsystem.LEDState;

import com.revrobotics.SparkPIDController;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation; 


/**
 * The EndEffector class represents the subsystem responsible for controlling the end effector of the robot.
 * It includes functionality for controlling the intake, flywheel, and feeding mechanism.
 */
public class EndEffector extends SubsystemBase {


  public Rev2mDistanceSensor distanceSensor = new Rev2mDistanceSensor(Rev2mDistanceSensor.Port.kMXP);
  private CANSparkFlex intakeMotor = new CANSparkFlex(EndEffectorConstants.kIntakeMotorID, MotorType.kBrushless);
  private CANSparkFlex flywheelMotorTop = new CANSparkFlex(EndEffectorConstants.kFlywheelMotorTopID,
      MotorType.kBrushless);
  private CANSparkFlex flywheelMotorBottom = new CANSparkFlex(EndEffectorConstants.kFlywheelMotorBottomID,
      MotorType.kBrushless);
  private SparkPIDController pidController;
  private RelativeEncoder encoderTop;
  private RelativeEncoder encoderBottom;

  private double flywheelSpeed = EndEffectorConstants.kFlywheelMotorSpeed;

  private boolean isFlywheelRunning = false;
  private boolean isIntaking = false;
  private boolean isOuttaking = false;
  private boolean isFeeding = false;
  private boolean isNote = false;
  private double setpoint = 6000;

  LinearFilter m_currenFilter = LinearFilter.singlePoleIIR(0.1, 0.02);


  private PS4Controller m_driveController;
  private Climb m_Climb;
  private LEDSubsystem m_LedSubsystem;

  public EndEffector(PS4Controller driveController, LEDSubsystem ledSubsystem, Climb climb) {
    m_LedSubsystem = ledSubsystem;
    m_driveController = driveController;
    m_Climb = climb;
    // RainbowAnimation rainbowAnimation = new RainbowAnimation(1, 1, 0);
    // led.animate(rainbowAnimation);
    pidController = flywheelMotorTop.getPIDController();
    flywheelMotorBottom.follow(flywheelMotorTop, true);
    distanceSensor.setAutomaticMode(true);
    // intakeMotor.setSmartCurrentLimit(120);
    // flywheelMotorBottom.setSmartCurrentLimit(40);
    // flywheelMotorTop.setSmartCurrentLimit(40);

    encoderTop = flywheelMotorTop.getEncoder();
    encoderBottom = flywheelMotorBottom.getEncoder();

    SmartDashboard.putNumber("Flywheel P Gain", EndEffectorConstants.kFlywheelP);
    SmartDashboard.putNumber("Flywheel I Gain", EndEffectorConstants.kFlywheelI);
    SmartDashboard.putNumber("Flywheel D Gain", EndEffectorConstants.kFlywheelD);
    SmartDashboard.putNumber("Flywheel Feed Forward", EndEffectorConstants.kFlywheelFF);
    SmartDashboard.putNumber("Flywheel I Zone", EndEffectorConstants.kFlywheelIz);
    SmartDashboard.putNumber("Flywheel Min Output", EndEffectorConstants.kFlywheelMinOutput);
    SmartDashboard.putNumber("FLywheel Max Output", EndEffectorConstants.kFlywheelMaxOutput);

    SmartDashboard.putNumber("Flywheel Speed", flywheelSpeed);

    pidController.setP(EndEffectorConstants.kFlywheelP);
    pidController.setI(EndEffectorConstants.kFlywheelI);
    pidController.setD(EndEffectorConstants.kFlywheelD);
    pidController.setFF(EndEffectorConstants.kFlywheelFF);
    pidController.setIZone(EndEffectorConstants.kFlywheelIz);
    pidController.setOutputRange(EndEffectorConstants.kFlywheelMinOutput, EndEffectorConstants.kFlywheelMaxOutput);

    pidController.setSmartMotionAccelStrategy(SparkPIDController.AccelStrategy.kSCurve, 0);
    pidController.setSmartMotionAllowedClosedLoopError(5, 0);
    pidController.setSmartMotionMaxVelocity(EndEffectorConstants.kFlywheelMotorSpeed, 0);

    flywheelMotorTop.setIdleMode(IdleMode.kBrake);
    flywheelMotorBottom.setIdleMode(IdleMode.kCoast);
    intakeMotor.setIdleMode(IdleMode.kBrake);

    intakeMotor.burnFlash();
    flywheelMotorTop.burnFlash();
  }

  public void ampFlywheel(){
    feed();
    flywheelMotorTop.set(-0.5);
  }



  public Command toggleIntakeCommand() {
    return new InstantCommand(() -> {
      toggleIntake();
    }, this);
  }

  public Command intakeCommand() {
    return new InstantCommand(() -> {
      intake();
    }, this);
  }


  public Command toggleFlywheelCommand() {
    return new InstantCommand(() -> {
      toggleFlywheel();
    }, this);
  }

  public Command flywheelCommand() {
    return new InstantCommand(() -> {
      spinupFlywheel();
    }, this);
  }

 public Command flywheelAutoCommand() {
    return new InstantCommand(() -> {
      spinupFlywheelAuto();
    }, this);
  }


  public Command stopFlywheelCommand() {
    return new InstantCommand(() -> {
      stopFlywheel();
    });
  }

  public Command toggleOuttakeCommand() {
    return new InstantCommand(() -> {
      toggleOuttake();
    }, this);
  }

  public Command outtakeCommand() {
    return new InstantCommand(() -> {
      outtake();
    });
  }

  public Command feedCommand() {
    return new InstantCommand(() -> {
      feed();
    }, this);
  }

  public Command stopIntakeCommand() {
    return new InstantCommand(() -> {
      stopIntake();
    }, this);
  }

  // public Command funnyBlinkCommand() {
  //   return new InstantCommand(() -> {
  //     m_LedSubsystem.setState(LEDState.NOTE_DETECTED)
  //   }, this);
  // }

  // Command zeepleCommand = Commands.sequence(m_LedSubsystem.setState(LEDState.NOTE_DETECTED),
  //                                     Commands.waitSeconds(0.5),
  //                                     m_LedSubsystem.setState(LEDState.FIRE_EFFECT))
  // CommandScheduler.getInstance().ScheduleCommand(zeepleCommand);

  public double getFilterCurrent(){
    return m_currenFilter.calculate(intakeMotor.getOutputCurrent());
  }

  public void intake() {
    if (isNote) return;
    isIntaking = true;
    isOuttaking = false;
    intakeMotor.set(EndEffectorConstants.kIntakeSpeed);
    if (!isFlywheelRunning)
        flywheelMotorTop.set(0.05);
  }


  public void intakeAuto() {
    intakeMotor.set(EndEffectorConstants.kIntakeSpeed);
    flywheelMotorTop.set(0.05);
  }

  public void feed() {
    isIntaking = false;
    isFeeding = true;
    intakeMotor.set(1);
  }

  public void toggleIntake() {
    if (isIntaking)
      stopIntake();
    else
      intake();
  }

  public void outtake() {
    isOuttaking = true;
    isIntaking = false;
    intakeMotor.set(-0.22);
    if (!isFlywheelRunning)
        flywheelMotorTop.set(0.07);
  }

  public void toggleOuttake() {
    if (isOuttaking)
      stopIntake();
    else
      outtake();
  }

  public void stopIntake() {
    intakeMotor.set(0);
    if (!isFlywheelRunning)
        flywheelMotorTop.set(0);
    isIntaking = false;
    isOuttaking = false;
    isFeeding = false;
  }

  public void toggleFlywheel() {
    if (isFlywheelRunning)
      stopFlywheel();
    else
      spinupFlywheel();
  }

  public void intakeBeamBreak(){
    if(distanceSensor.getRange() < 0.1){
      stopIntake();
    }
  }

  public boolean isNoteDetected(){
    return distanceSensor.getRange() < 8 && distanceSensor.getRange() > 0;
  }

  

  public void spinupFlywheel() {
    isFlywheelRunning = true;

    //uncomment for smart pid control and for auto aim to work properly and accurately
    // pidController.setReference(setpoint, CANSparkMax.ControlType.kSmartMotion);
    flywheelMotorTop.set(-0.7);

    // flywheelMotorBottom.set(-1);
  }

  public void spinupFlywheelAuto() {
    isFlywheelRunning = true;


    flywheelMotorTop.set(-0.9);
    // flywheelMotorBottom.set(-1);
  }

  public void stopFlywheel() {
    isFlywheelRunning = false;
    flywheelMotorTop.set(0);
    flywheelMotorBottom.set(0);
  }

  public boolean getIsIntaking() {
    return isIntaking;
  }

  public boolean getIsFlywheelRunning() {
    return isFlywheelRunning;
  }

  public double flywheelRPM() {
    return Math.abs(flywheelMotorBottom.getEncoder().getVelocity());
  }

  public boolean atSubwooferShootingSpeed(){
    return 2500 < flywheelRPM();
  }
  
  public boolean atShootingSpeed(){
    return 4500 < flywheelRPM();
  }

  public boolean AtShootingSpeed(double speed){
    return speed < flywheelRPM();
  }
  

  @Override
  public void periodic() {
    isNote = isNoteDetected();
    SmartDashboard.putBoolean("Note Detect", isNote);
    SmartDashboard.putNumber("Distance Value", distanceSensor.getRange());

    SmartDashboard.putNumber("Flywheel Top RPM", encoderTop.getVelocity());
    SmartDashboard.putNumber("Flywheel Bottom RPM", encoderBottom.getVelocity());
    SmartDashboard.putNumber("Intake Current", getFilterCurrent());
    SmartDashboard.putNumber("Distance", distanceSensor.getRange());
    SmartDashboard.putBoolean("Intake State", isIntaking);
    SmartDashboard.putBoolean("Flywheel State", isFlywheelRunning);
    SmartDashboard.putBoolean("Outtake State", isOuttaking);
    SmartDashboard.putBoolean("Note Detect", isNote);
    SmartDashboard.putNumber("Flywheel Speed", flywheelSpeed);
    SmartDashboard.putNumber("Flywheel Setpoint", setpoint);
    SmartDashboard.putBoolean("Note beam break", isNoteDetected());

    Logger.recordOutput("IntakeCurrent", getFilterCurrent());
    Logger.recordOutput("FlywheelTopRPM", encoderTop.getVelocity());
    Logger.recordOutput("FlywheelBottomRPM", encoderBottom.getVelocity());
    Logger.recordOutput("Distance", distanceSensor.getRange());
    Logger.recordOutput("IntakeState", isIntaking);
    Logger.recordOutput("FlywheelState", isFlywheelRunning);
    Logger.recordOutput("OuttakeState", isOuttaking);
    Logger.recordOutput("NoteBeamBreak", isNoteDetected());

    if (isFlywheelRunning) {
      m_LedSubsystem.setState(LEDState.FLYWHEELS_ON);
    }
    else if (isNote) {
      m_LedSubsystem.setState(LEDState.NOTE_DETECTED);
    }
    else if (isIntaking) {
      m_LedSubsystem.setState(LEDState.INTAKING);
    }
    else if (isFeeding) {
      m_LedSubsystem.setState(LEDState.SCORE);
    }
    else if (m_Climb.isClimbing) {
      m_LedSubsystem.setState(LEDState.CLIMB);
    }
    else {
      m_LedSubsystem.setState(LEDState.FIRE_EFFECT);
    }

    // m_driveController.setRumble(RumbleType.kBothRumble, 0.5);
  }
}
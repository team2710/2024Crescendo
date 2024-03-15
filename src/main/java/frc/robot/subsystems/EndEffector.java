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
import com.revrobotics.SparkPIDController;
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
  private boolean isNote = false;
  private double setpoint = 6000;
  CANdle led = new CANdle(45); 

  private Debouncer intakDebouncer = new Debouncer(0.08, DebounceType.kRising);

  public enum IntakeState {
    OFF,
    On,
    Outtake,
    Feed,
  };

  public enum FlywheelState {
    OFF,
    On,
  };

  
  public enum LightState {
    HasNote,
    Idle,
    ReadyForNote,
    FlyWheelOn,
    IntakeOn
  };

  public static IntakeState m_intakeState = IntakeState.OFF;
  public static FlywheelState m_shootState = FlywheelState.OFF;
  LinearFilter m_currenFilter = LinearFilter.singlePoleIIR(0.1, 0.02);


  private PS4Controller m_driveController;

  public EndEffector(PS4Controller driveController) {
    m_driveController = driveController;
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
    flywheelMotorBottom.setIdleMode(IdleMode.kBrake);
    intakeMotor.setIdleMode(IdleMode.kBrake);

    intakeMotor.burnFlash();
    flywheelMotorTop.burnFlash();
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
  
  

  // public void currentDetectionIntake() {
  //   double INTAKE_STALL_DETECTION = 45;
  //   Debouncer debounce = new Debouncer(1);
  //   intake();
  //   if (getIsFlywheelRunning()) {
  //     isNote = false;
  //   }
  //   if (isIntaking || isNote) {
  //     if (debounce.calculate(intakeMotor.getOutputCurrent() > INTAKE_STALL_DETECTION)) {
  //       isNote = true;
  //       stopIntake();
  //     }
  //   }
  // }

  public boolean currentDetectIntake() {
    double INTAKE_STALL_DETECTION = 50;
    if (intakDebouncer.calculate(intakeMotor.getOutputCurrent() < INTAKE_STALL_DETECTION)) {
      return true;
    }
    return false;
  }

  // /**
  //  * Sets the intake state based on the controller input.
  //  * 
  //  * @param controller the PS4 controller object used for input
  //  */
  // public void IntakeSetter(PS4Controller controller) {
  //   Debouncer delay = new Debouncer(0.5);
    
  //   switch (m_intakeState) {
  //     case OFF:
  //       stopIntake();
  //       if(controller.getTriangleButton()) {
  //         m_intakeState = IntakeState.On;
  //       }
  //       else if(controller.getR1Button()) {
  //         m_intakeState = IntakeState.Shoot;
  //       } else {
  //         stopFlywheel();
  //       }
  //       break;

  //     case On: 
  //       if(isNote) {
  //         stopIntake();
  //         m_intakeState = IntakeState.OFF;
  //       }
  //       else {
  //         intake();
  //         isNote = currentDetectIntake();
  //       }
  //       break;
        
  //     case Shoot:
  //     spinupFlywheel();
  //     if(MathUtil.isNear(setpoint, encoderTop.getVelocity() + encoderBottom.getVelocity() * 0.5, 50.0)) {
  //       intake();
  //       isNote = false;
  //       if(delay.calculate(!isNote)) {
  //         m_intakeState = IntakeState.OFF;
  //       }
  //     }

  //     break;
  //   default:
  //   // should never be reached, as liftState should never be null
  //   m_intakeState = IntakeState.OFF;
  //   break;
  //   }

  // }

  /**
   * Sets the intake state of the end effector.
   * 
   * @param m_IntakeState the desired intake state
   */
  public void IntakeSetter() {
    switch (m_intakeState) {
      case OFF:
        stopIntake();
        break;

      case On: 
        if(isNote) {
          stopIntake();
          m_intakeState = IntakeState.OFF;
        }
        else {
          intake();
          isNote = currentDetectIntake();
        }
        break;

      case Outtake:
        outtake();
        break;
      case Feed:
        feed();
        isNote = false;
        break;
    default:
      m_intakeState = IntakeState.OFF;
      break;
    }

  }

public void LightSetter(LightState m_state) {
  switch(m_state) {
    case Idle:
      led.setLEDs(160, 32, 240);
      //  led.animate(new FireAnimation(2, 1.0, 5 * 3, 1.0, 1.0, false, 8));
      break;
    case IntakeOn:
      led.setLEDs(0, 0, 0);
      break;
    case HasNote:
      led.setLEDs(0, 0, 0);
      break;
    case ReadyForNote:
      led.setLEDs(0, 0, 0);
      break;
    case FlyWheelOn:
      led.setLEDs(0, 0, 0);
      break;
    default:
      m_state = LightState.Idle;
  } 
}

public void ShootSetter() {
    switch (m_shootState) {
      case OFF:
        stopFlywheel();
        break;
      case On: 
        if(isNote) {
          stopIntake();
          m_intakeState = IntakeState.OFF;
        }
        else {
          intake();
          // isNote = currentDetectIntake();
        }
        break;
    default:
      m_shootState = FlywheelState.OFF;
      break;
    }

  }

  public void feed() {
    isIntaking = false;
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
    intakeMotor.set(-0.2);
    if (!isFlywheelRunning)
        flywheelMotorTop.set(0.05);
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

  public void setIntakeState(IntakeState intakeState) {
    m_intakeState = intakeState;
  }

  public void setShooterState(FlywheelState shootState) {
    m_shootState = shootState;
  }

  public double flywheelRPM() {
    return Math.abs(flywheelMotorBottom.getEncoder().getVelocity());
  }

  public boolean atSubwooferShootingSpeed(){
    return 2500 < flywheelRPM();
  }

  public boolean AtShootingSpeed(double speed){
    return speed < flywheelRPM();
  }

  @Override
  public void periodic() {
    isNote = isNoteDetected();
    SmartDashboard.putBoolean("Note Detect", isNote);
    SmartDashboard.putNumber("Distance Value", distanceSensor.getRange());
    // if (intakDebouncer.calculate(getFilterCurrent() > 65) && isIntaking) {
    //   isNote = true;
    //   // stopIntake();
    // } else if (isNote)
    // {
    //   isNote = false;
    //   stopIntake();
    // }

    // double p = SmartDashboard.getNumber("Flywheel P Gain",
    // EndEffectorConstants.kFlywheelP);
    // double i = SmartDashboard.getNumber("Flywheel I Gain",
    // EndEffectorConstants.kFlywheelI);
    // double d = SmartDashboard.getNumber("Flywheel D Gain",
    // EndEffectorConstants.kFlywheelD);
    // double ff = SmartDashboard.getNumber("Flywheel Feed Forward",
    // EndEffectorConstants.kFlywheelFF);
    // double iz = SmartDashboard.getNumber("Flywheel I Zone",
    // EndEffectorConstants.kFlywheelIz);
    // double min = SmartDashboard.getNumber("Flywheel Min Output",
    // EndEffectorConstants.kFlywheelMinOutput);
    // double max = SmartDashboard.getNumber("FLywheel Max Output",
    // EndEffectorConstants.kFlywheelMaxOutput)
    
    // IntakeSetter();
    // ShootSetter();

    // if (p != pidController.getP()) pidController.setP(p);
    // if (i != pidController.getI()) pidController.setI(i);
    // if (d != pidController.getD()) pidController.setD(d);
    // if (ff != pidController.getFF()) pidController.setFF(ff);
    // if (iz != pidController.getIZone()) pidController.setIZone(iz);
    // if (min != pidController.getOutputMin() || max !=
    // pidController.getOutputMax()) pidController.setOutputRange(min, max);

    // flywheelSpeed = SmartDashboard.getNumber("Flywheel Speed", flywheelSpeed);

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

    // led.animate(new FireAnimation(2, 1.0, 15, 1.0, 1.0, false, 8));
      led.setLEDs(255, 0, 0);
      //     led.setLEDs(0, 255, 0);
      // led.setLEDs(0, 0, 255);

    // if (isNote && isIntaking) {
    //   // CommandScheduler.getInstance().schedule(
    //   //   Commands.sequence(
    //   //     stopIntakeCommand()
    //   //   )
    //   // );
    //   stopIntake();
    // }


    m_driveController.setRumble(RumbleType.kBothRumble, 0.5);

  //   if(isNoteDetected()) {
  //     LightSetter(LightState.HasNote);
  //   } else if(isFlywheelRunning) {
  //     LightSetter(LightState.FlyWheelOn);
  //   } else if(isIntaking) {
  //     LightSetter(LightState.IntakeOn);
  //   } else {
  //     LightSetter(LightState.Idle);
  //   }
  // }
  }
}
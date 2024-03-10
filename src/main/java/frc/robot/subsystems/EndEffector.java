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
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;
import com.revrobotics.SparkPIDController;

/**
 * The EndEffector class represents the subsystem responsible for controlling the end effector of the robot.
 * It includes functionality for controlling the intake, flywheel, and feeding mechanism.
 */
public class EndEffector extends SubsystemBase {


  public Rev2mDistanceSensor distanceSensor = new Rev2mDistanceSensor(Rev2mDistanceSensor.Port.kOnboard);
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

  public static IntakeState m_intakeState = IntakeState.OFF;
  public static FlywheelState m_shootState = FlywheelState.OFF;
  LinearFilter m_currenFilter = LinearFilter.singlePoleIIR(0.1, 0.02);


  public EndEffector() {
    pidController = flywheelMotorTop.getPIDController();
    flywheelMotorBottom.follow(flywheelMotorTop, true);

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
    flywheelMotorTop.burnFlash();
  }

  public Command toggleIntakeCommand() {
    return new InstantCommand(() -> {
      toggleIntake();
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
    return distanceSensor.getRange() < 0.1;
  }



  public void spinupFlywheel() {
    isFlywheelRunning = true;

    //uncomment for smart pid control and for auto aim to work properly and accurately
    // pidController.setReference(setpoint, CANSparkMax.ControlType.kSmartMotion);
    flywheelMotorTop.set(-0.75);
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

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Note Detect", isNote);

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
  }

}
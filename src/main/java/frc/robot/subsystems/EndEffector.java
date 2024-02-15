package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;
import com.revrobotics.SparkPIDController;

public class EndEffector extends SubsystemBase {
    
    private CANSparkMax intakeMotor = new CANSparkMax(EndEffectorConstants.kIntakeMotorID, MotorType.kBrushless);
    private CANSparkMax flywheelMotorTop = new CANSparkMax(EndEffectorConstants.kFlywheelMotorTopID, MotorType.kBrushless);
    private CANSparkMax flywheelMotorBottom = new CANSparkMax(EndEffectorConstants.kFlywheelMotorBottomID, MotorType.kBrushless);
    private SparkPIDController pidController;
    private RelativeEncoder encoderTop;
    private RelativeEncoder encoderBottom;

    private double flywheelSpeed = EndEffectorConstants.kFlywheelMotorSpeed;

    private boolean isFlywheelRunning = false;
    private boolean isIntaking = false;
    private boolean isOuttaking = false;

    public EndEffector() {
        pidController = flywheelMotorTop.getPIDController();
        flywheelMotorBottom.follow(flywheelMotorTop, true);

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

    public void intake() {
        isIntaking = true;
        isOuttaking = false;
        intakeMotor.set(-EndEffectorConstants.kIntakeSpeed);
    }

    public void feed() {
        intakeMotor.set(-1);
    }

    public void toggleIntake() {
        if (isIntaking) stopIntake();
        else intake();
    }

    public void outtake() {
        isOuttaking = true;
        isIntaking = false;
        intakeMotor.set(EndEffectorConstants.kIntakeSpeed);
    }

    public void toggleOuttake() {
        if (isOuttaking) stopIntake();
        else outtake();
    }

    public void stopIntake() {
        intakeMotor.set(0);
        isIntaking = false;
        isOuttaking = false;
    }

    public void toggleFlywheel() {
        if (isFlywheelRunning) stopFlywheel();
        else spinupFlywheel();
    }

    public void spinupFlywheel() {
        isFlywheelRunning = true;
        // pidController.setReference(flywheelSpeed, ControlType.kVelocity);
        // pidControllerBottom.setReference(-2000, ControlType.kVelocity);
        flywheelMotorTop.set(1);
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

    @Override
    public void periodic() {
        // double p = SmartDashboard.getNumber("Flywheel P Gain", EndEffectorConstants.kFlywheelP);
        // double i = SmartDashboard.getNumber("Flywheel I Gain", EndEffectorConstants.kFlywheelI);
        // double d = SmartDashboard.getNumber("Flywheel D Gain", EndEffectorConstants.kFlywheelD);
        // double ff = SmartDashboard.getNumber("Flywheel Feed Forward", EndEffectorConstants.kFlywheelFF);
        // double iz = SmartDashboard.getNumber("Flywheel I Zone", EndEffectorConstants.kFlywheelIz);
        // double min = SmartDashboard.getNumber("Flywheel Min Output", EndEffectorConstants.kFlywheelMinOutput);
        // double max = SmartDashboard.getNumber("FLywheel Max Output", EndEffectorConstants.kFlywheelMaxOutput);

        // if (p != pidController.getP()) pidController.setP(p);
        // if (i != pidController.getI()) pidController.setI(i);
        // if (d != pidController.getD()) pidController.setD(d);
        // if (ff != pidController.getFF()) pidController.setFF(ff);
        // if (iz != pidController.getIZone()) pidController.setIZone(iz);
        // if (min != pidController.getOutputMin() || max != pidController.getOutputMax()) pidController.setOutputRange(min, max);

        // flywheelSpeed = SmartDashboard.getNumber("Flywheel Speed", flywheelSpeed);

        SmartDashboard.putNumber("Flywheel Top RPM", encoderTop.getVelocity());
        SmartDashboard.putNumber("Flywheel Bottom RPM", encoderBottom.getVelocity());
        SmartDashboard.putNumber("Intake Current", intakeMotor.getOutputCurrent());
    }

}
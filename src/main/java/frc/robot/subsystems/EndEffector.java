package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffector extends SubsystemBase {
    
    private CANSparkMax intakeMotor = new CANSparkMax(EndEffectorConstants.kIntakeMotorID, MotorType.kBrushless);
    private CANSparkMax flywheelMotorTop = new CANSparkMax(EndEffectorConstants.kFlywheelMotorTopID, MotorType.kBrushless);
    private CANSparkMax flywheelMotorBottom = new CANSparkMax(EndEffectorConstants.kFlywheelMotorBottomID, MotorType.kBrushless);

    private boolean isFlywheelRunning = false;
    private boolean isIntaking = false;

    public EndEffector() { }

    public void intake() {
        isIntaking = true;
        intakeMotor.set(EndEffectorConstants.kIntakeSpeed);
    }

    public void stopIntake() {
        intakeMotor.set(0);
        isIntaking = false;
    }

    public void spinupFlywheel() {
        isFlywheelRunning = true;
        flywheelMotorBottom.set(EndEffectorConstants.kFlywheelMotorBottomSpeed);
        flywheelMotorTop.set(EndEffectorConstants.kFlywheelMotorTopSpeed);
    }

    public void stopFlywheel() {
        isFlywheelRunning = false;
        flywheelMotorBottom.set(0);
        flywheelMotorTop.set(0);
    }

    public boolean getIsIntaking() {
        return isIntaking;
    }

    public boolean getIsFlywheelRunning() {
        return isFlywheelRunning;
    }

}
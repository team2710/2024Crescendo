package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climb extends SubsystemBase {
    
    private CANSparkMax climber = new CANSparkMax(ClimberConstants.kClimberMotorID, MotorType.kBrushless);
    private SparkPIDController pidController;
    public Climb() {
        climber.setIdleMode(CANSparkMax.IdleMode.kBrake);
        pidController = climber.getPIDController();
        // climber.setSoftLimit(SoftLimitDirection.kReverse, (float) ClimberConstants.kMinPosition);
        // climber.setSoftLimit(SoftLimitDirection.kForward, (float) ClimberConstants.kMaxPosition);

        climber.setSmartCurrentLimit(40);

        pidController.setP(0);
        pidController.setI(0.0);
        pidController.setD(0);
        pidController.setIZone(0);

        //config this if going wrong direction
        climber.setInverted(false);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber Position", getClimberPosition());

        // if (getClimberPosition() >= ClimberConstants.kMaxPosition || getClimberPosition() <= ClimberConstants.kMinPosition) {
        //     climber.set(0);
        // }
    }

    public void stow() {
        pidController.setReference(ClimberConstants.kMinPosition, ControlType.kPosition);
    }

    public void stopClimb() {
        // pidController.setReference(getClimberPosition(), ControlType.kPosition);

        //this number from SVR
        // double m = 0.0002;
        // climber.set(0 - (getClimberPosition() *  m ));
        climber.set(0);
    }

    public double getClimberPosition() {
        return climber.getEncoder().getPosition();
    }

    public void climbUP() {
        climber.set(ClimberConstants.kUpSpeed);
    }

    public void climbDown() {
        climber.set(ClimberConstants.kDownSpeed);
    }

}

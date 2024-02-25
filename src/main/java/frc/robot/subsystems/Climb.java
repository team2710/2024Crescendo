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
        // climber.setSoftLimit(SoftLimitDirection.kReverse, 0);
        // climber.setSoftLimit(SoftLimitDirection.kForward, 600);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber Position", climber.getEncoder().getPosition());
    }

    public void stopClimb() {
        //really scuff feedfoward to prevent slipping
        climber.set(-0.0001);
    }

    public double climberPosition(){
        return climber.getEncoder().getPosition();
    }

    public void climbUP() {
        climber.set(1);
    }
    public void climbDown() {
        climber.set(-1);
    }

}

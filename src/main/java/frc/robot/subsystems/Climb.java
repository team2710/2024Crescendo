package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.climber;;

public class Climb extends SubsystemBase {
    
    private CANSparkMax climber = new CANSparkMax(frc.robot.Constants.climber.kClimberMotorID, MotorType.kBrushless);
    private SparkPIDController pidController;
    public Climb() {
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber Position", climberPosition());
    
    }

    public void stopClimb() {
        climber.set(0);
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

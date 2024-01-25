package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class Pivot extends SubsystemBase {
    
    private CANSparkMax pivotMotorLeft = new CANSparkMax(PivotConstants.kPivotMotorLeftID, MotorType.kBrushless);
    private CANSparkMax pivotMotorRight = new CANSparkMax(PivotConstants.kPivotMotorRightID, MotorType.kBrushless);
    private SparkPIDController pidController;
    private double position = PivotConstants.kPivotStartPosition;

    public Pivot() {
        pidController = pivotMotorLeft.getPIDController();

        pidController.setP(PivotConstants.kPivotP);
        pidController.setI(PivotConstants.kPivotI);
        pidController.setD(PivotConstants.kPivotD);
        pidController.setIZone(PivotConstants.kPivotIz);
        pidController.setFF(PivotConstants.kPivotFF);
        pidController.setOutputRange(PivotConstants.kPivotMinOutput, PivotConstants.kPivotMaxOutput);

        pivotMotorRight.follow(pivotMotorRight);
    }

    @Override
    public void periodic() {
        // get pid values

        pidController.setReference(position, ControlType.kPosition);
    }

    public void setPositionRaw(double position) {
        this.position = position;
    }

}

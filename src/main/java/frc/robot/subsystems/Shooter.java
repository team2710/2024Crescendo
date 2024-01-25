package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    CANSparkMax feeder;
    CANSparkMax shooter;

    public Shooter() {

        feeder = new CANSparkMax(42, MotorType.kBrushless);
        feeder.setSmartCurrentLimit(35);
        shooter = new CANSparkMax(43, MotorType.kBrushless);
        shooter.setSmartCurrentLimit(35);
    }

    public void setFeeder(double speed) {
        feeder.set(speed);
    }

    public void setShooter(double speed) {
        shooter.set(speed);
    }
}

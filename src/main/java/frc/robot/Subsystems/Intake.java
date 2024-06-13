package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private TalonFX motor;
    
    public Intake() {
        motor = new TalonFX(1, "rhino");
    }

    public void set(double speed) {
        motor.set(speed);
    }

    public void stop() {
        motor.stopMotor();
    }
}

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.Intake;

public class Intake_Commands {
    private Intake intake;

    public Intake_Commands(Intake intake) {
        this.intake = intake;
    }

    public InstantCommand runForward() {
        return new InstantCommand(() -> intake.set(0.7), intake);
    }

    public InstantCommand runBackward() {
        return new InstantCommand(() -> intake.set(-0.7), intake);
    }

    public InstantCommand stop() {
        return new InstantCommand(() -> intake.stop(), intake);
    }
}

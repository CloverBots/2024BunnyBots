package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BlowerSubsystem;

public class AutoBlowCommand extends Command {
    private final BlowerSubsystem blowerSubsystem;
    private double time;
    private Timer timer;

    public AutoBlowCommand(BlowerSubsystem blowerSubsystem, double time) {
        this.blowerSubsystem = blowerSubsystem;
        this.time = time;
        timer = new Timer();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        blowerSubsystem.runBlower();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        blowerSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (timer.get() > time) {
            return true;
        } else {
            return false;
        }
    }
}
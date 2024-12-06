package frc.robot.commands;

/* 
 * BUTTON MAPPING 
 * To score position is Driver Left Bumper
 * Score is Driver Right Bumper
 * Intake is Driver Right Trigger
 * Manual Pivot is Operator Back Button followed by Left Joystick
 * Blow is Operator Controller B Button
 */

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.subsystems.BlowerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.TimeOfFlightSubsystem;

public class SuperstructureCommand extends Command {
    private final TimeOfFlightSubsystem timeOfFlightSubsystem;
    private final PivotSubsystem pivotSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final BlowerSubsystem blowerSubsystem;
    private final Supplier<Boolean> scoreButton, prepareToScoreButton, backButton, blowButton;
    private final Supplier<Double> intakeLoadTrigger;
    private final DoubleSupplier leftJoystickY;
    private double pivotSpeed = 0;
    private Timer timer;

    private boolean firing = false;
    private boolean ballonLoaded = false;

    public enum ACTION {
        PARK, INTAKE, PREPARE_TO_SCORE, SCORE, PIVOT_MANUAL, BLOWER
    }

    private ACTION mode;
    private boolean modeChanged = false;

    public SuperstructureCommand(TimeOfFlightSubsystem timeOfFlightSubsystem,
            PivotSubsystem pivotSubsystem,
            IntakeSubsystem intakeSubsystem,
            BlowerSubsystem blowerSubsystem,
            Supplier<Double> intakeLoadTrigger,
            Supplier<Boolean> prepareToScoreButton,
            Supplier<Boolean> scoreButton,
            Supplier<Boolean> backButton,
            DoubleSupplier leftJoystickY,
            Supplier<Boolean> blowButton) {

        this.timeOfFlightSubsystem = timeOfFlightSubsystem;
        this.pivotSubsystem = pivotSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.blowerSubsystem = blowerSubsystem;
        this.intakeLoadTrigger = intakeLoadTrigger;
        this.prepareToScoreButton = prepareToScoreButton;
        this.scoreButton = scoreButton;
        this.backButton = backButton;
        this.leftJoystickY = leftJoystickY;
        this.blowButton = blowButton;

        timer = new Timer();

        mode = ACTION.PARK;

        addRequirements(timeOfFlightSubsystem);
        addRequirements(pivotSubsystem);
        addRequirements(intakeSubsystem);
        addRequirements(blowerSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        pivotSubsystem.setPivotPosition(SuperstructureConstants.PARK_SET_POINT);
        pivotSubsystem.enable();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        SmartDashboard.putBoolean("Ballon Loaded", ballonLoaded);

        checkControllerValues();

        // If mode changed then get appropriate values for the new mode
        if (modeChanged) {
            updateValues();
        }
        switch (mode) {
            case INTAKE:
                if (!ballonLoaded) {
                    intakeSubsystem.setIntakeSpeed(SuperstructureConstants.INTAKE_SPEED);
                }
                break;

            case PREPARE_TO_SCORE:
            case PARK:
                break;

            case SCORE:
                if (!firing) {
                    firing = true;
                    ballonLoaded = false;
                    timer.reset();
                    timer.start();
                    intakeSubsystem.setIntakeSpeed(SuperstructureConstants.OUTTAKE_SPEED);
                }
                break;

            case PIVOT_MANUAL:
                if (Math.abs(leftJoystickY.getAsDouble()) > .05) {
                    pivotSpeed = -leftJoystickY.getAsDouble() / 2.2;
                    if (pivotSpeed > 0.05 &&
                            pivotSubsystem.getPivotPosition() > SuperstructureConstants.UPPER_ENDPOINT) {
                        pivotSpeed = 0;
                    }

                    if (pivotSpeed < -.05 &&
                            pivotSubsystem.getPivotPosition() < SuperstructureConstants.LOWER_ENDPOINT) {
                        pivotSpeed = 0;
                    }

                    if (Math.abs(pivotSpeed) < 0.2) {
                        pivotSpeed = pivotSpeed / 2;
                    }
                    pivotSubsystem.setSpeed(pivotSpeed);
                } else {
                    pivotSubsystem.setSpeed(0);
                }

                break;

            case BLOWER:
                break;
        }
    }

    private void checkControllerValues() {
        if (intakeLoadTrigger.get() > 0.5 && mode != ACTION.INTAKE) {
            mode = ACTION.INTAKE;
            modeChanged = true;
        } else if (prepareToScoreButton.get() && mode != ACTION.PREPARE_TO_SCORE) {
            mode = ACTION.PREPARE_TO_SCORE;
            modeChanged = true;
        } else if (scoreButton.get()) {
            mode = ACTION.SCORE;
            modeChanged = true;
        } else if (backButton.get()) {
            mode = ACTION.PIVOT_MANUAL;
            modeChanged = true;
        } else if (blowButton.get()) {
            mode = ACTION.BLOWER;
            modeChanged = true;
        } else if (mode == ACTION.BLOWER && !blowButton.get()) {
            mode = ACTION.PARK;
            modeChanged = true;
        }
    }

    private void updateValues() {
        modeChanged = false;

        switch (mode) {
            case INTAKE:
                pivotSubsystem.setPivotPosition(SuperstructureConstants.INTAKE_SET_POINT);
                break;

            case SCORE:
                break;

            case PREPARE_TO_SCORE:
                pivotSubsystem.setPivotPosition(SuperstructureConstants.SCORE_SET_POINT);
                break;

            case PARK:
                pivotSubsystem.setPivotPosition(SuperstructureConstants.PARK_SET_POINT);
                blowerSubsystem.stop();
                break;

            case PIVOT_MANUAL:
                pivotSubsystem.disable();
                break;

            case BLOWER:
                blowerSubsystem.runBlower();
                break;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setIntakeSpeed(0);
        pivotSubsystem.setSpeed(0);
        pivotSubsystem.disable();
        blowerSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        switch (mode) {
            case INTAKE:
                if (timeOfFlightSubsystem.isBallonLoaded()) {
                    intakeSubsystem.setIntakeSpeed(0);
                    mode = ACTION.PARK;
                    modeChanged = true;
                    ballonLoaded = true;
                } else if (intakeLoadTrigger.get() < 0.5 && intakeSubsystem.getSpeed() > 0.1) {
                    intakeSubsystem.setIntakeSpeed(0);
                    mode = ACTION.PARK;
                    modeChanged = true;
                }
                break;

            case PREPARE_TO_SCORE:
                break;
            case SCORE:
                if (timer.get() > SuperstructureConstants.OUTTAKE_TIME) { // TO-DO tune time
                    firing = false;
                    mode = ACTION.PARK;
                    modeChanged = true;
                }
                break;

            case PARK:
            case PIVOT_MANUAL:
            case BLOWER:
                break;
        }
        return false;
    }
}
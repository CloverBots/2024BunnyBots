// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveCommand extends Command {

  private final SwerveSubsystem drivebase;
  private final Supplier<double[]> speedXY;
  private final DoubleSupplier rot;
  private final double leftTrigger;
  public static boolean lockOnMode = false;

  public DriveCommand(SwerveSubsystem drivebase, Supplier<double[]> speedXY, DoubleSupplier rot,
      double leftTrigger) {
    this.drivebase = drivebase;
    this.speedXY = speedXY;
    this.rot = rot;
    this.leftTrigger = leftTrigger;

    addRequirements(this.drivebase);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    var xy = speedXY.get();
    var r = rot.getAsDouble();

    if (rot.getAsDouble() > DriveConstants.deadband) {
      r = DriveConstants.teleOpNormalAngularSpeed;
    }

    if (leftTrigger > 0.5) {
      xy[0] = xy[0] / 2;
      xy[1] = xy[1] / 2;
      r = DriveConstants.teleOpSlowAngularSpeed;
    }

    drivebase.defaultDrive(-xy[1], -xy[0], r);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
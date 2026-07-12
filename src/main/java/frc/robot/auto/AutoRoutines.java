package frc.robot.auto;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoRoutines {
  private final Drive drive;

  public AutoRoutines(Drive drive) {
    this.drive = drive;
  }

  public SendableChooser<Command> buildAutoChooser() {
    SendableChooser<Command> chooser = new SendableChooser<>();
    chooser.setDefaultOption("Do Nothing", Commands.none());
    // TODO: Add your auto routines here, e.g.:
    // chooser.addOption("My Auto", AutoBuilder.buildAuto("MyAuto"));
    return chooser;
  }

  /**
   * Builds the dashboard auto chooser: configured routines plus drive characterization/SysId
   * options. Falls back to an empty chooser (and raises an Alert) if routine construction throws.
   */
  public static LoggedDashboardChooser<Command> buildDashboardChooser(Drive drive) {
    try {
      AutoRoutines autoRoutines = new AutoRoutines(drive);
      LoggedDashboardChooser<Command> chooser =
          new LoggedDashboardChooser<>("Auto Choices", autoRoutines.buildAutoChooser());
      chooser.addOption(
          "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
      chooser.addOption(
          "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
      chooser.addOption(
          "Drive SysId (Quasistatic Forward)",
          drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      chooser.addOption(
          "Drive SysId (Quasistatic Reverse)",
          drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      chooser.addOption(
          "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
      chooser.addOption(
          "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
      return chooser;
    } catch (Exception e) {
      e.printStackTrace();
      Alert alert = new Alert("auto failed to load", AlertType.kError);
      alert.set(true);
      return new LoggedDashboardChooser<Command>("Auto Choices");
    }
  }
}

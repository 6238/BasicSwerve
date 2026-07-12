package frc.lib.penguincore;

import static frc.lib.penguincore.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * NT-tunable PIDF gains for a TalonFX. Every instance registers itself with the {@link
 * CommandScheduler} as a pseudo-subsystem on construction, so {@link #periodic()} runs
 * automatically off the main robot loop — no caller wiring required. Honors {@link
 * Settings#SETTINGS_LOCK} — when locked, dashboard edits are never pushed to hardware.
 */
public class LoggedNetworkPIDFeedforwardGains implements Subsystem {
  private final TalonFX talon;

  private final LoggedNetworkNumber kP;
  private final LoggedNetworkNumber kI;
  private final LoggedNetworkNumber kD;
  private final LoggedNetworkNumber kA;
  private final LoggedNetworkNumber kV;
  private final LoggedNetworkNumber kS;
  private final LoggedNetworkNumber kG;

  private double lastKP;
  private double lastKI;
  private double lastKD;
  private double lastKA;
  private double lastKV;
  private double lastKS;
  private double lastKG;

  public LoggedNetworkPIDFeedforwardGains(
      TalonFX talon,
      double defaultKP,
      double defaultKI,
      double defaultKD,
      double defaultKA,
      double defaultKV,
      double defaultKS,
      double defaultKG,
      String namePrefix) {
    this.talon = talon;

    kP = new LoggedNetworkNumber("Gains/" + namePrefix + "/kP", defaultKP);
    kI = new LoggedNetworkNumber("Gains/" + namePrefix + "/kI", defaultKI);
    kD = new LoggedNetworkNumber("Gains/" + namePrefix + "/kD", defaultKD);
    kA = new LoggedNetworkNumber("Gains/" + namePrefix + "/kA", defaultKA);
    kV = new LoggedNetworkNumber("Gains/" + namePrefix + "/kV", defaultKV);
    kS = new LoggedNetworkNumber("Gains/" + namePrefix + "/kS", defaultKS);
    kG = new LoggedNetworkNumber("Gains/" + namePrefix + "/kG", defaultKG);

    lastKP = defaultKP;
    lastKI = defaultKI;
    lastKD = defaultKD;
    lastKA = defaultKA;
    lastKV = defaultKV;
    lastKS = defaultKS;
    lastKG = defaultKG;

    CommandScheduler.getInstance().registerSubsystem(this);
  }

  /** Re-applies gains to the motor controller if any tunable value changed since last call. */
  @Override
  public void periodic() {
    if (Settings.SETTINGS_LOCK) return;

    if (kP.get() == lastKP
        && kI.get() == lastKI
        && kD.get() == lastKD
        && kA.get() == lastKA
        && kV.get() == lastKV
        && kS.get() == lastKS
        && kG.get() == lastKG) {
      return;
    }

    lastKP = kP.get();
    lastKI = kI.get();
    lastKD = kD.get();
    lastKA = kA.get();
    lastKV = kV.get();
    lastKS = kS.get();
    lastKG = kG.get();

    Slot0Configs configs = toSlot0Configs();
    tryUntilOk(5, () -> talon.getConfigurator().apply(configs, 0.25));
  }

  public Slot0Configs toSlot0Configs() {
    Slot0Configs configs = new Slot0Configs();
    configs.kP = kP.get();
    configs.kI = kI.get();
    configs.kD = kD.get();
    configs.kA = kA.get();
    configs.kV = kV.get();
    configs.kS = kS.get();
    configs.kG = kG.get();
    return configs;
  }
}

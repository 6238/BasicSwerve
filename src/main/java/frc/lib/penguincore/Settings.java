package frc.lib.penguincore;

/** Library-wide settings switches for frc.lib.penguincore. */
public final class Settings {
  /**
   * When true, dashboard-tunable values (e.g. {@link LoggedNetworkPIDFeedforwardGains}) stop
   * pushing changes to hardware. Flip to true before competition so a fat-fingered dashboard number
   * can't reconfigure a motor controller mid-match.
   */
  public static final boolean SETTINGS_LOCK = false;

  private Settings() {}
}

package io.excaliburfrc.lib.sim;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.simulation.CTREPCMSim;

public class DoubleSolenoidSim {
  private static final CTREPCMSim pcm = new CTREPCMSim();
  private final int m_fwd;
  private final int m_rev;

  public DoubleSolenoidSim(int fwd, int rev) {
    this.m_fwd = fwd;
    this.m_rev = rev;
  }

  public DoubleSolenoid.Value get() {
    int rev = pcm.getSolenoidOutput(m_rev) ? 1 : 0;
    int fwd = (pcm.getSolenoidOutput(m_fwd) ? 1 : 0) << 1;

    switch (rev | fwd) {
      case 0b0:
        return DoubleSolenoid.Value.kOff;
      case 0b1:
        return DoubleSolenoid.Value.kReverse;
      case 0b10:
        return DoubleSolenoid.Value.kForward;
    }
    throw new AssertionError("In-Valid state");
  }
}

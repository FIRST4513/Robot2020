package robot.utils;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.buttons.Button;
import java.util.Objects;
//import org.jetbrains.annotations.NotNull;

/**
 * A button based on a joystick axis. This can be used to emulate a button using a trigger or
 * joystick.
 */
public class POVButton extends Button {

  private GenericHID stick;
  private int pov;

  /**
   * Constructs an {@code AxisButton}.
   *
   * @param stick The axis's joystick
   * @param axis The axis to use as a button
   * @param threshold The threshold for the button to be triggered
   * @throws NullPointerException If {@code stick} is {@code null}.
   */
  public POVButton( GenericHID stick, int pov) {
    //this.stick = Objects.requireNonNull(stick);
    this.stick = stick;
    this.pov = pov;
  }

  @Override
  public boolean get() {
      if (stick.getPOV() == pov) {
          return true;
      } else {
          return false;
      }
  }
}

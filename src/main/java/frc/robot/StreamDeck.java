package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;

public class StreamDeck {
  // private GenericHID streamDeck;
  private int numButtons;
  private int selected = 2;

  public StreamDeck(int port, int numButtons) {
    // this.streamDeck = new GenericHID(port);
    this.numButtons = numButtons;
  }

  // checks if each button has been pressed since the last call, if it has selected becomes that
  // value -1.
  // due to the nature of the for loop, if 2 buttons are pressed in between calls the button with
  // the highest index will be selected.
  public int getSelected() {
    return 0;
    // for (int i = 1; i <= numButtons; i++) {
    //   if (streamDeck.getRawButtonPressed(i)) {
    //     selected = i;
    //   }
    // }
    // return selected;
  }
}

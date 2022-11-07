package org.firstinspires.ftc.teamcode.util;


public class ButtonPress {

	boolean lastButton = false;
	boolean buttonState = false;

	public ButtonPress() {
	}

	/**
	 * get the current button state
	 *
	 * @param buttonState the current button state
	 */
	public void button(boolean buttonState) {

		lastButton = this.buttonState;
		this.buttonState = buttonState;

	}

	/**
	 * check if a button had a singular press
	 *
	 * @return if it is a singular button press
	 */
	public boolean press() {

		return buttonState && !lastButton;

	}

	/**
	 * check if a button was just released
	 *
	 * @return if the button was just released
	 */
	public boolean release() {

		return !buttonState && lastButton;

	}


}

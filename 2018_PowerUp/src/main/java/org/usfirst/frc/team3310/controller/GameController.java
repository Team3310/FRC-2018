package org.usfirst.frc.team3310.controller;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.Button;


/**
 * Contains functions for use with MOST controller (XBox, Playstation,Logitech).
 * @author Joshua Lewis joshlewi14@gmail.com
 */
public class GameController extends Joystick {

	private static final double DEADZONE = 0.1;
	private static final double TRIGGER_TOLERANCE = 0.5;

	private ButtonMap map;	
	/**
	 * Constructor that creates a Joystick object.
	 */
	public GameController(int gamepadPort, ButtonMap map) {
		super(gamepadPort);
		this.map = map;
	}

	/**
	 * Returns the X position of the left stick.
	 */
	public double getLeftXAxis() {
		return getRawAxis(map.AXIS_LEFT_X);
	}

	/**
	 * Returns the X position of the right stick.
	 */
	public double getRightXAxis() {
		return getRawAxis(map.AXIS_RIGHT_X);
	}

	/**
	 * Returns the Y position of the left stick.
	 */
	public double getLeftYAxis() {
		return getRawAxis(map.AXIS_LEFT_Y);
	}

	/**
	 * Returns the Y position of the right stick.
	 */
	public double getRightYAxis() {
		return getRawAxis(map.AXIS_RIGHT_Y);
	}

	/**
	 * Checks whether Button X/A is being pressed and returns true if it is.
	 */
	public boolean getButtonStateA() {
		return getRawButton(map.A_BUTTON);
	}

	/**
	 * Checks whether Button Circle/B is being pressed and returns true if it is.
	 */
	public boolean getButtonStateB() {
		return getRawButton(map.B_BUTTON);
	}

	/**
	 * Checks whether Button Sqaure/X is being pressed and returns true if it is.
	 */
	public boolean getButtonStateX() {
		return getRawButton(map.X_BUTTON);
	}

	/**
	 * Checks whether Button Triangle/Y is being pressed and returns true if it is.
	 */
	public boolean getButtonStateY() {
		return getRawButton(map.Y_BUTTON);
	}

	
	public boolean getButtonStatePad() {
		return getRawButton(map.BUTTON_PAD);
	}

	public int getDpadAngle() {
		return this.getPOV();
	}

	/**
	 * Returns an object of Button A.
	 */
	public Button getButtonA() {
		return new JoystickButton(this, map.A_BUTTON);
	}

	/**
	 * Returns an object of Button B.
	 */
	public Button getButtonB() {
		return new JoystickButton(this, map.B_BUTTON);
	}

	/**
	 * Returns an object of Button X.
	 */
	public Button getButtonX() {
		return new JoystickButton(this, map.X_BUTTON);
	}

	/**
	 * Returns an object of Button Y.
	 */
	public Button getButtonY() {
		return new JoystickButton(this, map.Y_BUTTON);
	}



	public JoystickButton getButtonPad() {
		return new JoystickButton(this, map.BUTTON_PAD);
	}
	

	/**
	 * Gets the state of the Start button
	 * @return the state of the Start button
	 */
	public JoystickButton getOptionsButton(){
		return new JoystickButton(this, map.BUTTON_OPTIONS);
	}

	public JoystickButton getShareButton() {
		return new JoystickButton(this, map.BUTTON_SHARE);
	}
	
	public JoystickButton getStartButton() {
		return new JoystickButton(this, map.BUTTON_START);
	}
 
	/**
	 * Gets the state of the left bumper
	 * @return the state of the left bumper
	 */
	public JoystickButton getLeftBumper() {
		return new JoystickButton(this, map.BUTTON_L1);
	}


	/**
	 * Gets the state of the right bumper
	 * @return the state of the right bumper
	 */
	public Button getRightBumper() {
		return new JoystickButton(this, map.BUTTON_R1);
	}

	public Button getRightTrigger() {
		return new AxisTriggerButton(this, map.AXIS_RIGHT_TRIGGER, TRIGGER_TOLERANCE);
	}
	public Button getLeftTrigger() {
		return new AxisTriggerButton(this, map.AXIS_LEFT_TRIGGER, TRIGGER_TOLERANCE);
	}

	public Button getDPadUp() {
		return new DPadTriggerButton(this, map.DPAD_UP);
	}
	public Button getDPadDown() {
		return new DPadTriggerButton(this, map.DPAD_DOWN);
	}
	public Button getDPadLeft() {
		return new DPadTriggerButton(this, map.DPAD_LEFT);
	}
	public Button getDPadRight() {
		return new DPadTriggerButton(this, map.DPAD_RIGHT);
	}
	/**
	 * Gets the state of the left stick button
	 * @return the state of the left stick button
	 */
	public JoystickButton getL3() {
		return new JoystickButton(this, map.BUTTON_L3);
	}

	/**
	 * Gets the state of the right stick button
	 * @return the state of the right stick button
	 */
	public JoystickButton getR3() {
		return new JoystickButton(this, map.BUTTON_R3);
	}

	/**
	 * Gets the state of the left trigger
	 * @return the state of the left trigger
	 */
	public JoystickButton getL2() {
		return new JoystickButton(this, map.BUTTON_L2);
	}

	/**
	 * Gets the state of the right trigger
	 * @return the state of the right trigger
	 */
	public JoystickButton getR2() {
		return new JoystickButton(this, map.BUTTON_R2);
	}	

	private boolean inDeadZone(double input){
		boolean inDeadZone; 
		if(Math.abs(input) < DEADZONE){
			inDeadZone = true;
		}else{
			inDeadZone = false;
		}
		return inDeadZone;
	}

	private double getAxisWithDeadZoneCheck(double input){
		if(inDeadZone(input)){
			input = 0.0;       
		}
		return input; 
	}

	public double getTriggerAxis(int axis){         
		return getAxisWithDeadZoneCheck(this.getRawAxis(axis)); 
	}

	private class AxisTriggerButton extends Button 
	{
		private GameController m_controller;
		private int m_axis;
		private double m_tolerance;

		public AxisTriggerButton(GameController controller, int axis, double tolerance) {
			m_controller = controller;
			m_axis = axis;
			m_tolerance = tolerance;
		}

		public boolean get() {
			return (m_controller.getTriggerAxis(m_axis) > m_tolerance);
		}
	}

	private class DPadTriggerButton extends Button 
	{

		private int buttonAngle;
		private GameController  controller;

		public DPadTriggerButton(GameController controller, int dPadButtonAngle) {
			this.buttonAngle = dPadButtonAngle;
			this.controller = controller;
		}
		
		@Override
		public boolean get() {
			return controller.getDpadAngle() == buttonAngle;
		}
	}
}
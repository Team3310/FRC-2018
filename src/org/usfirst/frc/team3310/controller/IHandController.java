package org.usfirst.frc.team3310.controller;

public interface IHandController {

	public double getLeftXAxis();

	public double getLeftYAxis();

	public double getRightXAxis();
	
	public double getRightYAxis();
	
	public boolean isQuickTurn();
	
	public boolean getLeftBumperButton();

	public boolean getRightBumperButton();

	public boolean getAButton();

	public boolean getBButton();

	public boolean getXButton();
	
	public boolean getYButton();
}

/**
 * File: 	DegeneratedLine2DException.java
 * Project: javaGeom
 * 
 * Distributed under the LGPL License.
 *
 */
package org.usfirst.frc.team3310.utility.geom;


/**
 * A degenerated line, whose direction vector is undefined, had been
 * encountered.
 * This kind of exception can occur during polygon or polylines algorithms,
 * when polygons have multiple vertices. 
 * @author dlegland
 * @since 0.9.0
 */
public class DegeneratedLine2DException extends RuntimeException {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

	protected AbstractLine2D line;
	
	/**
	 * @param msg the error message
	 * @param line the degenerated line
	 */
	public DegeneratedLine2DException(String msg, AbstractLine2D line) {
		super(msg);
		this.line = line;
	}
	
	/**
	 * @param line the degenerated line
	 */
	public DegeneratedLine2DException(AbstractLine2D line) {
		super();
		this.line = line;
	}

	public AbstractLine2D getLine() {
		return line;
	}
}

/**
 * 
 */

package org.usfirst.frc.team3310.utility.geom;

/**
 * Exception thrown when an unbounded shape is involved in an operation
 * that assumes a bounded shape. 
 * @author dlegland
 */
public class UnboundedShape2DException extends RuntimeException {

	private AbstractLine2D shape;
	
    /**
     * 
     */
    private static final long serialVersionUID = 1L;

    public UnboundedShape2DException(AbstractLine2D shape) {
    	this.shape = shape;
    }

    public AbstractLine2D getShape() {
    	return shape;
    }
}

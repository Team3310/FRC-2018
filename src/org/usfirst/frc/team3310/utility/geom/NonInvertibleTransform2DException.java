/**
 * 
 */

package org.usfirst.frc.team3310.utility.geom;

/**
 * Exception thrown when trying to compute an inverse transform of a transform
 * that does not allows this feature.
 * @author dlegland
 */
public class NonInvertibleTransform2DException extends RuntimeException {

    /**
     * 
     */
    private static final long serialVersionUID = 1L;

    protected AffineTransform2D transform;
    
    public NonInvertibleTransform2DException() {
    	this.transform = null;
    }
    
    public NonInvertibleTransform2DException(AffineTransform2D transform) {
    	this.transform = transform;
    }
    
    public AffineTransform2D getTransform() {
    	return transform;
    }
}

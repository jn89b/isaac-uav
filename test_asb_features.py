
import aerosandbox as asb
import aerosandbox.numpy as np

def test_rotation():
    print("Testing Rotation Matrix...")
    
    # Case 1: Alpha=0, Beta=0
    # V aligned with X_wind.
    op = asb.OperatingPoint(velocity=100, alpha=0, beta=0)
    R = op.compute_rotation_matrix_wind_to_geometry()
    
    print("\nAlpha=0, Beta=0")
    print("R_w_g:\n", R)
    
    # If Geometry is (Back, Right, Up) and Wind is (Forward, Right, Up)
    # Then R should map Forward to Back (-1), Right to Right (1), Up to Up (1)?
    # Or maybe V is defined along -X_geo?
    
    # Case 2: Alpha=90 deg. (V coming from below? or pitch up?)
    # "Alpha is the angle between the projection of V on the x-z plane and the x-axis".
    op2 = asb.OperatingPoint(velocity=100, alpha=90, beta=0)
    R2 = op2.compute_rotation_matrix_wind_to_geometry()
    print("\nAlpha=90")
    print("R_w_g:\n", R2)

if __name__ == "__main__":
    test_rotation()

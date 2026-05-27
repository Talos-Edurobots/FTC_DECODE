The turret LUT contains position - target aiming position pairs that are created and tested inside our [[Simulator]]. We tested the robot on different field coordinates and we match each one of those with their corresponding aim position as shown on [[TurretLUT_img.png]]. 

### What it solved and why we needed it
Before applying that logic, the turret was always facing on a fixed position, but that lead to inaccurate shots on specific angles. Now we solved that problem with that interpolated LUT

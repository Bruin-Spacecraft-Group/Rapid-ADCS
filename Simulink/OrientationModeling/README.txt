Requires download of all files within OrientationSim to run, MatLab 2021b
-----------
Uses AddOns: 
- Simulink 3D Animation
- Robotics System Toolbox
- Simulink
- Aerospace Blockset
-----------
Final model will:
- Outputs dynamic orientation in space with animation by:
    - requested motor angular velocity values
    - magnetorquer currents
    - external torques
- Provides input for external torques (gravity gradient, solar pressure, atmospheric drag, etc.)
-----------
Variable setup:
- Inertias:
    - Satellite:
        - Use TensorTransformation to import inertia tensor data from SolidWorks into [Orientation > Inertia Tensors > Satellite Inertia Tensor]
    - Reaction wheels:
        - Assuming wheels are pointed at CG and symmetric around axis of rotation, options:
            1. Use built-in disk tensor creator for mass and radius
            2. Calculate inertia for unique shaped-wheel and input according to comment
            3. Use TensorTransformation to import inertia tensor from CAD (only viable option for wheels not symmetric around axis of rotation)
- Motors:
    - Torques input in [Orientation > Motor Output]
    - Deadzones input in [Orientation > Motor Output]
    - Animation wheel speed reduction factor (does not affect physics) in [Orientation]
- Magnetorquers:
    - Directions specified by unit vector input [Orientation > Magnetorquer Model > mag1_r / mag2_r]
- Models:
    - Gravitational:
        Gravity Model
    - Magnetic Field:
        [Orientation > Magnetorquer Model > Magnetic Field Vector]
    - External Torques:
        NOT CURRENTLY AVAILABLE (torque input in [Orientation > Angular Velocity Calculator])
-----------
Control inputs:
- Desired wheel angular velocity
- Magnetorquer moment
-----------
Outputs:
- Yaw, pitch, roll
- XYZ unit vectors
- Magnetic field vector direction
- Assignable animation vector direction
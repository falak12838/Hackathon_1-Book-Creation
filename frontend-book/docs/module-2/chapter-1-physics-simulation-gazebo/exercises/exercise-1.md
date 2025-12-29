# Exercise 1: Basic Physics Simulation

## Objective

Create a humanoid robot model and observe its behavior under different physical conditions in Gazebo.

## Prerequisites

- Gazebo Harmonic installed and configured
- Basic understanding of URDF
- Completed the previous tutorials in Chapter 1

## Tasks

### Task 1: Create an Enhanced Humanoid Model

1. Start with the basic humanoid model from the tutorial
2. Add at least 3 additional joints that allow for movement (e.g., neck joint, shoulder joints)
3. Ensure all mass and inertia properties are properly calculated
4. Test that the model loads correctly in Gazebo

### Task 2: Gravity Experimentation

1. Create three different world files with different gravity values:
   - Standard Earth gravity: 9.8 m/s²
   - Moon gravity: 1.62 m/s²
   - Mars gravity: 3.71 m/s²
2. Launch each world with your humanoid model positioned at 3 meters height
3. Record the time it takes for the robot to hit the ground in each scenario
4. Compare your results with the theoretical calculation: `t = √(2h/g)`

### Task 3: Collision Analysis

1. Create a world with multiple obstacles of different shapes (cube, sphere, cylinder)
2. Position your humanoid model to collide with each obstacle
3. Observe and document the differences in collision response
4. Modify friction coefficients and observe the changes

## Deliverables

1. Enhanced URDF file for your humanoid model with additional joints
2. Screenshots of your robot in each gravity environment
3. Table comparing theoretical vs. measured fall times
4. Documentation of collision behavior differences
5. Brief report (1-2 paragraphs) summarizing your findings

## Evaluation Criteria

- [ ] Model loads successfully in Gazebo
- [ ] Additional joints function properly
- [ ] Gravity experiments show expected differences
- [ ] Collision behaviors are properly documented
- [ ] Calculations match theoretical values within reasonable tolerance

## Time Estimate

This exercise should take approximately 2-3 hours to complete.

## Next Steps

After completing this exercise, proceed to Chapter 2 to learn about Digital Twins and Human-Robot Interaction using Unity.
# HyPA-system-simulation-and-analysis
This is a repository containing the modeling, simulation and analysis code (Matlab) for the Hybrid Pose Adjustment (HyPA) System.
The modeling tool is based on Featherstone's version of rigid body dynamics algorithm.
Various actuation methods are supported, including the **cable actuation**, the **thruster actuation**, and the **active joint**.

1. The active joint is modeled as a direct actuation force/torque provider that fully actuates the degrees of freedom (DoFs) of the corresponding joint.
2. The cable actuation is modeled as a unilateral force that goes sequentially through the line segments between its attachment points (which can be located on different links).
3. The thruster is modeled as a unilateral force and dragging torque generator that is fixed on a certain link.


## Usage

* Add all files/subfolders of `HyPA-system-simulation-and-analysis` into Matlab paths.
* Run the preset simulation: open and execute script `HyPA-system-simulation-and-analysis/scripts/apas_var1/apas_var1_motion_planning_translation_and_leveling.m`. 
Executing this script will start a preset module trajectory following simulation, which involves both model translation and steering, while maintaining leveling.
The motion is achieved through open-loop trajectory planning, together with low-level actuator position/force control.
After the simulation, results will be automatically plotted and saved to files.
An animation generation will also be started after the simulation, which will produce several `.avi` video clips if left running uninterrupted.
* Run a group of simulations to derive module control error distribution: open and execute script `HyPA-system-simulation-and-analysis/scripts/apas_var1/apas_var1_motion_planning_set_point_error_map.m`. 
This script will create a default grid of the desired module position and control the module to reach the desired position with zero steering angle change while attempting to maintain leveling.
More specifically, for each desired module position, the script will generate a trajectory from the starting point to the desired position grid point.
A set of corresponding actuator trajectories is then calculated and sent to the corresponding actuator controller to execute. 
After the trajectory following simulation, an module pose error evaluation will be calculated and recorded.
The results are saved to `HyPA-system-simulation-and-analysis/data/analysis_result/APAS/APAS_var1.mat`.
**This process takes a very long time.**

* Script `HyPA-system-simulation-and-analysis/scripts/apas_var1/apas_var1_motion_planning_set_point_error_map_data_processing.m` loads the aforementioned saved result data, visualize the error distribution and save to files.

## Code Structure

The primary functions of each folder are summarized as follows:

* `HyPA-system-simulation-and-analysis/tools_modelling`: creates the dynamics model of rigid-body system with different actuation methods.
    * `joints`: a collection of various joint definitions.
    Functions are defined for various joint-related calculations, such as joint Jacobian and its derivatives, relative position and attitude of adjacent links, etc.
    * `model`: holds `RigidBodySystem` class which takes a model definition struct and produces a dynamics model.
    The model generation uses various functions defined eleswhere, such as the ones in `joints` subfolder.
    * `trajectory`: defines trajectory based on the trajectory definition, which can involve segments of different type, such as polynomial interpolation between setpoints or arbitrary functions of time.
* `HyPA-system-simulation-and-analysis/tools_mathematics`: tool functions mainly used to calculate the dynamics model.
* `HyPA-system-simulation-and-analysis/tools_toolbox`: quality of life tool functions.
* `HyPA-system-simulation-and-analysis/tools_control`: single-input-single-output PID controller.
* `HyPA-system-simulation-and-analysis/definitions`: scripts to generate the model and trajectory definition structs.
* `HyPA-system-simulation-and-analysis/scripts`: scripts to run simulation, analysis or tests.
* `HyPA-system-simulation-and-analysis/data`: holds analysis and simulation results (data and figures).
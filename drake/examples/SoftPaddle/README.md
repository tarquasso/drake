# Soft Paddle Model and Control

## About This Example

The example in this directory implements the model (a plant) for a soft paddle consisting of a rigid frame wrapped by an elastic rubber band. The goal is to juggle a disk bounding up and down an inclined plane by controlling the action of the paddle (much like a pinball).

### The Plant

The soft paddle and disk models are implemented by the class **SoftPaddlePlant** in the source files `soft_paddle_plant.h (.cc)`. 
The generalized positions for the plant are the disk `x` and `z` positions `q = [x, z]` and its state vector is `xc = [q, qdot]`. The input to the plant is the paddle angle `phi`.
The rubber band is modelled assuming a constant tension `T0` all across and forces on the disk are obtained assuming small deformations. The constant tension model is a good approximation when the extra elongations (the band is already elongated by the tension `T0`) cause changes in the tension that  can be neglected in comparison to `T0`. Small angle deformations are assumed so that this model can be directly compared to a model based on a linear string equation that can be solved by the FEM method. This plant is therefore a baseline for these future studies. 

It is straightforward to extend this model to include variations in the tension `T` and large deformations.

### Mirror Law Control

A very simple mirror law control is implemented in the class **PaddleMirrorLawSystem**. This system has parameters `paddle_aim` and `stroke_strenght`. The input to this system is the output of a **SoftPaddlePlant** and it outputs a paddle angle that can be fed to the input of a **SoftPaddlePlant**.
The mirror law is very simple: `phi = paddle_aim + stroke_strength * zdot`, where `zdot` is the vertical velocity of the disk.
This system is implemented in the files `mirror_law_system.h (.cc)`.

### Feedback

A system diagram is implemented in **SoftPaddleWithMirrorControl** connecting the output of a **SoftPaddlePlant** to the input of a **PaddleMirrorLawSystem** and closing the feedback loop by connecting the output angle from the **PaddleMirrorLawSystem** back to the input of the **SoftPaddlePlant**.
 **SoftPaddleWithMirrorControl** is implemented in the files `mirror_law_system.h (.cc)`.

### Discrete Poincare Map System

The poincare map of the **SoftPaddleWithMirrorControl** is built by integrating the equations of motion between two apexes of the disk. This is implemented as a discrete system with updates `x_{n+1} = f(x_{n}, u_{n})` where `x_n = [x_n, z_n]` the disk's position at an apex and `u_n = [paddle_aim, stroke_strength]` the parameters to the **PaddleMirrorLawSystem** controlling the paddle motion between two apexes.
This is implemented by the **SoftPaddlePoincareMap** discrete system in the files `soft_paddle_poincare_map.h (.cc)`.

### Visualization

A **SoftPaddleStateToBotVisualizer** system implements the conversion of a paddle state to a **RigidBodyPlant** state that can be visualized with **DrakeVisualizer**.

## Demos

### Dynamics

Open two terminals. In the first terminal, start Drake Visualizer:

    $ cd drake-distro
    $ ./build/install/bin/drake-visualizer

In the second terminal, start the simulation:

    $ cd drake-distro
    $ ./build/drake/examples/SoftPaddle/soft_paddle_run_dynamics

### Fixed points

The executable `soft_paddle_poincare_map_test` finds the input parameters `u_n` to the **SoftPaddlePoincareMap** system for a given position of the disk `[x_n, z_n]` so that `x_{n} = f(x_{n}, u_{n})`. This is done via a Newton-Raphson iteration. The Jacobian of the set of equations is found passing `AutoDiffScalar`'s through the Poincare map.

To run `soft_paddle_poincare_map_test`:

    $ cd drake-distro
    $ ./build/drake/examples/SoftPaddle/soft_paddle_poincare_map_test

In the current version the target fixed point `[x_n, z_n]` is hardcoded in the source.

### Run Dynamics Around a Fixed Point

Open two terminals. In the first terminal, start Drake Visualizer:

    $ cd drake-distro
    $ ./build/install/bin/drake-visualizer

In the second terminal, start the simulation:

    $ cd drake-distro
    $ ./build/drake/examples/SoftPaddle/soft_paddle_mirror_law_test

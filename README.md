### Description ###
Examples and methods to simulate inelastic rigid-body contact dynamics with Coulomb friction. Companion to the paper:
P. C. Horak and J. C. Trinkle, "Comparison of Complementarity and Convex Contact Models," arXiv:TBA [cs], May. 2018.

### Contents ###
* **experiments** - scripts to run the simulation experiments (depend on timestepping)
    * **all_examples.m** - reproduces plots from the results section of the paper
* **plotting** - animate results from the experiments
* **solvers** - solve generic contact dynamics problems
* **timestepping** - simulate the dynamics for the experiments (depend on solvers)

### Solvers ###

The solvers are based on or discussed in the following papers:

* **solver_blcp.m** -
K. Erleben, “Velocity-based shock propagation for multibody dynamics animation,” *ACM Trans. Graph.*, vol. 26, no. 2, pp. 12-1–20, Jun. 2007.
(sections 2-3, no velocity-based shock propagation)

* **solver_ccp.m** -
A. Tasora and M. Anitescu, “A matrix-free cone complementarity approach for solving large-scale, nonsmooth, rigid body dynamics,” *Comput. Methods in Appl. Mechanics and Eng.*, vol. 200, no. 5, pp. 439–453, Jan. 2011.

* **solver_convex.m** -
E. Todorov, “Convex and analytically-invertible dynamics with contacts and constraints: Theory and implementation in MuJoCo,” in *2014 IEEE Int. Conf. Robotics and Automation*, 2014, pp. 6054–6061.

* **solver_lcp.m** -
D. E. Stewart and J. C. Trinkle, “An implicit time-stepping scheme for rigid body dynamics with inelastic collisions and coulomb friction,” *Int. J. Numer. Meth. Eng.*, vol. 39, no. 15, pp. 2673–2691, Aug. 1996.

* **solver_ncp.m** - 
Y. Lu, “A framework for comparison of methods for solving complementarity problems that arise in multibody dynamics,” Rensselaer Polytechnic Inst., Troy, NY, 2016.
(see [prox_NCP.m](https://github.com/rpiRobotics/rpi-matlab-simulator/blob/master/engine/solvers/prox_based/prox_NCP.m) from the [RPI-MATLAB-Simulator](https://github.com/rpiRobotics/rpi-matlab-simulator))

* **solver_qp.m** -
E. Todorov, “A convex, smooth and invertible contact model for trajectory optimization,” in *2011 IEEE Int. Conf. Robotics and Automation*, 2011, pp. 1071–1076.

### Optional Dependencies ###

* lemke.m from the [CompEcon Toolbox](http://www4.ncsu.edu/~pfackler/compecon/toolbox.html)
* The [PATH Solver](http://pages.cs.wisc.edu/~ferris/path.html)

### Tested Environments ###

* Ubuntu 14.04, MATLAB R2017a

# CFS
This repo contains implementations of convex feasible set algorithm (CFS).

Refer to the following paper for details about the CFS algorithm

C. Liu, C. Lin, and M. Tomizuka, "The convex feasible set algorithm for real time optimization in motion planning," [arXiv:1709.00627](https://arxiv.org/abs/1709.00627).

## CFS_Matlab_2D 
This folder contains the implementation for a 2D problem.

install [multi-parametric toolbox](https://www.mpt3.org) first 

run main_CFS.m to see the result below

![](https://github.com/changliuliu/CFS/blob/master/CFS_Matlab_2D/outcome.jpg)

## CFS_Matlab_Arm 
This folder contains the implementation for a robot arm.

run main_CFS.m to see the result below

![](https://github.com/changliuliu/CFS/blob/master/CFS_Matlab_Arm/outcome.jpg)

It takes a while to finish the 3D plot ...

## CFS_Knitro_2D
This folder contains the implementation in C++ on top of Knitro

install [Knitro](https://www.artelys.com/en/optimization-tools/knitro) first

> cd build
>
> cmake ..
>
> make

The problem is defined in PlanningProblem.h. Some adjustable parameters are loadable from the folder "parameter".

MyCFS solves the problem using CFS.

MyNLNC solves the problem using built-in solvers in Knitro. 

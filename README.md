# Requirements

```
$ pip install pipenv
$ pipenv install
```

# Contents



## convex_optimization_algorithms

### prinam dual interpoint method

```
pipenv run python convex_optimization_algorithms/primal_dual_interpoint/primal_dual_interpoint.py
```

<p align="center">
  <img src="./convex_optimization_algorithms/media/primal_dual_interpoint.png" width="600">
</p>

### prinam dual interpoint method

```
pipenv run python convex_optimization_algorithms/active_set/active_set.py
```

<p align="center">
  <img src="./convex_optimization_algorithms/media/active_set.png" width="600">
</p>




## A* based speed planner

A* based speed planning test in t-s space.


```
$ pipenv run python astar_velocity_planning/astar_speed_planner.py
```




 - Black : Obstacle occupied area
 - Red : Optimal path
 - Gray : Candidates

 ![readme_res](./astar_velocity_planning/media/res1.png)


### Reference

[Lim, Wonteak & Lee, Seongjin & Sunwoo, Myoungho & Jo, Kichun. (2018). Hierarchical Trajectory Planning of an Autonomous Car Based on the Integration of a Sampling and an Optimization Method. IEEE Transactions on Intelligent Transportation Systems. 19. 1-14.](https://www.researchgate.net/publication/322202031_Hierarchical_Trajectory_Planning_of_an_Autonomous_Car_Based_on_the_Integration_of_a_Sampling_and_an_Optimization_Method)



## stop_dist_calc_w_jerk_acc_constraint

### calc_to_generate_csv

calculate stop dist and generate csv.

```
$ pipenv run python stop_dist_calc_w_jerk_acc_constraint/calc_to_generate_csv.py
```

### calc_with_plots
calculate stop dist and plot the result.

```
$ pipenv run python stop_dist_calc_w_jerk_acc_constraint/calc_with_plots.py
```

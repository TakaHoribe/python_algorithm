
# Path Following Sim

## sim_1d_time_delay.py

```
$ pipenv run python ./path_following_sim/sim_1d_time_delay.py
```

<p align="center">
  <img src="./media/sim_res_1d_delay.png" width="800">
</p>


### model

Path following error dynamics with `time_delay: tau` and `time_constant: d`

<p align="left">
  <img src="./media/model.png" width="200">
</p>

### args

`-h` shows help for usage.

```
$ pipenv run python3 ./path_following_sim/sim_1d_time_delay.py -h

usage: sim_1d_time_delay.py [-h] [-D] [-v VELOCITY] [-d TIME_DELAY]
                            [-t TIME_CONSTANT] [-kp P_GAIN] [-kd D_GAIN]

optional arguments:
  -h, --help            show this help message and exit
  -D, --sim_with_delay  set if sim with delay time is needed
  -v VELOCITY, --velocity VELOCITY
                        sim parameter: velocity
  -d TIME_DELAY, --time_delay TIME_DELAY
                        sim parameter: delay time
  -t TIME_CONSTANT, --time_constant TIME_CONSTANT
                        sim parameter: time constant
  -kp P_GAIN, --p_gain P_GAIN
                        sim parameter: p gain
  -kd D_GAIN, --d_gain D_GAIN
                        sim parameter: d gain
```


## param_tune_for_two_models.py

Do grid-search for `PD` gain parameter tuning. Simulation runs for two models to compare (NOTE: this is a very special use case).

You need to set some parameters defined in `param_tune_for_two_models.py`. The `velocity`, `kp_range`, `kd_range` are common parameters, `time_delay` and `time_constant` are for both of model-A, and model-B.

The `len(kp_range) * len(kd_range)` number of figures will be displayed.


```
$ pipenv run python ./path_following_sim/param_tune_for_two_models.py
```

# Ardupilot Passthrough messages format

## __0x5001__ (status message)

32 bit value orgnized as follow:

|  111111  | 000000000000000 |    1    |    1    |    1    |    1    |    1    |    1    |  00010   |
|:--------:|:---------------:|:-------:|:-------:|:-------:|:-------:|:-------:|:-------:|:--------:|
| *6 bits* |    *15 bits*    | *1 bit* | *1 bit* | *1 bit* | *1 bit* | *1 bit* | *1 bit* | *5 bits* |
|  imu_t   |     unused?     | ekf_fs  | bat_fs  |  armd   |   lnd   |  ssmpl  |  smpl   |   mode   |

## __0x5006__ (attitude and rangefinder message)

32 bit value orgnized as follow:

| 11111100000 | 11111100010 | 11111100010 |
|:-----------:|:-----------:|:-----------:|
|  *11 bits*  |  *10 bits*  |  *11 bits*  |
| rng_finder  |    pitch    |    roll     |

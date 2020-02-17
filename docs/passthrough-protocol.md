# Ardupilot Passthrough messages format

## __0x5001__ (Status message)

32 bit value orgnized as follow:

|  111111  | 000000000000000 |    1    |    1    |    1    |    1    |    1    |    1    |  00010   |
|:--------:|:---------------:|:-------:|:-------:|:-------:|:-------:|:-------:|:-------:|:--------:|
| *6 bits* |    *15 bits*    | *1 bit* | *1 bit* | *1 bit* | *1 bit* | *1 bit* | *1 bit* | *5 bits* |
|  imu_t   |     unused?     | ekf_fs  | bat_fs  |  armd   |   lnd   |  ssmpl  |  smpl   |   mode   |

## __0x5002__ (GPS status)

32 bit value orgnized as follow:

| 11111100000 |  11111100   | 01011111 |    10    |   0010   |
|:-----------:|:-----------:|:--------:|:--------:|:--------:|
|  *11 bits*  |  *10 bits*  | *8 bits* | *2 bits* | *4 bits* |
|  alt (msl)  | adv fix tye |  h dop   | fix type | num sats |

## __0x5003__ (Battery1),  __0x5008__ (Battery2)

32 bit value orgnized as follow:

| 111111000001111 | 10001011 | 111100010 |
|:---------------:|:--------:|:---------:|
|    *15 bits*    | *8 bits* | *9 bits*  |
|   curr drawn    | current  |  voltage  |


## __0x5005__ (Vel & Yaw)

32 bit value orgnized as follow:

| 111111000001111 | 10001011 | 111100010 |
|:---------------:|:--------:|:---------:|
|    *15 bits*    | *8 bits* | *9 bits*  |
|       yaw       |  H vel   |   V vel   |


## __0x5006__ (Attitude & rangefinder message)

32 bit value orgnized as follow:

| 11111100000 | 11111100010 | 11111100010 |
|:-----------:|:-----------:|:-----------:|
|  *11 bits*  |  *10 bits*  |  *11 bits*  |
| rng_finder  |    pitch    |    roll     |

## __0x5007__ (Params)

32 bit value orgnized as follow:

| 11111000 | 001111110001011111100010 |
|:--------:|:------------------------:|
| *8 bits* |        *24 bits*         |
| param ID |       param Value        |

## __0x50F2__ (VFR HUD)

32 bit value orgnized as follow:

|       11111        | 000001111110 | 0010111  | 11100010  |
|:------------------:|:------------:|:--------:|:---------:|
|      *5 bits*      |  *12 bits*   | *7 bits* | *8 bits*  |
| baro alt sign (-+) |   baro alt   | throttle | air speed |

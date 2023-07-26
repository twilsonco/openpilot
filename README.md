![intro_vid](https://github.com/twilsonco/openpilot/blob/log-info/comma-steering-control-vid.gif?raw=true)

# Improving controls with log data

Here's plots of the [comma-steering-control](https://github.com/commaai/comma-steering-control) dataset and the data I've collected so far from the community.
Community log collection is in a limited state; only cars not in the comma-steering-control dataset will be collected for the time being.
I'll update here periodically so log contributors can see which speeds/angles need to be filled out.

Head to the Comma, Community, or SunnyPilot Discord server #tuning (or tuning-nnff) channels if you want to contribute or learn more!

## Table of Contents
- [1 Community vehicle log counts](#current-counts-of-collected-logs) (83718 total; 1395.0 hours)
- [2 Community lateral data](https://github.com/twilsonco/openpilot/blob/log-info/2%20Community%20lateral%20data.md) (      66 cars)
- [3a Community lateral torque NNFF fits steer command](https://github.com/twilsonco/openpilot/blob/log-info/3a%20Community%20lateral%20torque%20NNFF%20fits%20steer%20command.md) (42 cars)
- [3b Community lateral torque NNFF fits LKA EPS torque](https://github.com/twilsonco/openpilot/blob/log-info/3b%20Community%20lateral%20torque%20NNFF%20fits%20LKA%20EPS%20torque.md) (34 cars)
- [3c Community lateral torque NNFF fits driver torque](https://github.com/twilsonco/openpilot/blob/log-info/3c%20Community%20lateral%20torque%20NNFF%20fits%20driver%20torque.md) (34 cars)
- [3d Community lateral torque NNFF fits driver EPS combined torque](https://github.com/twilsonco/openpilot/blob/log-info/3d%20Community%20lateral%20torque%20NNFF%20fits%20driver%20EPS%20combined%20torque.md) (32 cars)
- [4 Comma lateral data combined firmware](https://github.com/twilsonco/openpilot/blob/log-info/4%20Comma%20lateral%20data%20combined%20firmware.md) (     121 cars)
- [5a Comma lateral data separate firmwares](https://github.com/twilsonco/openpilot/blob/log-info/5a%20Comma%20lateral%20data%20separate%20firmwares.md) (     526 cars)
- [5b Comma lateral data EPS firmware comparison](https://github.com/twilsonco/openpilot/blob/log-info/5b%20Comma%20lateral%20data%20EPS%20firmware%20comparison.md) (      95 cars)
- [6 Comma lateral torque NNFF fits](https://github.com/twilsonco/openpilot/blob/log-info/6%20Comma%20lateral%20torque%20NNFF%20fits.md) (106 cars)



## Current counts of collected logs


```
nissan                                  598 (691M)
  NISSAN LEAF 2018                      598 (691M)
mazda                                   1308 (1.4G)
  MAZDA CX-5 2022                       15 (21M)
  MAZDA CX-9                            1293 (1.4G)
subaru                                  1740 (2.1G)
  SUBARU LEGACY 2015 - 2018             774 (975M)
  SUBARU IMPREZA SPORT 2020             7 (9.4M)
  SUBARU OUTBACK 6TH GEN                459 (503M)
  SUBARU LEGACY 7TH GEN                 500 (603M)
ford                                    1787 (2.0G)
  FORD F-150 14TH GEN                   902 (1.1G)
  FORD MAVERICK 1ST GEN                 883 (955M)
  MURPHY CAR                            2 (1.9M)
volkswagen                              8379 (9.8G)
  VOLKSWAGEN JETTA 7TH GEN              345 (443M)
  SKODA KODIAQ 1ST GEN                  3172 (3.8G)
  SKODA KAROQ 1ST GEN                   572 (661M)
  VOLKSWAGEN PASSAT 8TH GEN             378 (494M)
  VOLKSWAGEN TIGUAN 2ND GEN             471 (588M)
  VOLKSWAGEN SHARAN 2ND GEN             56 (73M)
  VOLKSWAGEN POLO 6TH GEN               525 (649M)
  VOLKSWAGEN PASSAT NMS                 1459 (1.7G)
  VOLKSWAGEN GOLF 7TH GEN               744 (859M)
  AUDI A3 3RD GEN                       656 (737M)
toyota                                  13242 (16G)
  TOYOTA SIENNA 2018                    13 (16M)
  TOYOTA COROLLA 2017                   2 (612K)
  TOYOTA HIGHLANDER HYBRID 2020         10 (12M)
  TOYOTA RAV4 2019                      1114 (1.4G)
  LEXUS RX 2020                         95 (112M)
  TOYOTA PRIUS TSS2 2021                603 (741M)
  LEXUS ES 2019                         46 (54M)
  TOYOTA PRIUS v 2017                     1489 (1.9G)
  TOYOTA PRIUS 2017                     1606 (1.9G)
  TOYOTA CAMRY HYBRID 2021              1022 (1.2G)
  TOYOTA COROLLA HYBRID TSS2 2019       2261 (2.6G)
  TOYOTA COROLLA TSS2 2019              544 (594M)
  TOYOTA RAV4 HYBRID 2019               4437 (5.3G)
hyundai                                 17828 (22G)
  KIA NIRO HYBRID 2021                  367 (446M)
  KIA NIRO HYBRID 2ND GEN               190 (243M)
  HYUNDAI SANTA FE HYBRID 2022          1375 (1.8G)
  KIA NIRO HYBRID 2019                  75 (91M)
  KIA NIRO EV 2020                      246 (282M)
  KIA CEED INTRO ED 2019                2371 (2.6G)
  HYUNDAI SANTA FE 2019                 614 (771M)
  HYUNDAI KONA ELECTRIC 2022            116 (148M)
  HYUNDAI IONIQ HYBRID 2017-2019        12 (13M)
  HYUNDAI SANTA FE 2022                 1 (512)
  GENESIS GV70 1ST GEN                  1304 (1.6G)
  GENESIS (DH)                          156 (186M)
  KIA SPORTAGE 5TH GEN                  2049 (2.5G)
  HYUNDAI IONIQ 5 2022                  1223 (1.5G)
  KIA EV6 2022                          4659 (5.6G)
  HYUNDAI SONATA 2020                   2331 (2.9G)
  HYUNDAI PALISADE 2020                 738 (871M)
honda                                   4141 (4.7G)
  HONDA CIVIC (BOSCH) 2019              346 (419M)
  HONDA CLARITY 2018                    2 (1.9M)
  HONDA ACCORD 2018                     565 (681M)
  HONDA HR-V 2023                       126 (147M)
  HONDA HRV                             1 (512)
  HONDA CIVIC 2022                      22 (27M)
  HONDA RIDGELINE 2017                  3078 (3.5G)
chrysler                                7046 (8.1G)
  RAM HD 5TH GEN                        851 (1.1G)
  RAM 1500 5TH GEN                      4260 (4.9G)
  CHRYSLER PACIFICA HYBRID 2019         896 (1.1G)
  CHRYSLER PACIFICA 2018                1039 (1.2G)
gm                                      27471 (33G)
  CHEVROLET TRAILBLAZER 2021            801 (991M)
  CHEVROLET BOLT EV NO ACC              916 (990M)
  GMC ACADIA DENALI 2018                5843 (6.8G)
  CHEVROLET SUBURBAN PREMIER 2019       58 (72M)
  CHEVROLET SILVERADO 1500 2020         4581 (5.5G)
  CHEVROLET BOLT EUV 2022               5876 (6.7G)
  BUICK LACROSSE 2017                   93 (105M)
  CHEVROLET VOLT PREMIER 2018           6274 (8.4G)
  CHEVROLET VOLT PREMIER 2017           3028 (3.5G)
```

Last updated July 25, 2023

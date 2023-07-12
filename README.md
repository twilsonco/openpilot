# Improving controls with log data

Here's the data I've collected so far from the community. I'll update here periodically so you can see which speeds/angles need to be filled out.

Head to the SunnyPilot Discord server #tuning-nnff channel if you want to contribute or learn more!

## Table of Contents
- [Vehicle log counts](#current-counts-of-collected-logs) (83718 total)
- [Lateral data](#lateral-data) (      66 cars)
- [Lateral torque NNFF fits](#lateral-torque-nnff-fits) (      40 cars)



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

## Lateral data


| 🛣️ | 🚗 |
| --- | --- |
| [AUDI A3 3RD GEN](#table-of-contents)  ![AUDI A3 3RD GEN](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/AUDI%20A3%203RD%20GEN.png?raw=true)| [BUICK LACROSSE 2017](#table-of-contents)  ![BUICK LACROSSE 2017](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/BUICK%20LACROSSE%202017.png?raw=true)|
| [CHEVROLET BOLT EUV 2022](#table-of-contents)  ![CHEVROLET BOLT EUV 2022](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/CHEVROLET%20BOLT%20EUV%202022.png?raw=true)| [CHEVROLET BOLT EV NO ACC](#table-of-contents)  ![CHEVROLET BOLT EV NO ACC](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/CHEVROLET%20BOLT%20EV%20NO%20ACC.png?raw=true)|
| [CHEVROLET SILVERADO 1500 2020](#table-of-contents)  ![CHEVROLET SILVERADO 1500 2020](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/CHEVROLET%20SILVERADO%201500%202020.png?raw=true)| [CHEVROLET SUBURBAN PREMIER 2019](#table-of-contents)  ![CHEVROLET SUBURBAN PREMIER 2019](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/CHEVROLET%20SUBURBAN%20PREMIER%202019.png?raw=true)|
| [CHEVROLET TRAILBLAZER 2021](#table-of-contents)  ![CHEVROLET TRAILBLAZER 2021](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/CHEVROLET%20TRAILBLAZER%202021.png?raw=true)| [CHEVROLET VOLT PREMIER 2017](#table-of-contents)  ![CHEVROLET VOLT PREMIER 2017](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/CHEVROLET%20VOLT%20PREMIER%202017.png?raw=true)|
| [CHEVROLET VOLT PREMIER 2018](#table-of-contents)  ![CHEVROLET VOLT PREMIER 2018](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/CHEVROLET%20VOLT%20PREMIER%202018.png?raw=true)| [CHRYSLER PACIFICA 2018](#table-of-contents)  ![CHRYSLER PACIFICA 2018](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/CHRYSLER%20PACIFICA%202018.png?raw=true)|
| [CHRYSLER PACIFICA HYBRID 2019](#table-of-contents)  ![CHRYSLER PACIFICA HYBRID 2019](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/CHRYSLER%20PACIFICA%20HYBRID%202019.png?raw=true)| [FORD F-150 14TH GEN](#table-of-contents)  ![FORD F-150 14TH GEN](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/FORD%20F-150%2014TH%20GEN.png?raw=true)|
| [FORD MAVERICK 1ST GEN](#table-of-contents)  ![FORD MAVERICK 1ST GEN](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/FORD%20MAVERICK%201ST%20GEN.png?raw=true)| [GENESIS (DH)](#table-of-contents)  ![GENESIS (DH)](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/GENESIS%20(DH).png?raw=true)|
| [GENESIS GV70 1ST GEN](#table-of-contents)  ![GENESIS GV70 1ST GEN](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/GENESIS%20GV70%201ST%20GEN.png?raw=true)| [GMC ACADIA DENALI 2018](#table-of-contents)  ![GMC ACADIA DENALI 2018](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/GMC%20ACADIA%20DENALI%202018.png?raw=true)|
| [HONDA ACCORD 2018](#table-of-contents)  ![HONDA ACCORD 2018](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/HONDA%20ACCORD%202018.png?raw=true)| [HONDA CIVIC (BOSCH) 2019](#table-of-contents)  ![HONDA CIVIC (BOSCH) 2019](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/HONDA%20CIVIC%20(BOSCH)%202019.png?raw=true)|
| [HONDA CIVIC 2022](#table-of-contents)  ![HONDA CIVIC 2022](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/HONDA%20CIVIC%202022.png?raw=true)| [HONDA CLARITY 2018](#table-of-contents)  ![HONDA CLARITY 2018](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/HONDA%20CLARITY%202018.png?raw=true)|
| [HONDA HR-V 2023](#table-of-contents)  ![HONDA HR-V 2023](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/HONDA%20HR-V%202023.png?raw=true)| [HONDA RIDGELINE 2017](#table-of-contents)  ![HONDA RIDGELINE 2017](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/HONDA%20RIDGELINE%202017.png?raw=true)|
| [HYUNDAI IONIQ 5 2022](#table-of-contents)  ![HYUNDAI IONIQ 5 2022](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/HYUNDAI%20IONIQ%205%202022.png?raw=true)| [HYUNDAI IONIQ HYBRID 2017-2019](#table-of-contents)  ![HYUNDAI IONIQ HYBRID 2017-2019](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/HYUNDAI%20IONIQ%20HYBRID%202017-2019.png?raw=true)|
| [HYUNDAI KONA ELECTRIC 2022](#table-of-contents)  ![HYUNDAI KONA ELECTRIC 2022](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/HYUNDAI%20KONA%20ELECTRIC%202022.png?raw=true)| [HYUNDAI PALISADE 2020](#table-of-contents)  ![HYUNDAI PALISADE 2020](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/HYUNDAI%20PALISADE%202020.png?raw=true)|
| [HYUNDAI SANTA FE 2019](#table-of-contents)  ![HYUNDAI SANTA FE 2019](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/HYUNDAI%20SANTA%20FE%202019.png?raw=true)| [HYUNDAI SANTA FE HYBRID 2022](#table-of-contents)  ![HYUNDAI SANTA FE HYBRID 2022](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/HYUNDAI%20SANTA%20FE%20HYBRID%202022.png?raw=true)|
| [HYUNDAI SONATA 2020](#table-of-contents)  ![HYUNDAI SONATA 2020](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/HYUNDAI%20SONATA%202020.png?raw=true)| [KIA CEED INTRO ED 2019](#table-of-contents)  ![KIA CEED INTRO ED 2019](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/KIA%20CEED%20INTRO%20ED%202019.png?raw=true)|
| [KIA EV6 2022](#table-of-contents)  ![KIA EV6 2022](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/KIA%20EV6%202022.png?raw=true)| [KIA NIRO EV 2020](#table-of-contents)  ![KIA NIRO EV 2020](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/KIA%20NIRO%20EV%202020.png?raw=true)|
| [KIA NIRO HYBRID 2019](#table-of-contents)  ![KIA NIRO HYBRID 2019](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/KIA%20NIRO%20HYBRID%202019.png?raw=true)| [KIA NIRO HYBRID 2ND GEN](#table-of-contents)  ![KIA NIRO HYBRID 2ND GEN](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/KIA%20NIRO%20HYBRID%202ND%20GEN.png?raw=true)|
| [KIA SPORTAGE 5TH GEN](#table-of-contents)  ![KIA SPORTAGE 5TH GEN](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/KIA%20SPORTAGE%205TH%20GEN.png?raw=true)| [LEXUS ES 2019](#table-of-contents)  ![LEXUS ES 2019](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/LEXUS%20ES%202019.png?raw=true)|
| [LEXUS RX 2020](#table-of-contents)  ![LEXUS RX 2020](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/LEXUS%20RX%202020.png?raw=true)| [MAZDA CX-5 2022](#table-of-contents)  ![MAZDA CX-5 2022](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/MAZDA%20CX-5%202022.png?raw=true)|
| [MAZDA CX-9](#table-of-contents)  ![MAZDA CX-9](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/MAZDA%20CX-9.png?raw=true)| [MURPHY CAR](#table-of-contents)  ![MURPHY CAR](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/MURPHY%20CAR.png?raw=true)|
| [NISSAN LEAF 2018](#table-of-contents)  ![NISSAN LEAF 2018](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/NISSAN%20LEAF%202018.png?raw=true)| [RAM 1500 5TH GEN](#table-of-contents)  ![RAM 1500 5TH GEN](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/RAM%201500%205TH%20GEN.png?raw=true)|
| [RAM HD 5TH GEN](#table-of-contents)  ![RAM HD 5TH GEN](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/RAM%20HD%205TH%20GEN.png?raw=true)| [SKODA KAROQ 1ST GEN](#table-of-contents)  ![SKODA KAROQ 1ST GEN](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/SKODA%20KAROQ%201ST%20GEN.png?raw=true)|
| [SKODA KODIAQ 1ST GEN](#table-of-contents)  ![SKODA KODIAQ 1ST GEN](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/SKODA%20KODIAQ%201ST%20GEN.png?raw=true)| [SUBARU IMPREZA SPORT 2020](#table-of-contents)  ![SUBARU IMPREZA SPORT 2020](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/SUBARU%20IMPREZA%20SPORT%202020.png?raw=true)|
| [SUBARU LEGACY 2015 - 2018](#table-of-contents)  ![SUBARU LEGACY 2015 - 2018](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/SUBARU%20LEGACY%202015%20-%202018.png?raw=true)| [SUBARU LEGACY 7TH GEN](#table-of-contents)  ![SUBARU LEGACY 7TH GEN](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/SUBARU%20LEGACY%207TH%20GEN.png?raw=true)|
| [SUBARU OUTBACK 6TH GEN](#table-of-contents)  ![SUBARU OUTBACK 6TH GEN](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/SUBARU%20OUTBACK%206TH%20GEN.png?raw=true)| [TOYOTA CAMRY HYBRID 2021](#table-of-contents)  ![TOYOTA CAMRY HYBRID 2021](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/TOYOTA%20CAMRY%20HYBRID%202021.png?raw=true)|
| [TOYOTA COROLLA 2017](#table-of-contents)  ![TOYOTA COROLLA 2017](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/TOYOTA%20COROLLA%202017.png?raw=true)| [TOYOTA COROLLA HYBRID TSS2 2019](#table-of-contents)  ![TOYOTA COROLLA HYBRID TSS2 2019](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/TOYOTA%20COROLLA%20HYBRID%20TSS2%202019.png?raw=true)|
| [TOYOTA COROLLA TSS2 2019](#table-of-contents)  ![TOYOTA COROLLA TSS2 2019](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/TOYOTA%20COROLLA%20TSS2%202019.png?raw=true)| [TOYOTA HIGHLANDER HYBRID 2020](#table-of-contents)  ![TOYOTA HIGHLANDER HYBRID 2020](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/TOYOTA%20HIGHLANDER%20HYBRID%202020.png?raw=true)|
| [TOYOTA PRIUS 2017](#table-of-contents)  ![TOYOTA PRIUS 2017](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/TOYOTA%20PRIUS%202017.png?raw=true)| [TOYOTA PRIUS TSS2 2021](#table-of-contents)  ![TOYOTA PRIUS TSS2 2021](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/TOYOTA%20PRIUS%20TSS2%202021.png?raw=true)|
| [TOYOTA PRIUS v 2017](#table-of-contents)  ![TOYOTA PRIUS v 2017](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/TOYOTA%20PRIUS%20v%202017.png?raw=true)| [TOYOTA RAV4 2019](#table-of-contents)  ![TOYOTA RAV4 2019](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/TOYOTA%20RAV4%202019.png?raw=true)|
| [TOYOTA RAV4 HYBRID 2019](#table-of-contents)  ![TOYOTA RAV4 HYBRID 2019](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/TOYOTA%20RAV4%20HYBRID%202019.png?raw=true)| [VOLKSWAGEN GOLF 7TH GEN](#table-of-contents)  ![VOLKSWAGEN GOLF 7TH GEN](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/VOLKSWAGEN%20GOLF%207TH%20GEN.png?raw=true)|
| [VOLKSWAGEN JETTA 7TH GEN](#table-of-contents)  ![VOLKSWAGEN JETTA 7TH GEN](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/VOLKSWAGEN%20JETTA%207TH%20GEN.png?raw=true)| [VOLKSWAGEN PASSAT 8TH GEN](#table-of-contents)  ![VOLKSWAGEN PASSAT 8TH GEN](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/VOLKSWAGEN%20PASSAT%208TH%20GEN.png?raw=true)|
| [VOLKSWAGEN PASSAT NMS](#table-of-contents)  ![VOLKSWAGEN PASSAT NMS](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/VOLKSWAGEN%20PASSAT%20NMS.png?raw=true)| [VOLKSWAGEN POLO 6TH GEN](#table-of-contents)  ![VOLKSWAGEN POLO 6TH GEN](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/VOLKSWAGEN%20POLO%206TH%20GEN.png?raw=true)|
| [VOLKSWAGEN SHARAN 2ND GEN](#table-of-contents)  ![VOLKSWAGEN SHARAN 2ND GEN](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/VOLKSWAGEN%20SHARAN%202ND%20GEN.png?raw=true)| [VOLKSWAGEN TIGUAN 2ND GEN](#table-of-contents)  ![VOLKSWAGEN TIGUAN 2ND GEN](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20data/VOLKSWAGEN%20TIGUAN%202ND%20GEN.png?raw=true)|

## Lateral torque NNFF fits


| 🛣️ | 🚗 |
| --- | --- |
| [AUDI A3 3RD GEN](#table-of-contents)  ![AUDI A3 3RD GEN](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20torque%20NNFF%20fits/AUDI%20A3%203RD%20GEN.png?raw=true)| [BUICK LACROSSE 2017](#table-of-contents)  ![BUICK LACROSSE 2017](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20torque%20NNFF%20fits/BUICK%20LACROSSE%202017.png?raw=true)|
| [CHEVROLET BOLT EUV 2022](#table-of-contents)  ![CHEVROLET BOLT EUV 2022](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20torque%20NNFF%20fits/CHEVROLET%20BOLT%20EUV%202022.png?raw=true)| [CHEVROLET BOLT EV NO ACC](#table-of-contents)  ![CHEVROLET BOLT EV NO ACC](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20torque%20NNFF%20fits/CHEVROLET%20BOLT%20EV%20NO%20ACC.png?raw=true)|
| [CHEVROLET SILVERADO 1500 2020](#table-of-contents)  ![CHEVROLET SILVERADO 1500 2020](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20torque%20NNFF%20fits/CHEVROLET%20SILVERADO%201500%202020.png?raw=true)| [CHEVROLET SUBURBAN PREMIER 2019](#table-of-contents)  ![CHEVROLET SUBURBAN PREMIER 2019](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20torque%20NNFF%20fits/CHEVROLET%20SUBURBAN%20PREMIER%202019.png?raw=true)|
| [CHEVROLET TRAILBLAZER 2021](#table-of-contents)  ![CHEVROLET TRAILBLAZER 2021](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20torque%20NNFF%20fits/CHEVROLET%20TRAILBLAZER%202021.png?raw=true)| [CHEVROLET VOLT PREMIER 2017](#table-of-contents)  ![CHEVROLET VOLT PREMIER 2017](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20torque%20NNFF%20fits/CHEVROLET%20VOLT%20PREMIER%202017.png?raw=true)|
| [CHRYSLER PACIFICA HYBRID 2019](#table-of-contents)  ![CHRYSLER PACIFICA HYBRID 2019](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20torque%20NNFF%20fits/CHRYSLER%20PACIFICA%20HYBRID%202019.png?raw=true)| [FORD MAVERICK 1ST GEN](#table-of-contents)  ![FORD MAVERICK 1ST GEN](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20torque%20NNFF%20fits/FORD%20MAVERICK%201ST%20GEN.png?raw=true)|
| [GENESIS GV70 1ST GEN](#table-of-contents)  ![GENESIS GV70 1ST GEN](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20torque%20NNFF%20fits/GENESIS%20GV70%201ST%20GEN.png?raw=true)| [GMC ACADIA DENALI 2018](#table-of-contents)  ![GMC ACADIA DENALI 2018](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20torque%20NNFF%20fits/GMC%20ACADIA%20DENALI%202018.png?raw=true)|
| [HONDA ACCORD 2018](#table-of-contents)  ![HONDA ACCORD 2018](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20torque%20NNFF%20fits/HONDA%20ACCORD%202018.png?raw=true)| [HONDA CIVIC 2022](#table-of-contents)  ![HONDA CIVIC 2022](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20torque%20NNFF%20fits/HONDA%20CIVIC%202022.png?raw=true)|
| [HONDA RIDGELINE 2017](#table-of-contents)  ![HONDA RIDGELINE 2017](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20torque%20NNFF%20fits/HONDA%20RIDGELINE%202017.png?raw=true)| [HYUNDAI IONIQ 5 2022](#table-of-contents)  ![HYUNDAI IONIQ 5 2022](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20torque%20NNFF%20fits/HYUNDAI%20IONIQ%205%202022.png?raw=true)|
| [HYUNDAI PALISADE 2020](#table-of-contents)  ![HYUNDAI PALISADE 2020](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20torque%20NNFF%20fits/HYUNDAI%20PALISADE%202020.png?raw=true)| [HYUNDAI SONATA 2020](#table-of-contents)  ![HYUNDAI SONATA 2020](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20torque%20NNFF%20fits/HYUNDAI%20SONATA%202020.png?raw=true)|
| [KIA EV6 2022](#table-of-contents)  ![KIA EV6 2022](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20torque%20NNFF%20fits/KIA%20EV6%202022.png?raw=true)| [KIA SPORTAGE 5TH GEN](#table-of-contents)  ![KIA SPORTAGE 5TH GEN](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20torque%20NNFF%20fits/KIA%20SPORTAGE%205TH%20GEN.png?raw=true)|
| [LEXUS ES 2019](#table-of-contents)  ![LEXUS ES 2019](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20torque%20NNFF%20fits/LEXUS%20ES%202019.png?raw=true)| [MAZDA CX-9](#table-of-contents)  ![MAZDA CX-9](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20torque%20NNFF%20fits/MAZDA%20CX-9.png?raw=true)|
| [RAM 1500 5TH GEN](#table-of-contents)  ![RAM 1500 5TH GEN](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20torque%20NNFF%20fits/RAM%201500%205TH%20GEN.png?raw=true)| [RAM HD 5TH GEN](#table-of-contents)  ![RAM HD 5TH GEN](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20torque%20NNFF%20fits/RAM%20HD%205TH%20GEN.png?raw=true)|
| [SKODA KAROQ 1ST GEN](#table-of-contents)  ![SKODA KAROQ 1ST GEN](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20torque%20NNFF%20fits/SKODA%20KAROQ%201ST%20GEN.png?raw=true)| [SKODA KODIAQ 1ST GEN](#table-of-contents)  ![SKODA KODIAQ 1ST GEN](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20torque%20NNFF%20fits/SKODA%20KODIAQ%201ST%20GEN.png?raw=true)|
| [SUBARU LEGACY 7TH GEN](#table-of-contents)  ![SUBARU LEGACY 7TH GEN](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20torque%20NNFF%20fits/SUBARU%20LEGACY%207TH%20GEN.png?raw=true)| [SUBARU OUTBACK 6TH GEN](#table-of-contents)  ![SUBARU OUTBACK 6TH GEN](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20torque%20NNFF%20fits/SUBARU%20OUTBACK%206TH%20GEN.png?raw=true)|
| [TOYOTA CAMRY HYBRID 2021](#table-of-contents)  ![TOYOTA CAMRY HYBRID 2021](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20torque%20NNFF%20fits/TOYOTA%20CAMRY%20HYBRID%202021.png?raw=true)| [TOYOTA COROLLA HYBRID TSS2 2019](#table-of-contents)  ![TOYOTA COROLLA HYBRID TSS2 2019](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20torque%20NNFF%20fits/TOYOTA%20COROLLA%20HYBRID%20TSS2%202019.png?raw=true)|
| [TOYOTA COROLLA TSS2 2019](#table-of-contents)  ![TOYOTA COROLLA TSS2 2019](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20torque%20NNFF%20fits/TOYOTA%20COROLLA%20TSS2%202019.png?raw=true)| [TOYOTA PRIUS 2017](#table-of-contents)  ![TOYOTA PRIUS 2017](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20torque%20NNFF%20fits/TOYOTA%20PRIUS%202017.png?raw=true)|
| [TOYOTA PRIUS TSS2 2021](#table-of-contents)  ![TOYOTA PRIUS TSS2 2021](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20torque%20NNFF%20fits/TOYOTA%20PRIUS%20TSS2%202021.png?raw=true)| [TOYOTA PRIUS v 2017](#table-of-contents)  ![TOYOTA PRIUS v 2017](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20torque%20NNFF%20fits/TOYOTA%20PRIUS%20v%202017.png?raw=true)|
| [TOYOTA RAV4 HYBRID 2019](#table-of-contents)  ![TOYOTA RAV4 HYBRID 2019](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20torque%20NNFF%20fits/TOYOTA%20RAV4%20HYBRID%202019.png?raw=true)| [VOLKSWAGEN GOLF 7TH GEN](#table-of-contents)  ![VOLKSWAGEN GOLF 7TH GEN](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20torque%20NNFF%20fits/VOLKSWAGEN%20GOLF%207TH%20GEN.png?raw=true)|
| [VOLKSWAGEN JETTA 7TH GEN](#table-of-contents)  ![VOLKSWAGEN JETTA 7TH GEN](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20torque%20NNFF%20fits/VOLKSWAGEN%20JETTA%207TH%20GEN.png?raw=true)| [VOLKSWAGEN PASSAT 8TH GEN](#table-of-contents)  ![VOLKSWAGEN PASSAT 8TH GEN](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20torque%20NNFF%20fits/VOLKSWAGEN%20PASSAT%208TH%20GEN.png?raw=true)|
| [VOLKSWAGEN PASSAT NMS](#table-of-contents)  ![VOLKSWAGEN PASSAT NMS](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20torque%20NNFF%20fits/VOLKSWAGEN%20PASSAT%20NMS.png?raw=true)| [VOLKSWAGEN TIGUAN 2ND GEN](#table-of-contents)  ![VOLKSWAGEN TIGUAN 2ND GEN](https://github.com/twilsonco/openpilot/blob/log-info/data/Lateral%20torque%20NNFF%20fits/VOLKSWAGEN%20TIGUAN%202ND%20GEN.png?raw=true)|

Last updated July 12, 2023

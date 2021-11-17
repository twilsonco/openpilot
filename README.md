# Table of Contents
------
* [Appreciate my work?](#appreciate-my-work)
* [Infographics](#infographics)
* [Fork Details](#fork-details)
  * [Current Fork Features](#current-fork-features---optional-via-toggle)
  * [Planned Fork Features](#planned-fork-features-in-no-particular-order)
  * [Supported Hardware](#supported-hardware)
  * [Installation Instructions](#installation-instructions)
  * [Automatic Updates](#automatic-updates)
  * [Tuning](#tuning)
* [Commaai Table of Contents](#commaai-table-of-contents)

### Infographics
------
![](https://user-images.githubusercontent.com/37757984/127420744-89ca219c-8f8e-46d3-bccf-c1cb53b81bb1.png)
![](https://github.com/twilsonco/openpilot/blob/tw-0.8.9-dev/fork_image_onepedal.png?raw=true)
![](https://github.com/twilsonco/openpilot/blob/tw-0.8.9-dev/fork_image_touchcontrols.png?raw=true)
![](https://github.com/twilsonco/openpilot/blob/tw-0.8.9-dev/fork_image_misc.png?raw=true)

### Appreciate My Work?
------

**[Buy me a beer/coffee](https://www.patreon.com/twilsonco)***

# Fork Details
------

This fork exists to improve OP performance and convenience for GM cars, specifically the Chevy Volt, because the kegman fork wouldn't run on Comma Three.
The result is the best volt driving experience there's ever been.

Most of the features in this fork are my original implementations, often inspired and always informed by the work of others in the community. Pleae provide any positive/negative feedback on the comma/openpilot/retropilot Discord servers, especially if you'd like your issues addressed.

Running on move-fast fork of openpilot, which adds:

* Vision and/or map-based slowing down for curves
* Map-based automatic changing of speed limit (with optional offset)
* Hands on wheel monitoring
* Disable disengage when gas pressed

#### Current Fork Features [✅ = optional via toggle]:
-----

- [x] Latest openpilot 0.8.10 lateral and driver monitoring models
- [x] [Comma3] Latest AGNOS2 OS
- [x] [Chevy Volt] Sigmoidal steering response (thanks Qadmus)
- [x] [GM] [✅] AutoHold (autohold brakes when stopped; ported from kegman)
- [x] [GM] Adjustable follow "mode" using ACC distance button (ported from kegman, but smoother follow profiles)
- [x] [GM] Toggle steering with LKAS button (wheel color changes to indicate disengagement)
- [x] [GM] One-pedal driving a.k.a. autosteering only a.k.a. toggle longitudinal control: using regen (volt) and/or light/moderate/heavy braking, control OP all the way to a stop, without a lead, and without disengaging, with just the gas pedal (see below)
- [x] [✅] [Dynamic Lane Profile](https://github.com/sunnyhaibin/openpilot#new-dynamic-lane-profile-dlp) (DLP); *tap button while driving to switch between auto/laneless/lane-only. must enable "Disable use of lanelines" for button to appear* (ported from sunnyhaibin)
- [x] [✅] Normal/sport acceleration modes with improved acceleration/braking profiles (ported from kegman)
- [x] [✅] 1/5 mph changes for tap/hold of the inc/dec buttons (ported from Spector56)
- [x] [✅] 3mph cruise speed offset: speed will be 23/28/33/38/etc.
- [x] [✅] Alternate sound effect set
- [x] [✅] Mute engage and disengage sounds
- [x] [✅] Coasting: OP will still brake behind a lead car and to slow down for curves, but will not apply engine/regen/friction brakes in order to keep the set speed (by user or map speed limit)
    * Toggle coasting while driving by tapping the max speed indicator
    * A "+" after the max speed indicates that coasting is enabled
    * *Can be a bit rough on the brakes when following downhill over set speed; recommend to disable if uncomfortable when constantly following downhill*
- [x] [✅] Brake when 15% over set speed when coasting enabled
- [x] [✅] Nudgeless lane change: OP will start lane change automatically in direction of blinker after blinker on for 3s
- [x] [✅] Friction braking indicator
- [x] **Customizable, dynamic vehicle/device metrics**
    * To use:
        * Tap the current speed on the openpilot display to cycle the number of metrics
        * Tap any metric to cycle its content (sorry for all the god-forsaken tapping, a better metric display with vehicle, following, position, and device widgets is a WIP)
    * Metrics (24 to choose from):
        * Device info: CPU temperature, CPU percent, CPU temp + percent, memory temperature, memory used, free storage, ambient temperature, fanspeed (as percent of max), GPS accuracy (and number of satelites), altitude
        * Vehicle info: Engine RPM, steering torque, steering angle, desired steering angle, vehicle acceleration, vehicle jerk, percent grade of current road
        * Lead-following info: follow distance level, lead distance [length], desired lead distance [length], lead distance [time], desired lead distance [time], follow distance and acceleration mpc costs [in units of the stock OP costs; i.e. 2.5 means 2.5× the stock OP value], relative lead velocity, absolute lead velocity
- [x] [GM] [✅] **One-pedal driving**: OP will apply light to heavy braking when you let completely off the gas, allowing you to come to a full stop and resume without OP disengaging
    * Must have disable disengage on gas toggle enabled
    * **Not necessary to enable the one-pedal toggle; you engage/disengage while driving**
    * When blinker is on below 30mph, autosteer will automatically pause [optional; tap wheel icon to toggle while in one-pedal mode; a second white circle around the wheel icon indicates autosteer pause is enabled]
    * Engage in three ways
      1. While cruise is set, press and hold the follow distance button for 0.5s (continue to hold for immediate hard braking if necessary)
      2. If one-pedal engage on gas toggle is enabled, press gas while cruise is set and traveling above 1mph
      3. While cruise is set, lower cruise speed to 1
    * When in one-pedal mode, the max speed indicator in openpilot will be replaced with a one-pedal mode indicator. Tap the one-pedal icon (or use follow distance button, see below) to toggle coasting/braking
    * Vehicle follow distance indicator and pedal icon color indicate the one-pedal braking profile in use; 1/2/3 = 🟢/🟠/🔴 = light/moderate/heavy braking
    * Control braking with follow distance button:
      * *Single press*: alternate between persistent light or moderate braking
      * *Press and hold*: apply temporary hard braking (indicated by follow level 3 on vehicle cluster and red one-pedal icon) (Chevy's the ones that decided a brake paddle on the steering wheel was a good idea; not me)
      * *Press when friction braking disabled*: activating friction braking
      * *Double-press when stopped or when gas is pressed and friction braking is active*: deactivate friction braking
- [x] [GM] [✅] One-pedal pro braking: Completely disable cruise/speed limit/curve/follow braking when in one-pedal mode. You are soley responsible for slowing the car using the adjustable one-pedal braking (by pressing/holding the follow distance button) or with the physical brakes/regen paddle
- [x] [GM] [✅] One-pedal engage on gas: When cruising at speed and the driver presses the gas (i.e. not when resuming from a stop), engage one-pedal/always-on-steering mode
    * Toggle while driving by tapping the pedal icon
- [x] [GM] JShuler panda-based GM steering fault fix
- [x] Remember last follow mode

#### Planned Fork Features (in no particular order):
-----

- [ ] Intellibeam (auto-bright headlights based on oncoming cars)
- [ ] Stop-and-go for 2018 Volt
- [ ] Grey panda support
- [ ] Chevy Bolt support
- [ ] Record screen button
- [ ] Auto engage parking brake
- [ ] Live tuner
- [ ] Redo UI metrics as themed "widgets" instead that can be activated independently and stack against the right (and left if necessary) side of the screen
  * Follow widget: a colored vertical bar indicating follow distance with lines indicating the actual and desired (length/time) follow distances. Tap to include more info items like current distance cost
  * Openpilot widget: a similar vertical bar (or maybe something like a circular progress bar or a speedometer--looking thing) showing the gas/braking being requested by OP. Also include Driver monitoring info.
  * Car widget: Acceleration/jerk, tire pressures, low voltage battery info, ...
  * Geo widget: GPS signal/coords/#satellites, altitude, percent grade of current road, ...
  * Device widget: CPU/memory/temps/fans/...
  * EV widget: high voltage battery info similar to that shown in the LeafSpyPro app
- [ ] [✅] [Modified assistive driving system](https://github.com/sunnyhaibin/openpilot#new-modified-assistive-driving-safety-mads) (MADS)
- [ ] [✅] 0.5 second delay before activating newly selected follow mode so user can switch around without OP slightly jerking in response
- [ ] [✅] Auto screen brightness (or at least a way to dim a bit at night)
- [ ] [✅] Lane Speed alerts ([sshane](https://github.com/sshane/openpilot#lane-speed-alerts))
- [ ] [✅] Dynamic camera offset (based on oncoming traffic) ([sshane)](https://github.com/sshane/openpilot#dynamic-camera-offset-based-on-oncoming-traffic)
- [ ] [Chevy Volt] Steering control below 7mph using parking commands
- [ ] [Chevy Volt] [✅] Road trip mode: automatically put car into Mountain Mode if sustained speed 55mph+
- [ ] [GM] Use physical drive mode button to switch between normal/sport acceleration profiles
- [ ] [GM] [✅] Dynamic follow mode: point-based
    * Follow distance "earns points" the longer you're behind the same lead car, moving from close to medium after about 5 minutes
    * When on highway, continue to increase from medium to far follow distance after about 20 minutes behind the same car
    * If someone cuts in, the follow distance "takes a penalty" down to a closer follow distance proportional to the distance and relative speed of the car that cut in
    * The penalties can "go negative", that is, repeated cut-ins can result in close follow being use for longer than normal
* Metrics to add:
    - [ ] number of interventions/cut-ins during drive session
    - [ ] time since last intervention/cut-in
    - [ ] apply gas/apply brake

### Supported Hardware
------

This fork is developed and used on a Comma Three in a 2018 Chevy Volt, and is also *known* to work on Comma Two and Comma Zero, and in 2017 Volt and 2018 Acadia.

### Installation Instructions
------

#### Easy: using sshane's [openpilot-installer-generator](https://github.com/sshane/openpilot-installer-generator)

Use [these instructions](https://github.com/sshane/openpilot-installer-generator#usage) and the following url:
`https://smiskol.com/fork/twilsonco`

To ride the bleeding edge, try the staging branch where new features are tested before they go to regular users:
(Be extra diligent and attentive when using the staging branch; it is considered experimental moreso than the regular branch!)
`https://smiskol.com/fork/twilsonco/tw-0.8.9-staging`

#### Less easy

With a stock installation of OpenPilot confirmed working, SSH into device and run the following:

`cd /data;mv openpilot openpilot_stock;git clone --recurse-submodules https://github.com/twilsonco/openpilot;sudo reboot`

### Automatic Updates
------

This fork will auto-update while your device has internet access, and changes are automatically applied the next time the device restarts.
If you're device stays connected to your car all the time, you'll be presented with a message to update when your car is off.

### Tuning
------

* Remember to make small adjustments to 1 value at a time and then test.
* Use [PlotJugger](https://github.com/commaai/openpilot/tree/master/tools/plotjuggler) to make sure you are going in the right direction.

#### Lateral Tuning
------
**Note**: All of these parameters interact with each other so finding the balance is a bit experimental.

* **Kp too high** - The vehicle overshoots and undershoots center.
* **Kp too low** - The vehicle doesn't turn enough.

* **Ki too high** - The vehicle gets to center without oscillations, but it takes too long to center. If you hit a bump or give the wheel a quick nudge, it should oscillate 3 - 5 times before coming to steady-state. If the wheel oscillates forever (critically damped), then your Kp or Ki or both are too high.
* **Ki too low** - The vehicle oscillates trying to reach the center.

* **steerRatio too high** - The vehicle ping pongs on straights and turns. If you're on a turn and the wheel is oversteering and then correcting, steerRatio is too high, and it's fighting with Kp and Ki (which you don't want) - although in the past it has been observed having an oscillating oversteering tune which could do tighter turns, but the turns weren't pleasant.

* **steerRatio too low** - The vehicle doesn't turn enough on curves.

* **Kf** - Lower this if your car oscillates and you've done everything else. It can be lowered to 0.

---

![](https://user-images.githubusercontent.com/37757984/127420744-89ca219c-8f8e-46d3-bccf-c1cb53b81bb1.png)

Commaai Table of Contents
=======================

* [What is openpilot?](#what-is-openpilot)
* [Running in a car](#running-in-a-car)
* [Running on PC](#running-on-pc)
* [Community and Contributing](#community-and-contributing)
* [User Data and comma Account](#user-data-and-comma-account)
* [Safety and Testing](#safety-and-testing)
* [Directory Structure](#directory-structure)
* [Licensing](#licensing)

---

What is openpilot?
------

[openpilot](http://github.com/commaai/openpilot) is an open source driver assistance system. Currently, openpilot performs the functions of Adaptive Cruise Control (ACC), Automated Lane Centering (ALC), Forward Collision Warning (FCW) and Lane Departure Warning (LDW) for a growing variety of [supported car makes, models and model years](docs/CARS.md). In addition, while openpilot is engaged, a camera based Driver Monitoring (DM) feature alerts distracted and asleep drivers. See more about [the vehicle integration](docs/INTEGRATION.md) and [limitations](docs/LIMITATIONS.md).

<table>
  <tr>
    <td><a href="https://www.youtube.com/watch?v=mgAbfr42oI8" title="YouTube" rel="noopener"><img src="https://i.imgur.com/kAtT6Ei.png"></a></td>
    <td><a href="https://www.youtube.com/watch?v=394rJKeh76k" title="YouTube" rel="noopener"><img src="https://i.imgur.com/lTt8cS2.png"></a></td>
    <td><a href="https://www.youtube.com/watch?v=1iNOc3cq8cs" title="YouTube" rel="noopener"><img src="https://i.imgur.com/ANnuSpe.png"></a></td>
    <td><a href="https://www.youtube.com/watch?v=Vr6NgrB-zHw" title="YouTube" rel="noopener"><img src="https://i.imgur.com/Qypanuq.png"></a></td>
  </tr>
  <tr>
    <td><a href="https://www.youtube.com/watch?v=Ug41KIKF0oo" title="YouTube" rel="noopener"><img src="https://i.imgur.com/3caZ7xM.png"></a></td>
    <td><a href="https://www.youtube.com/watch?v=NVR_CdG1FRg" title="YouTube" rel="noopener"><img src="https://i.imgur.com/bAZOwql.png"></a></td>
    <td><a href="https://www.youtube.com/watch?v=tkEvIdzdfUE" title="YouTube" rel="noopener"><img src="https://i.imgur.com/EFINEzG.png"></a></td>
    <td><a href="https://www.youtube.com/watch?v=_P-N1ewNne4" title="YouTube" rel="noopener"><img src="https://i.imgur.com/gAyAq22.png"></a></td>
  </tr>
</table>


Running in a car
------

To use openpilot in a car, you need four things
* This software. It's free and available right here.
* One of [the 140+ supported cars](docs/CARS.md). We support Honda, Toyota, Hyundai, Nissan, Kia, Chrysler, Lexus, Acura, Audi, VW, and more. If your car is not supported, but has adaptive cruise control and lane keeping assist, it's likely able to run openpilot.
* A supported device to run this software. This can be a [comma two](https://comma.ai/shop/products/two), [comma three](https://comma.ai/shop/products/three), or if you like to experiment, a [Ubuntu computer with webcams](https://github.com/commaai/openpilot/tree/master/tools/webcam).
* A way to connect to your car. With a comma two or three, you need only a [car harness](https://comma.ai/shop/products/car-harness). With an EON Gold or PC, you also need a [black panda](https://comma.ai/shop/products/panda).

We have detailed instructions for [how to install the device in a car](https://comma.ai/setup).

Running on PC
------

All of openpilot's services can run as normal on a PC, even without special hardware or a car. To develop or experiment with openpilot you can run openpilot on recorded or simulated data.

With openpilot's tools you can plot logs, replay drives and watch the full-res camera streams. See [the tools README](tools/README.md) for more information.

You can also run openpilot in simulation [with the CARLA simulator](tools/sim/README.md). This allows openpilot to drive around a virtual car on your Ubuntu machine. The whole setup should only take a few minutes, but does require a decent GPU.


Community and Contributing
------

openpilot is developed by [comma](https://comma.ai/) and by users like you. We welcome both pull requests and issues on [GitHub](http://github.com/commaai/openpilot). Bug fixes and new car ports are encouraged. Check out [the contributing docs](docs/CONTRIBUTING.md).

Documentation related to openpilot development can be found on [docs.comma.ai](https://docs.comma.ai). Information about running openpilot (e.g. FAQ, fingerprinting, troubleshooting, custom forks, community hardware) should go on the [wiki](https://github.com/commaai/openpilot/wiki).

You can add support for your car by following guides we have written for [Brand](https://blog.comma.ai/how-to-write-a-car-port-for-openpilot/) and [Model](https://blog.comma.ai/openpilot-port-guide-for-toyota-models/) ports. Generally, a car with adaptive cruise control and lane keep assist is a good candidate. [Join our Discord](https://discord.comma.ai) to discuss car ports: most car makes have a dedicated channel.

Want to get paid to work on openpilot? [comma is hiring](https://comma.ai/jobs/).

And [follow us on Twitter](https://twitter.com/comma_ai).

User Data and comma Account
------

By default, openpilot uploads the driving data to our servers. You can also access your data through [comma connect](https://connect.comma.ai/). We use your data to train better models and improve openpilot for everyone.

openpilot is open source software: the user is free to disable data collection if they wish to do so.

openpilot logs the road facing cameras, CAN, GPS, IMU, magnetometer, thermal sensors, crashes, and operating system logs.
The driver facing camera is only logged if you explicitly opt-in in settings. The microphone is not recorded.

By using openpilot, you agree to [our Privacy Policy](https://comma.ai/privacy). You understand that use of this software or its related services will generate certain types of user data, which may be logged and stored at the sole discretion of comma. By accepting this agreement, you grant an irrevocable, perpetual, worldwide right to comma for the use of this data.

Safety and Testing
----

* openpilot observes ISO26262 guidelines, see [SAFETY.md](docs/SAFETY.md) for more details.
* openpilot has software in the loop [tests](.github/workflows/selfdrive_tests.yaml) that run on every commit.
* The code enforcing the safety model lives in panda and is written in C, see [code rigor](https://github.com/commaai/panda#code-rigor) for more details.
* panda has software in the loop [safety tests](https://github.com/commaai/panda/tree/master/tests/safety).
* Internally, we have a hardware in the loop Jenkins test suite that builds and unit tests the various processes.
* panda has additional hardware in the loop [tests](https://github.com/commaai/panda/blob/master/Jenkinsfile).
* We run the latest openpilot in a testing closet containing 10 comma devices continuously replaying routes.

Directory Structure
------
    .
    ├── cereal              # The messaging spec and libs used for all logs
    ├── common              # Library like functionality we've developed here
    ├── docs                # Documentation
    ├── opendbc             # Files showing how to interpret data from cars
    ├── panda               # Code used to communicate on CAN
    ├── third_party         # External libraries
    ├── pyextra             # Extra python packages
    └── selfdrive           # Code needed to drive the car
        ├── assets          # Fonts, images, and sounds for UI
        ├── athena          # Allows communication with the app
        ├── boardd          # Daemon to talk to the board
        ├── camerad         # Driver to capture images from the camera sensors
        ├── car             # Car specific code to read states and control actuators
        ├── common          # Shared C/C++ code for the daemons
        ├── controls        # Planning and controls
        ├── debug           # Tools to help you debug and do car ports
        ├── locationd       # Precise localization and vehicle parameter estimation
        ├── logcatd         # Android logcat as a service
        ├── loggerd         # Logger and uploader of car data
        ├── modeld          # Driving and monitoring model runners
        ├── proclogd        # Logs information from proc
        ├── sensord         # IMU interface code
        ├── test            # Unit tests, system tests, and a car simulator
        └── ui              # The UI

Licensing
------

openpilot is released under the MIT license. Some parts of the software are released under other licenses as specified.

Any user of this software shall indemnify and hold harmless Comma.ai, Inc. and its directors, officers, employees, agents, stockholders, affiliates, subcontractors and customers from and against all allegations, claims, actions, suits, demands, damages, liabilities, obligations, losses, settlements, judgments, costs and expenses (including without limitation attorneys’ fees and costs) which arise out of, relate to or result from any use of this software by user.

**THIS IS ALPHA QUALITY SOFTWARE FOR RESEARCH PURPOSES ONLY. THIS IS NOT A PRODUCT.
YOU ARE RESPONSIBLE FOR COMPLYING WITH LOCAL LAWS AND REGULATIONS.
NO WARRANTY EXPRESSED OR IMPLIED.**

---

<img src="https://d1qb2nb5cznatu.cloudfront.net/startups/i/1061157-bc7e9bf3b246ece7322e6ffe653f6af8-medium_jpg.jpg?buster=1458363130" width="75"></img> <img src="https://cdn-images-1.medium.com/max/1600/1*C87EjxGeMPrkTuVRVWVg4w.png" width="225"></img>

[![openpilot tests](https://github.com/commaai/openpilot/workflows/openpilot%20tests/badge.svg?event=push)](https://github.com/commaai/openpilot/actions)
[![Total alerts](https://img.shields.io/lgtm/alerts/g/commaai/openpilot.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/commaai/openpilot/alerts/)
[![Language grade: Python](https://img.shields.io/lgtm/grade/python/g/commaai/openpilot.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/commaai/openpilot/context:python)
[![Language grade: C/C++](https://img.shields.io/lgtm/grade/cpp/g/commaai/openpilot.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/commaai/openpilot/context:cpp)
[![codecov](https://codecov.io/gh/commaai/openpilot/branch/master/graph/badge.svg)](https://codecov.io/gh/commaai/openpilot)

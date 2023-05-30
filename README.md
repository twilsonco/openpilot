### Which fork do you want?
* C2 users: you might be stuck on the `tw-c2` branch if this branch (`tw-main`) doesn't work.
* C3 users: depending on how new your C3 is, support might have been lost with my "tw" branches, so you'll want to use my SunnyPilot fork `sunnu-dev` (will update as this changes).
* `tw-main` is a stable branch based on a 1-ish year old build
* `tw-staging` receives more updates and newer features
* `tw-dev` is modified often and is for breaking; use at your own risk
* I'll list relevent branches here as they're made.
* What's the difference?
    * "My fork" is based on OpenPilot v0.8.9 from Winter 2021
        * Uses the last of the "medium models" for planning the driving path, which is rock solid dependable, unlike the newer models that cut curners bad
    * `sunny-dev` is based on SunnyPilot, a popular fork with some great features, and it stays up to date with upstream OpenPilot software.
        * It provides support for newer C3s
        * *Please (at least partially) direct your support for the SunnyPilot fork to SunnyPilot directly.*

### Join us on Discord!
* Find us in the "[twilsonco Volt GM fork](https://discord.com/channels/469524606043160576/884811574773157949)" thread in the [GM channel](https://discord.com/channels/469524606043160576/524611823090008065) of the [Comma.ai Discord server](https://discord.comma.ai).
* Also find help in the "[twilsonco fork](https://discord.com/channels/771493367246094347/954439641023524884)" thread in the [GM channel](https://discord.com/channels/771493367246094347/772164144643702855) of the [OpenPilot Community Discord server](https://discord.gg/XnYqf6ZB9A)

# Table of Contents
------
- [Table of Contents](#table-of-contents)
    - [Infographics](#infographics)
    - [Appreciate My Work?](#appreciate-my-work)
- [Fork Details](#fork-details)
  - [Current Fork Features](#current-fork-features---optional-via-toggle)
  - [Planned Fork Features](#planned-fork-features-in-no-particular-order)
  - [Supported Hardware](#supported-hardware)
  - [Installation Instructions](#installation-instructions)
  - [Automatic Updates](#automatic-updates)
  - [Tuning](#tuning)
  - [Lateral Tuning](#lateral-tuning)
- [Commaai Table of Contents](#commaai-table-of-contents)
  - [What is openpilot?](#what-is-openpilot)
  - [Running in a car](#running-in-a-car)
  - [Running on PC](#running-on-pc)
  - [Community and Contributing](#community-and-contributing)
  - [User Data and comma Account](#user-data-and-comma-account)
  - [Safety and Testing](#safety-and-testing)
  - [Directory Structure](#directory-structure)
  - [Licensing](#licensing)

### Infographics
------
![](https://github.com/twilsonco/openpilot/blob/tw-0.8.9-dev/fork_image_touchcontrols.png?raw=true)
![](https://github.com/twilsonco/openpilot/blob/tw-0.8.9-dev/fork_image_coasting.png?raw=true)
![](https://github.com/twilsonco/openpilot/blob/tw-0.8.9-dev/fork_image_lane_position.png?raw=true)
![](https://github.com/twilsonco/openpilot/blob/tw-0.8.9-dev/fork_image_metrics.png?raw=true)
![](https://github.com/twilsonco/openpilot/blob/tw-0.8.9-dev/fork_image_misc.png?raw=true)

### Videos
------
<table>
  <tr>
    <td><a href="https://youtu.be/9hXuDye1HT0" title="YouTube" rel="noopener"><img src="https://i.imgur.com/kPEPNvB.png"></a></td>
    <td><a href="https://youtu.be/NuFGm8Jhlp0" title="YouTube" rel="noopener"><img src="https://i.imgur.com/CCXiCJi.png"></a></td>
    <td><a href="https://youtu.be/8i4mUsxL-jk" title="YouTube" rel="noopener"><img src="https://i.imgur.com/mhZfclR.png"></a></td>
  </tr>
  <tr>
    <td><a href="https://youtu.be/Ssx_1S5sJXw" title="YouTube" rel="noopener"><img src="https://i.imgur.com/rAVCth7.png"></a></td>
    <td><a href="https://youtu.be/O7F07dESH2g" title="YouTube" rel="noopener"><img src="https://i.imgur.com/1Xkggr4.png"></a></td>
    <td><a href="https://youtu.be/sir1iIg8InI" title="YouTube" rel="noopener"><img src="https://i.imgur.com/nVzVIpX.png"></a></td>
  </tr>
</table>

### Appreciate My Work?
------

Check out the [commit history](https://github.com/twilsonco/openpilot/commits/tw-0.8.9-dev) of `tw-0.8.9-dev` to see the ongoing work you can support.

**[Patreon (recurring contributions)](https://www.patreon.com/twilsonco)**

**[PayPal (one-time contribution)](https://paypal.me/twilsonco)**

# Fork Details
------

> This fork exists to improve OP performance and convenience for GM cars, specifically the Chevy Volt.
> I started because the *kegman* fork wouldn't run on Comma Three, and the result is the best Volt driving experience in the known universe.

> While most of the features in this fork are my original implementations, none of it would have been possible without the work of others in the community, which is often my starting point or inspiration, and nearly always a reference.
> I reference code/concepts to their original authors to the best of my knowledge.
> Feel free to let me know if I have missed or mistaken a reference.

> **Pleae provide any positive/negative feedback to me, twilsonco#9281, on the [comma](https://discord.comma.ai)/[openpilot](https://discord.gg/SgbgsdGWu5)/[retropilot](https://discord.gg/retropilot-660951518014341124) Discord servers, especially if you'd like your issues addressed.**


### Supported Hardware
------

This fork is developed and used on a Comma Three in a 2018 Chevy Volt, and is also *known* to work on Comma Two and Comma Zero, and in 2017 Volt, 2018 Acadia, and supported Escalades.

### Installation Instructions
------

#### Easy: using comma's [openpilot-installer-generator](https://github.com/sshane/openpilot-installer-generator)

For fewer, but better tested updates, use `installer.comma.ai/twilsonco`,
or to ride the bleeding edge, try the staging branch where new features are tested before they go to regular users: `installer.comma.ai/twilsonco/tw-0.8.9-staging`
(*Be extra diligent and attentive when using the staging branch; it is considered experimental moreso than the regular branch!*)

Or using Shane's installer with [these instructions](https://github.com/sshane/openpilot-installer-generator#usage) and the following url:
`https://smiskol.com/fork/twilsonco`



#### Less easy

With a stock installation of OpenPilot confirmed working, SSH into device and run the following:

`cd /data;mv openpilot openpilot_stock;git clone --recurse-submodules https://github.com/twilsonco/openpilot`

Then, `sudo reboot`

### Automatic Updates
------

This fork will auto-update while your device has internet access, and changes are automatically applied the next time the device restarts.
If you're device stays connected to your car all the time, you'll be presented with a message to update when your car is off.

#### Current Fork Features [✅ = optional via toggle]:
-----

**Running on move-fast fork of openpilot v0.8.9**, which adds:

* **Vision and/or map-based slowing down for curves**
* **Map-based automatic changing of speed limit** (with optional offset)
  * **Map-based slowing for curves and automatic setting of speed requires a data connection**
  * You can subscribe to [comma Prime](https://comma.ai/prime), which is a great service and is totally worth the cost, or you can use your own hotspot and enable the "disable onroad uploads" toggle (see below) to only use data for these features
* ~~Hands on wheel monitoring~~ *In stock*
* ~~Disable disengage when gas pressed~~ *Now in stock*

**Additional fork features:**

- [x] Running openpilot 0.8.12 lateral (last of the medium-models) and 0.8.10 driver monitoring models
- [x] [Comma3] AGNOS4 OS
- [x] [✅] [Volt/Acadia] **Alternate lateral** (steering) tunes using the new "torque" controller
- [Volt] Much improved steering control over stock (working on upstreaming)
- [x] [✅] **MADS (Modified Assistive Driving Safety)**. Three independent, always-on safety features. Enable the MADS autosteer toggle in order to access the other features as well. If enabled, the max speed indicator is replaced by the MADS icon. *Reminder: You can use the cruise main button to cancel out all openpilot controls*. 
    * ***Autosteer*** (a.k.a. SunnyHaibin's MADS) comes on as soon as you start driving the car, before initially engaging, and even while brakes are pressed. *Toggle autosteer with the lane keep assist button on your steering wheel*. Optional toggle in settings to pause steering at low speeds when blinker is on and you're decelerating. When autosteer is enabled, the steering wheel icon will be colored ("disengaged" color when disabled).
    * ***Lead braking*** will be applied automatically, so OpenPilot will smoothly slow for other cars *if neither the gas or brake pedal are pressed in the slightest*. Volt regen-on-demand paddle can be used and lead braking will still be applied. Use either pedal to override lead braking. *Toggle lead braking with the ACC follow distance button on your steering wheel*. When lead braking is enabled, an additional white ring is shown around the MADS icon.
    * ***One-pedal driving*** [Volt only], where OpenPilot applies light braking when you're in L-mode and not pressing the gas, so you can come to a stop without using brakes. The feeling is as if L-mode regen braking brought you to a stop. Hold the regen-on-demand paddle for additional braking. *Toggle one-pedal driving with a double-press of the regen paddle*. You can also toggle one-pedal driving by tapping the MADS icon. When one-pedal driving is enabled, the MADS icon turns to the "engaged" color, or the "disengaged" color when disabled. (application of friction brakes originally suggested by cybertronicify — 10/06/2021)
    * ***One-pedal one-time stop***. Hold the Volt regen paddle to below 5mph and one pedal mode will engage temporarily to stop you once, then turn off when you resume driving.
- [x] [Volt 2017] **Auto-resume** behind stopped lead car as they pull away; a.k.a. stop and go (ported from kegman)
- [x] [Volt 2018] **Auto-creep** behind stopped lead car as they pull away (tap gas or resume button to resume)
- [x] [GM] [✅] **AutoHold** (autohold brakes when stopped; ported from kegman)
- [x] [GM] Adjustable follow "mode" using ACC distance button (ported from kegman, but smoother follow profiles)
- [x] [✅] **Extended radar capabilities**
    * Brake for car in front of lead, avoiding pile-ups and able to brake when lead changes lanes right in front of a column of stopped cars
    * Indicate adjacent oncoming/ongoing traffic
    * Indicate all tracked cars, including those in front of the lead, and print all speeds
    * Tap at screen bottom-center (middle of path) to switch where speeds are printed
- [x] [✅] **Longer range lead detection**, indicated by blue dot over lead indicator
    * 10% more range using radar, or
    * up to 40% more range using the Volt LKA camera's VOACC capabilities, which have been incorporated into openpilot
- [x] [✅] **Print lead details**: lead distance (length and time) and speed (absolute and relative) are printed to the sides of the lead indicator
- [x] [✅] **Adjustable lane position** using onscreen buttons
    * Tap buttons to change lane position for 1/3 mile; double-tap to change for 10 miles
    * Tap again to go back to center position
- [x] [✅] **Automatic lane position**: if adjacent traffic on one side and not the other, assume opposite lane position
    * To activate, enable adjustable lane position, then while onroad, tap left then right (or right then left) lane position button within 2s.
    * Tap either button to deactivate
    * Requires clear lanelines, and a mostly-straight road
    * [✅] **Automatic automatic lane position** (you heard me) turns on automatic lane positioning automatically when at 22mph+ with clear lanelines and adjacent traffic is present
- [x] [GM] [✅] **Dynamic follow mode**: Defensive driving triggered by a cut-in or by adjacent ongoing traffic's average following distance (the less chill they drive, the more defensive OpenPilot gets)
- [x] [GM] **Toggle steering while openpilot is engaged with the LKAS button** (wheel color changes to indicate disengagement)
- [x] [✅] [**Dynamic Lane Profile**](https://github.com/sunnyhaibin/openpilot#dynamic-lane-profile-dlp) (DLP); *tap button while driving to switch between auto/laneless/lane-only. must enable "Disable use of lanelines" for button to appear* (ported from sunnyhaibin)
- [x] [✅] **Normal/sport/eco acceleration modes** [cycle with on-screen button] (adapted from kegman implementation)
    * Eco mode is great for
      * the environment (you pig),
      * not over-accelerating behind a jumpy lead in traffic, and 
      * softer acceleration when exiting curves (if curve braking is enabled).
- [x] [✅] **Reverse speed adjustment**; tap/hold for 5mph/1mph increments (ported from Spector56; different from that of newer stock OP)
- [x] [✅] **3mph cruise speed offset**: speed will be 23/28/33/38/etc.
- [x] [✅] **Alternate color scheme**
- [x] [✅] **Alternate sound effect set** (inspired by sunnyhaibin implementation of mute sounds)
- [x] [✅] **Mute engage and disengage sounds** (inspired by sunnyhaibin implementation)
- [x] **Brightness control**: Tap driver monitoring icon to cycle stock/medium/low brightness (most recently suggested by veryluckyguy)
    * Screen will temporarily undim for warning/critical alerts
- [x] [✅] **Disable onroad uploads**: for use with data-limited wifi hotspots. Reduces data use from 400MB/hour or 22MB/mile (based on 30 minute low-speed trip) down to 25MB/hour or 0.4MB/mile (based on 5 hour trip at 84mph; i.e. not a perfect comparison to the other trip)
    * Don't bother if you subscribe to [comma Prime](https://comma.ai/prime), which has unlimited data, and a host of other benefits! Don't delay; subscribe today!!
    * iPhone users can use [this shortcut](https://www.icloud.com/shortcuts/7f3c7e98f95d4f85a9bad939aa069fcd) to instantly open the personal hotspot page in the Settings app in order to enable personal hotspot for your comma device to connect.
      * Combined with an Automation to run the shortcut when you enter CarPlay, or when you connect to your car's Bluetooth, this can be a pretty convenience setup.
    * Android users could try the [Hot Spot Starter](https://play.google.com/store/apps/details?id=de.thjunge11.autohotspot) app, but I can't recommend it as I haven't tried it, so maybe [look for something else if it doesn't work](https://forum.xda-developers.com/t/enable-hotspot-automatically-when-i-enter-the-car.3915107/)
- [x] [✅] **Over-speed coasting**: OP will still brake behind a lead car and to slow down for curves, but will not apply engine/regen/friction brakes in order to keep the set speed (by user or map speed limit)
    * Toggle coasting while driving 
      * in Volt using gear shifter: D for coasting, L for regen (thanks to jshuler for discovering the CAN message of extra gear shifter values)
      * in other cars by tapping the max speed indicator
    * A "+" after the max speed indicates that coasting is enabled
    * *Can be a bit rough on the brakes when following downhill over set speed; recommend to disable if uncomfortable when constantly following downhill*
    * (Inspired by the implementation in sunnyhaibin's fork)
    * [✅] Additional toggle to apply a **reasonable upper speed limit for coasting**, 15% over set speed at 20mph down to 12% over at 80mph
    * [Volt] [✅] **Coasting D/L control**: Control over-speed coasting when cruising with the D/L gear shifter position. Coast in D; maintain set speed exactly in L.
- [x] [✅] **Auto "nudgeless" lane change**: OP will start lane change automatically in direction of blinker after blinker on for 1.5s
    * Won't auto lane change if adjacent lane is not detected, or if oncoming traffic in adjacent lanes
    * Alert will tell you when/why auto lane change is blocked
    * Lane change alerts warn you if changing lanes when adjacent lane not detected or when oncoming traffic in adjacent lane
- [x] [✅] *Local weather information* using OpenWeatherMap.org. Tap the weather icon to switch between simple/full display modes. You can use the provided API key or enter your own into `/data/OpenWeatherMap_apiKey.txt`
- [x] [✅] *Weather-based safety* using OpenWeatherMap.org. Openpilot will automatically use far follow, mild acceleration, and less intense braking for curves when in inclement weather. You can use the provided API key or enter your own into `/data/OpenWeatherMap_apiKey.txt`
- [x] [✅] **Power meter** (or brake indicator) shows level of regenerative/engine and friction braking
    * Power meter shows lines for losses due to drag, rolling resistance, and elevation gain (in that order from bottom to top on the meter), and differentiates between regen and brake power
    * Tap brake indicator or power meter to switch between indicator, meter, or meter + power readout
    * Tap power readout to switch units from kw to hp
- [x] **Customizable, dynamic vehicle/device metrics** (adapted from kegman)
    * To use:
        * Tap the current speed on the openpilot display to cycle the number of metrics
        * Tap any metric to cycle its content (sorry for all the god-forsaken tapping, a better metric display with vehicle, following, position, and device widgets is a WIP)
    * Metrics (79 to choose from):
        * Device info: CPU temperature (°C and °F), CPU percent, CPU temp + percent (°C and °F), memory temperature (°C and °F), memory used, free storage, ambient temperature (°C and °F), fanspeed (as percent of max), GPS accuracy (and number of satelites), altitude
        * Vehicle info: Engine RPM, engine coolant temperature (°C and °F), engine RPM + coolant temperature (°C and °F), steering torque, steering angle, desired steering angle, vehicle acceleration, vehicle jerk, lane width, distance from lane center, percent grade of current road (one based on GPS, one based on device accelerometer), Volt high-voltage battery wattage [kW], voltage [V], current [A], voltage+wattage, drag force/power, brake force/power, regen force/power, electric motor force/power, gas engine power, total acceleration force/power, EV efficiency (instantaneous, 5 mile average, total), EV power consumption (instant/5mi/total), EV instantaneous efficiency/consumption (depending on sign), EV drivetrain efficiency (amount of power making it to the tires),  
        * Lead/traffic info: follow distance level, lead distance [length], desired lead distance [length], lead distance [time], desired lead distance [time], follow distance and acceleration mpc costs [in units of the stock OP costs; i.e. 2.5 means 2.5× the stock OP value], relative lead velocity, absolute lead velocity, number of cars (total, ongoing, oncoming, stopped, ongoing in adjacent lanes), minimum follow distance of ongoing cars in adjacent lanes
        * Engagement and interaction statistics
          * Time driving (drive or trip)
          * Time openpilot fully engaged (drive or trip)
          * Percent time openpilot fully engaged (drive or trip)
          * Distance driven (drive or trip)
          * Distance openpilot fully engaged (since last engage, drive, or trip)
          * Percent distance openpilot fully engaged (drive or trip)
          * Time since last disengagement (also for interaction, intervention, distraction)
          * Distance since last disengagement (also for interaction, intervention, distraction)
          * Count of disengagements (drive or trip; also for interactions, interventions, distractions)
          * Time per disengagement (drive or trip; also for interactions, interventions, distractions)
          * Distance per disengagement (drive or trip; also for interactions, interventions, distractions)
        * ...and others
- [x] [GM] panda-based GM steering fault fix (thanks jshuler)
- [x] Remember last follow mode (ported from kegman)
- [x] Grey/White panda support
- [x] **opParams**: command-line editor to configure and tune OpenPilot instantaneously from the passenger seat. You can tune run `./opparams.py` immediately after [connecting over SSH](https://docs.comma.ai/tools/ssh/README.html#) and configure the following:
    * acceleration profiles
    * traffic following profiles
    * camera (lane) offset
    * offroad shutdown timer and car 12v cutoff
    * over-speed coasting
    * assisted (and nudgeless) lane change
    * MADS (autosteer, one-pedal mode)
    * curve braking
    * automatic speed limits
    * adjustable/automatic lane positioning
    * traffic detection
    * lateral (six of them!) and longitudinal controllers
      * *can select and tune pid, torque, indi, lqr, torque indi, or torque lqr controllers*
    * select UI metrics (no more tapping! you can still tap though)
    * enter your own OpenWeatherMap api key (instead of freeloading off mine!)
    * *perform a reboot-less restart on your Comma Three by running `./opparams.py -r`*
    * *Started from @Shane's opParams implementation and added TONS of features. Thanks Shane!*

#### Planned Fork Features (in no particular order):
-----

- [ ] Chevy Bolt support
- [ ] Record screen button
- [ ] Redo UI metrics as themed "widgets" instead that can be activated independently and stack against the right (and left if necessary) side of the screen
  * Follow widget: a colored vertical bar indicating follow distance with lines indicating the actual and desired (length/time) follow distances. Tap to include more info items like current distance cost
  * Openpilot widget: a similar vertical bar (or maybe something like a circular progress bar or a speedometer--looking thing) showing the gas/braking being requested by OP. Also include Driver monitoring info.
  * Car widget: Acceleration/jerk, tire pressures, low voltage battery info, ...
  * Geo widget: GPS signal/coords/#satellites, altitude, percent grade of current road, ...
  * Device widget: CPU/memory/temps/fans/...
  * EV widget: high voltage battery info similar to that shown in the LeafSpyPro app

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

- [What is openpilot?](#what-is-openpilot)
- [Running in a car](#running-in-a-car)
- [Running on PC](#running-on-pc)
- [Community and Contributing](#community-and-contributing)
- [User Data and comma Account](#user-data-and-comma-account)
- [Safety and Testing](#safety-and-testing)
- [Directory Structure](#directory-structure)
- [Licensing](#licensing)

---

What is openpilot?
------

[openpilot](http://github.com/commaai/openpilot) is an open source driver assistance system. Currently, openpilot performs the functions of Adaptive Cruise Control (ACC), Automated Lane Centering (ALC), Forward Collision Warning (FCW) and Lane Departure Warning (LDW) for a growing variety of [supported car makes, models and model years](docs/CARS.md). In addition, while openpilot is engaged, a camera based Driver Monitoring (DM) feature alerts distracted and asleep drivers. See more about [the vehicle integration](docs/INTEGRATION.md) and [limitations](docs/LIMITATIONS.md).


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

=======

<img src="https://d1qb2nb5cznatu.cloudfront.net/startups/i/1061157-bc7e9bf3b246ece7322e6ffe653f6af8-medium_jpg.jpg?buster=1458363130" width="75"></img> <img src="https://cdn-images-1.medium.com/max/1600/1*C87EjxGeMPrkTuVRVWVg4w.png" width="225"></img>

[![openpilot tests](https://github.com/commaai/openpilot/workflows/openpilot%20tests/badge.svg?event=push)](https://github.com/commaai/openpilot/actions)
[![Total alerts](https://img.shields.io/lgtm/alerts/g/commaai/openpilot.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/commaai/openpilot/alerts/)
[![Language grade: Python](https://img.shields.io/lgtm/grade/python/g/commaai/openpilot.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/commaai/openpilot/context:python)
[![Language grade: C/C++](https://img.shields.io/lgtm/grade/cpp/g/commaai/openpilot.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/commaai/openpilot/context:cpp)
[![codecov](https://codecov.io/gh/commaai/openpilot/branch/master/graph/badge.svg)](https://codecov.io/gh/commaai/openpilot)

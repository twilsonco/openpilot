![](https://i.imgur.com/jJi1R88.jpg)

Table of Contents
=======================

* [What is openpilot?](#what-is-openpilot)
* [What is FrogPilot?](#what-is-frogpilot)
* [Features](#features)
* [How to Install](#how-to-install)
* [Bug reports / Feature Requests](#bug-reports--feature-requests)
* [Discord](#discord)
* [Donations](#donations)
* [Credits](#credits)
* [Licensing](#licensing)

---

What is openpilot?
------

[openpilot](http://github.com/commaai/openpilot) is an open source driver assistance system. Currently, openpilot performs the functions of Adaptive Cruise Control (ACC), Automated Lane Centering (ALC), Forward Collision Warning (FCW), and Lane Departure Warning (LDW) for a growing variety of [supported car makes, models, and model years](docs/CARS.md). In addition, while openpilot is engaged, a camera-based Driver Monitoring (DM) feature alerts distracted and asleep drivers. See more about [the vehicle integration](docs/INTEGRATION.md) and [limitations](docs/LIMITATIONS.md).

<table>
  <tr>
    <td><a href="https://youtu.be/NmBfgOanCyk" title="Video By Greer Viau"><img src="https://i.imgur.com/1w8c6d2.jpg"></a></td>
    <td><a href="https://youtu.be/VHKyqZ7t8Gw" title="Video By Logan LeGrand"><img src="https://i.imgur.com/LnBucik.jpg"></a></td>
    <td><a href="https://youtu.be/VxiR4iyBruo" title="Video By Charlie Kim"><img src="https://i.imgur.com/4Qoy48c.jpg"></a></td>
    <td><a href="https://youtu.be/-IkImTe1NYE" title="Video By Aragon"><img src="https://i.imgur.com/04VNzPf.jpg"></a></td>
  </tr>
  <tr>
    <td><a href="https://youtu.be/iIUICQkdwFQ" title="Video By Logan LeGrand"><img src="https://i.imgur.com/b1LHQTy.jpg"></a></td>
    <td><a href="https://youtu.be/XOsa0FsVIsg" title="Video By PinoyDrives"><img src="https://i.imgur.com/6FG0Bd8.jpg"></a></td>
    <td><a href="https://youtu.be/bCwcJ98R_Xw" title="Video By JS"><img src="https://i.imgur.com/zO18CbW.jpg"></a></td>
    <td><a href="https://youtu.be/BQ0tF3MTyyc" title="Video By Tsai-Fi"><img src="https://i.imgur.com/eZzelq3.jpg"></a></td>
  </tr>
</table>


What is FrogPilot? 🐸
------

FrogPilot is my custom "Frog Themed" fork of openpilot that has been tailored to improve the driving experience for my 2019 Lexus ES 350. I resync with the latest version of master quite frequently, so this fork is always up to date. I also strive to make every commit I make easy to read and easily cherry-pickable, so feel free to use any of my features in your own personal forks in any way that you see fit!

------

FrogPilot was last updated on:

**August 27th, 2023**

Features
------

FrogPilot offers a wide range of customizable features that can be easily toggled on or off to suit your preferences. Whether you want a completely stock openpilot experience or want to add some fun and personal touches, FrogPilot has you covered! Some of the features include:

- Frog theme!
  - Frog/green color scheme
  - Frog icons
  - Frog sounds (with a bonus goat sound effect)
  - Frog turn signals that "hop" along the bottom of your screen
- Adjustable driving personality profiles via the "Distance" button on the steering wheel (Toyota/Lexus only)
  - Ability to customize the profiles to your liking as well
  - Other makes can use the Onroad UI button
- Allow the device to be offline indefinitely
- Always On Lateral / No disengage lateral on brake
- Conditional Experimental Mode
  - Automatically enables "Experimental Mode":
    - When a curve is detected
    - When approaching a slower lead vehicle
    - When driving below a set speed
    - When either turn signal is activated below 55mph to assist with turns
    - When stop lights or stop signs are detected
- Custom steering wheel icons. Want to add your own? Post it in the "feature-request" channel on the FrogPilot Discord!
- Customize the road UI
  - Blind spot path to indicate when a vehicle is in your blind spot
  - Increase or decrease the lane line width
  - Increase or decrease the path width
  - Increase or decrease the road edges width
  - Path edges that represent driving statuses:
    - Blue - Navigation active
    - Light Blue - "Always On Lateral" active
    - Green - Default with "FrogPilot Colors" toggled on
    - Light green - Default with "FrogPilot Colors" toggled off
    - Orange - Experimental Mode activated
    - Yellow - Conditional Experimental overriden
  - "Unlimited" road UI that extends out as far as the model can see
- Customize the lateral and longitudinal behaviors
  - Lateral:
    - [Pfeiferj's distance based curvature adjustment](https://github.com/commaai/openpilot/pull/28118) for smoother handling of curves
    - [Twilsonco's NNFF](https://github.com/twilsonco/openpilot) that enhances the steering torque for smoother lateral control
  - Longitudinal:
    - Aggressive acceleration when following a lead from a stop
    - Increased stopping distance for a more comfortable stop behind a lead vehicle
    - Smoother braking to approach lead vehicles more naturally
    - Sport and Eco acceleration profiles
    - Toyota / TSS2 Tune for smoother driving dynamics
- Developer UI displaying various driving statistics and device states
- Device shuts down after being offroad for a set amount of time instead of 30 hours to help prevent battery drain
- Disable the wide camera while in Experimental Mode. This effect is purely cosmetic
- Easy Panda flashing via a "Flash Panda" button located within the "Device" menu
- Have the sidebar show by default to monitor your device temperature and connectivity with ease
- Increase the "MAX" speed by 5 instead of 1 on short presses (Toyota/Lexus only)
- Navigate on openpilot without a comma prime subscription
- Nudgeless lane changes
  - Lane detection to prevent lane changes into curbs or going off-road
  - Optional delay setting
  - Optional one lane change per signal activation
- Numerical temperature gauge to replace the "GOOD", "OK", and "HIGH" temperature statuses
  - Tap the gauge to switch between "Celsius" and "Fahrenheit"
- On screen compass that rotates according to the direction you're facing
- Prebuilt functionality for a faster boot
- Set the screen brightness to your liking (or even completely off while onroad)
- Silent Mode for a quiet openpilot experience
- Steering wheel in the top right corner of the onroad UI rotates alongside your car's steering wheel
- Tap the speed indicator to remove it from the screen
- Toggle Experimental Mode via the "Lane Departure Alert" button on your steering wheel (Toyota/Lexus only)
  - Other makes can simply double tap the screen while on-road
- Use turn desires when below the minimum lane change speed for more precise turns

How to Install
------

Easiest way to install FrogPilot is via this URL at the installation screen:

```
https://installer.comma.ai/FrogAi/FrogPilot
```
Be sure to capitalize the "F" and "P" in "FrogPilot" otherwise the installation will fail.

DO NOT install the "FrogPilot-Development" branch. I'm constantly breaking things on there so unless you don't want to use openpilot, NEVER install it!

![](https://i.imgur.com/wxKp3JI.png)

Bug reports / Feature Requests
------

If you encounter any issues or bugs while using FrogPilot, or if you have any suggestions for new features or improvements, please don't hesitate to reach out to me. I'm always looking for ways to improve the fork and provide a better experience for everyone!

To report a bug or request a new feature, feel free to make a post in the respective channel on the FrogPilot Discord. Provide as much detail as possible about the issue you're experiencing or the feature you'd like to see added. Photos, videos, log files, or other relevant information are very helpful!

I will do my best to respond to bug reports and feature requests in a timely manner, but please understand that I may not be able to address every request immediately. Your feedback and suggestions are valuable, and I appreciate your help in making FrogPilot the best it can be!

As for feature requests, these are my guidelines:

- Can I test it on my 2019 Lexus ES or are you up for testing it?
- How maintainable is it? Or will it frequently break with future openpilot updates?
- Is it not currently being developed by comma themselves? (i.e. Navigation)
- Will I personally use it or is it very niche?

Discord
------

[Join the FrogPilot Community Discord for easy access to updates, bug reporting, feature requests, future planned updates, and other FrogPilot related discussions!](https://l.linklyhq.com/l/1t3Il)

Donations
------

I DO NOT accept donations! So if anyone is claiming to be me or to be a part of FrogPilot and is asking for any type of financial compensation, IT IS A SCAM!

I work on FrogPilot on my own and is purely a passion project to refine my skills and to help improve openpilot for the community. I do not and will not ever expect any type of financial exchange for my work. The only thing I’ll ever ask for in return is constructive feedback!

Credits
------

* [AlexandreSato](https://github.com/AlexandreSato/openpilot)
* [Aragon7777](https://github.com/Aragon7777/openpilot)
* [Crwusiz](https://github.com/crwusiz/openpilot)
* [DragonPilot](https://github.com/dragonpilot-community/dragonpilot)
* [KRKeegan](https://github.com/krkeegan/openpilot)
* [Move-Fast](https://github.com/move-fast/openpilot)
* [Pfeiferj](https://github.com/pfeiferj/openpilot)
* [Sunnyhaibin](https://github.com/sunnyhaibin/sunnypilot)
* [Twilsonco](https://github.com/twilsonco/openpilot)

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
[![codecov](https://codecov.io/gh/commaai/openpilot/branch/master/graph/badge.svg)](https://codecov.io/gh/commaai/openpilot)

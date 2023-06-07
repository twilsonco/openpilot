Version tw-0.8.12-17 (2023-4-30--6-06)
========================
  * IMPROVED: NNFF improvements
  * NEW Add Acadia and LaCrosse NNFF for torque and lat-pid controllers
  * IMPROVED: UI path gets a face lift
  * NEW: Print time to pass (seconds) for closest adjacent ongoing lead (if show all car speeds is enabled in toggles)
  * IMPROVED: Vision turn controller fixes and improvements

Version tw-0.8.12-16 (2023-4-7,9)
========================
  * Improved "torque" lateral control
    * Validate imported NN FF models against precomputed test values
    * Volt neural network feedforward (optional with toggle) (updated 4/9)
    * Volt 3-part composite fit feedforward with lateral accel, lateral jerk, and lateral gravitaional accel components
    * Revamped AZNPIDF lateral controller for more robust corrections and better centering on straights
    * Error downscalling in curves to allow feedforward to do most of the work, resulting in smoother steering wheel motion.
    * *Additional error downscaling after driver applied steer torque to avoid overcorrecting when autosteer comes back online*
  * Weather: full view shows imminent sunrise/sunset
  * OPParams: add toggle to reset all opparams to defaults on next start
  * Volt: disable autohold + one pedal mode when park assist is active.

Version tw-0.8.12-15 (2023-🥧+1)
========================
  * **support for newer C3s** (panda fan stall detection and internal fan controller added)
  * **support for Buick Lacrosse**
  * **support for no-acc volts** (see ed5e081; thanks Eric Brown!)
  * add toggle to disable openpilot updates
  * improved: controls:
    * Volt:
      * updated custom torque controller lateral acceleration feedforward function
      * replace "friction" with **custom lateral jerk feedforward** function
        * ~~"friction" in gm/interface.py or opparams is now a linear coefficient to cap the lateral jerk FF value when desired lateral jerk ≈ 0. For cars without custom lateral jerk FF (everything but Volt), friction specifies the max amount of "friction" torque sent.~~ This is still possible by setting the value of friction for Volt to be ≥ 0.0, but it should be left at -1.0 to disable this linear cap.
        * Now future planned lateral jerk is used as a check on the amount of lateral jerk feedforward. This change removes the need for the linear cap.
      * lower steer actuator delay 
      * revert use of future curvature in torque controller "low speed factor" as it was neutering the ability to correct steer angle on straights at very low speeds
    * Acadia:
      * Updated torque controller feedforward
    * revert PID derivative gain calculation to use error rate (instead of output)
    * replace PID "integrator" (ema really) with explicit trapezoidal integrator with controllable period
      * lat/long controllers use 1.5s integral period
      * other controllers (i.e. fanspeed) use a 2s period, selected because it reproduced the values of the old integrator
    * any amount of gas "overrides" the long controller, preventing integral windup 
    * positive acceleration smoothing:
      * starts at the current accel, for all the smoothing and none of the lag
      * only smooths at low speeds, for no delay at higher speeds
    * removed post-resume negative acceleration smoothing; no longer required
    * PID "autotuned" parameters now normalized based on speed, for more error response at low speed
  * improved: include lead lateral velocity in lead velocity calculation, only at low speed, to prevent op from slowing down as much for leads turning right
  * improved: automatic lane position:
    * revert to center position when opposite blinker is on (e.g. if in left position and right blinker is on, go back to center, but not if left blinker is on)
  * improved: opparams
    * see history for each param when accessed (full history in /data/community)
    * add default param resetting mechanism so that improved defaults can be pushed to users
    * indicate which parameters are changed from default
    * show onscreen alert when live opparam is changed
    * lots of cleanup and fingerpainting
    * many new parameters
  * UI improvements
    * Hide current speed (toggle; can still tap where speed would be to cycle UI metric layouts)
    * UI metrics:
      * remove extra lines from power meter (they indicated drag/rolling/gravity power losses but it was incoherent to look at)
      * second column hides when map is open
      * some formatting improvements
      * fix placement of metrics when in 2x4 layout
      * new metrics:
        * kinetic energy
        * time/distance to stop from current speed
        * "minimum" time/distance to stop based on 7m/s^2 deceleration rate (I looked it up somewhere)
    * curve braking icon hides for alerts
    * add parked timer that shows after 80 seconds (configure in opparams)
  * ...many other small things (check the commit history)

Version tw-0.8.12-14_tws (2023-01-17)
========================
  * one-pedal mode regen paddle integration
  * alert shown when each MADS feature is toggled
  * stopped timer only increments when in gear
  * use deadzone for current speed to better match vehicle cluster speed
    * opParams MISC_cluster_speed params to disable/adjust

Version tw-0.8.12-13_tws (2023-01-02)
========================
  * opParams: command-line editor to configure OpenPilot over SSH that lets you tune and change options instantaneously while driving. You can tune run `./opparams.py` immediately after connecting over SSH (https://docs.comma.ai/tools/ssh/README.html#) and configure the following:
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
    * enter your own OpenWeatherMap api key (no more txt file with key)
    * *perform a reboot-less restart on your Comma Three by running `./opparams.py -r`*
    * *Started from @Shane's opParams implementation, made the read process for live tuning non-blocking so it has zero impact on performance, and added TONS of features*
  * **Two new lateral (steering) controllers**, torque INDI and torque LQR. They both need lots of tuning work, especially torque LQR. Feel free to give tuning torque INDI a shot! And torque LQR if you don't mind reading up on how LQR controllers work!
  * Weather polish
  * One-pedal mode improvements
  * Fixes to post-MADS autohold (regen tap still doesn't release like it used to, but gas tap does. Still working on this)
  * Grey Panda support (Thanks @Regloom)

Version tw-0.8.12-12_tws (2022-12-26)
========================
 * MADS (Modified Assistive Driving Safety). Three independent, always-on safety features. Enable the MADS autosteer toggle in order to access the other features as well. If enabled, the max speed indicator is replaced by the MADS icon. *Reminder: You can use the cruise main button to cancel out all openpilot controls*. 
  * *Autosteer* comes on as soon as you start driving the car, before initially engaging, and even while brakes are pressed. *Toggle autosteer with the lane keep assist button on your steering wheel*. Optional toggle in settings to pause steering at low speeds when blinker is on and you're decelerating. When autosteer is enabled, the steering wheel icon will be colored ("disengaged" color when disabled).
  * *Lead braking* will be applied automatically, so OpenPilot will smoothly slow for other cars *if neither the gas or brake pedal are pressed in the slightest*. Use either pedal to override lead braking. *Toggle lead braking with the ACC follow distance button on your steering wheel*. When lead braking is enabled, an additional white ring is shown around the MADS icon.
  * *One-pedal driving*, where OpenPilot applies light braking when you're in L-mode and not pressing the gas, so you can come to a stop without using brakes. The feeling is as if L-mode regen braking brought you to a stop. *Toggle one-pedal driving with a double-press of the regen paddle*. You can also toggle one-pedal driving by tapping the MADS icon. When one-pedal driving is enabled, the MADS icon turns to the "engaged" color, or the "disengaged" color when disabled. (application of friction brakes originally suggested by cybertronicify — 10/06/2021)
  * *One-time one-pedal stop* by holding the regen paddle to below 5mph and one-pedal mode will activate until the next gas press
* On-screen weather display based on your current location. Tap weather icon to switch between simple/full display modes. Optionally enter your own OpenWeatherMap.org api key into `/data/OpenWeatherMap_apiKey.txt`
* Weather-based safety: automatically lower speed limit offset, switch to far-follow, mild acceleration, and mild curve braking for inclement weather
* New (mostly) UI metrics: engagement and interaction statistics
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
* Spruced up the toggles page

Version tw-0.8.12-11_tws (2022-12-09)
========================
  * Huge One Pedal Mode improvements
    * Blending of One Pedal and lead braking works much better, including when in regen-only mode and coast-mode. If you haven't tried it, you have to try it now
      * Make sure "one pedal pro brakes" is *disabled* in order to allow lead braking
    * Autosteer low-speed blinker pause below 20mph now only pauses steering when consistently slowing.  As you start driving with the blinker on, autosteer will resume (so openpilot can assist in intersections etc.)
    * Now you can use friction brakes to come to a stop once, and have it auto-return to regen-only mode after starting again
      * If you're *in regen-only mode* going *less than 40mph* with your *foot off the gas*, and you *engage friction brakes and slow to a stop* (below 5mph), then regen-only mode will turn back on automatically when you eventually press the gas to resume!
  * Lower min lane change speed to 20mph
  * Engine coolant temperature metric now turns orange at 215°F so you can check your EGR valve
  * Fixed display of metric units (thanks regloom!)
  * Adjust dynamic follow mode to not jump to close follow at the fart of a mouse
  * Reduce false positives for long range leads
  * Improved Volt torque lateral
  * Less curve braking

Version tw-0.8.12-10_tws (2022-11-22)
========================
  * Dynamic follow mode:
    * should work now!
    * now uses cut-ins **and adjacent traffic** to change (continuously) between close/medium/far follow 
      * traffic-based dynamic follow requires extended radar capabilities
  * Smoother application of curve braking
  * Auto lane position: 
    * various improvements
    * exclusively uses traffic to run (no more use of laneline presence)
  * New UI metrics:
    * time since driver intervention (gas, brake, or steering)
    * time since driver interaction (intervention, screen tap, blinker, cruise speed adjust, follow distance adjust)
    * time simce driver distraction
    * distance from lane center
    * number of cars being tracked by radar/long range vision (oncoming/ongoing/stopped/total/ongoing adjacent lane)
    * minimum separation between ongoing adjacent traffic (in seconds)
  * Print bearing above current speed (next to street name)
  * [Volt] Improved "torque" lateral control

Version tw-0.8.12-8_twd (2022-09-29)
========================
 * NEW: [toggle] Extended radar capabilities
   * (VERY ALPHA) brake for lead+1 car (the car in front of the lead)
     * if the lead+1 brakes and the lead doesn't see it and slams into them, openpilot will avoid the crash
       * when lead+1 is going 15mph+ *slower* than the lead, their speed (printed above lead indicator) will print larger and in red
     * if the lead changes lanes right before a stopped column of cars (prick), openpilot can still react in time
   * [toggle] Indicate oncoming/ongoing traffic in adjacent lanes: gray indicates stopped adjacent traffic (or "objects"), red for oncoming, and *engaged* color for ongoing
   * [toggle] Indicate *all* cars, marking them with transparent circles using the same color scheme
     * also prints speeds of all detected cars; tap at screen bottom at center of path to switch where speeds are printed
     * speed of cars in your own lane are printed above the lead indicator, or at screen bottom-center if no "lead" is detected
 * IMPROVED: [toggle] Traffic-based auto lane position
   * if there's only adjacent traffic on one side and clear lanelines, keeps you away from the traffic
   * same auto lane position but also uses traffic to adjust if extended radar capabilities are enabled
 * IMPROVED: [toggle] Traffic-based auto auto lane position (two autos)
   * Auto lane position comes on automatically if speed > 22mph, adjacent traffic present, and clear lanelines on both sides of car
   * As before, also comes on automatically if speed > 40mph, clear lanelines, and you're on a state highway or interstate
 * IMPROVED: auto "nudgeless" lane changes
   * will NOT auto lane change if adjacent lane is not detected
   * will NOT auto lane change if oncoming traffic in adjacent lane
   * on-screen alert will show countdown before starting auto lane change, or
   * if auto lane change is blocked, it will tell you why
 * IMPROVED: lane change alerts
   * lane change alert now warns if changing lanes when adjacent lane isn't detected or if oncoming traffic in adjacent lane
 * (RE)NEW: [toggle] One pedal mode lead braking, better than ever, but still WIP as it doesn't act identically to stopping behind lead when op-long enabled
   * To *disable*, enable the one-pedal pro brakes toggle which turns off lead braking
 * IMPROVED: better lead tracking 
   * by raising model height to typical Volt mounting height
 * IMPROVED: [toggle] long range lead detection 
   * fewer false positives and more consistent tracking
   * use filtered lead velocity and acceleration values
 * IMPROVED: [toggle] Volt torque lateral control
   * increased linear component of feedforward to right closer to inside of curves
   * no more roll *over*compensation
 * NEW: [toggle] alternate color scheme
   * instead of green/orange/red it's blue/purple/red
 * IMPROVED: [toggle] onscreen power meter
   * includes bars to indicate power losses to drag, rolling resistance, and elevation gain
   * good luck remembering which is which FML
   * if you're doing 80mph uphill, the first bar is drag losses, then rolling resistance losses above that, then power to gain elevation on top
   * tap meter/brake indicator to switch between brake indicator or power meter w/ or w/o readout
   * tap readout to switch units
 * (RE)NEW: print temperature in sidebar
 * IMPROVED: lanelines drawn as white when in laneless
 * IMPROVED: UI metrics: less tapping
   * EV metrics only show if you have an EV
   * Prevented from showing duplicate metrics
   * 9 new metrics: electric motor force/power (kw or hp), regen force/power (kw or hp), brake force/power (kw or hp)

Version tw-0.8.12-8_twd (2022-09-29)
========================
 * NEW: new power meter to replace brake indicator
  * Enable brake indicator in settings
  * Tap indicator or meter to toggle between the two or show text
  * Tap text to switch between kW and hp output
 * IMPROVED: one-pedal mode: smoother everything
 * IMPROVED: long range leads: 
  * smooth distance and velocity data for long range leads in proportion to the distance of the lead. So farther leads will be smoothed more to overcome the increased noise in the data at long distances, while the smoothing continuously decreases as the lead approaches, so that there is no added time lag to the data by the time they're within normal distance
  * improve logic used for qualifying lead, now takes the long range lead that's closest to the "middle" of your path (or center of the lane), but will take a closer lead that is farther from the middle so long as they're within 0.5m of the middle.
  * blue dot drawing correctly on lead indicator
 * NEW: UI metric: drag losses (shows percent of power being lost to drag when it makes sense, otherwise it shows kW of drag losses)
 * IMPROVED: drag, accel, and drive power metrics now come in W and hp units
 * IMPROVED: instantaneous EV consumption metric will switch to MW when applicable (how fun)


Version tw-0.8.12-7_twd (2022-09-15)
========================
 * NEW: 7 UI metrics: Drag (resistance) force, drag power, acceleration force (F=ma), acceleration power, drive power (drag + accel + losses), ICE power (for volt when ice is on; untested!), EV drivetrain (plus other stuff) efficiency (power from battery vs drive power)
 * IMPROVED: dynamic steer rate limit now scales with future curvature, so no more excessive correction to potholes at low speeds on straights
 * NEW: Auto automatic lane position when on highways/freeways and 38mph+ [optional]
 * NEW: UI metric: hybrid EV efficiency/consumption (flips based on sign of consumption)
 * FIXED: UI metric: EV trip and 5mi efficiency and consumption metrics are correctly preserved between drives
 * IMPROVED: use dynamic speed-based max brake command value for guaranteed max brake of -3.5m/s^2 as originally intended
  * Old method produced -3.5m/s^2 only with EV regen included. Now that value is maintained even below regen speeds.
 * when steering paused with LKA button, stop coloring path
 * draw transparent path
 * long range leads now indicated by blue dot
 * one pedal mode: improved braking profiles and smoother transitions
 * "communications error" alert now lists the process(es) that failed


Version tw-0.8.12-6_twd (2022-09-09)
========================
 * NEW: Bolt EUV support
 * NEW: [GM w/ ASCM] Include LKA camera vision data with radar for lead tracking
  * Now both vision and radar have to fail in order for a radar fault to be thrown!
  * Slightly better accuracy with lead tracking, but otherwise you won't notice a difference
 * NEW: [toggle] Longer-range lead-detection! (use with caution and report problems, please!)
  * Before, OP only considered a lead if it agreed with the model, but now it will use the long range voacc information, compared to model-predicted path and lanelines, in order to detect leads up to 600ft+ away
 * NEW: UI metrics: EV efficiency (three metrics: instantaneous, 5mi, and trip efficiency), and EV consumption (another three for instant, 5mi, and trip)
  * Use the "Reset EV consumption metrics" toggle to reset values back to zero.
 * IMPROVED: UI metrics: additional 2-column layouts
 * IMPROVED: Completely redone one-pedal mode logic. Smoother and more consistent braking.
  * No more mixed openpilot + one-pedal braking: you're in charge of braking!


Version tw-0.8.12-5_twd (2022-08-04)
========================
 * NEW: auto mode for adjustable lane position
  * First enable adjustable lane position toggle, then activate by pressing left then right (or right then left) within 2 seconds
  * Disable by pressing either lane position button
 * NEW: added toggle to reverse the stock speed change behavior
  * Tap/hold for 5mph/1mph cruise speed changes
 * NEW: added alerts to tell you when your data signal drops and restores (only if you're using map curve braking or speed limit control, otherwise you don't care)
 * NEW: When debug ui toggle enabled, print lane, adjacent lane, and shoulder widths (along with line confidence values in parentheses) along bottom of screen
 * NEW: Print current road name above current speed
 * IMPROVED: Use liveParams pitch for better longitudinal pitch compensation
 * IMPROVED: Only show "signal lost" alert if data features (map curve braking or speed limit control) are enabled
  * Also only show if change persists for more than 1 minute
 * IMPROVED: Nudgeless lane change now won't activate unless op knows there's an adjacent lane
 * IMPROVED: UI metrics don't respond to touches unless you first tap the current speed to unlock them (indicated by a blue border around the metrics). 10 second timeout to lock again.
 * IMPROVED: '17 Volt stop and go anti-rollback on hill.
  * It works by delaying the autoresume until the desired acceleration is more than enough to overcome the gravitational acceleration
 * IMPROVED: vision/map curve braking now go faster on interchanges (under 55mph, otherwise normal braking), and map braking goes faster also on interstate and state highways
 * IMPROVED: map curve braking, on freeways, will now only brake for 35mph+ curves, to prevent harsh braking resulting from false positives
 * IMPROVED: all curve braking now indicated using triangle speed sign at left of screen.
 * IMPROVED: speed limit control now shows US style speed limit sign (disable in toggles)
 * IMPROVED: improved Volt lateral performance
 * IMPROVED: improved low-speed lateral performance using dynamic steer rate limits
 * IMPROVED: smooth in acceleration after a lead turns right in front of you
 * IMPROVED: dynamic gas/brake threshold for more accurate gm long control

Version tw-0.8.12-4_tws (2022-07-20)
========================
 * NEW: Added "resume required" alert when lead car pulls away 
 * NEW: When map-curve braking or speed-limit control are enabled, current signal strength is shown above max speed indicator onroad
 * IMPROVED: ['17 Volt] Eliminate rollback when autoresuming behind lead on hill
 * IMPROVED: map braking: 
  * smooth in braking to lower severity of false positives (at cost of ~0.4s delay in braking)
  * if no signal, red circle instead of green around wheel icon at top-right
 * IMPROVED: nudgeless lane changes are fixed
  * 1.5s delay before initiating lane change
  * only works over 40mph
  * doesn't work in one-pedal mode
  * no nudgeless multi-lane changes, so if you leave your blinker on, it will not perform another lane change unless you nudge for the subsequent lane changes
 * IMPROVED: Volt updated "torque" and traditional steering feedforward fits (Basically converged at the "correct" feedforward at this point!)
 * IMPROVED: engine coolant UI metric uses higher color thresholds to better match dexcool limits
 * IMPROVED: adjustable lane position: 
  * use 1/2 mi and 10 mi distance timeouts instead of 15s and 10min
  * disables if steering wheel turns more than 150° 
 * IMPROVED: one-pedal mode:
  * slightly smoother stops when using light/moderate braking
  * always light/regen braking after engage-on-gas-press
 * IMPROVED: vision curve braking improvements
 * IMPROVED: gas/brake pitch-based adjust gets a 1% grade deadzone
 * IMPROVED: dynamic lane profile now switches to laneless before entering curves

Version tw-0.8.12-3_tws (2022-06-29)
========================
 * IMPROVED: Vision and/or map-based curve braking separately toggleable by tapping steering wheel icon
  * Tap to cycle between no-curve-braking/vision-only/vision+map
  * Vision-only is indicated by a white circle around the wheel icon, and vision+map is indicated by a green circle
 * IMPROVED: vision braking: higher "entering" curve braking
 * IMPROVED: map-based curve braking gets a 10% speed boost at all speeds so it doesn't slow excessively.
  * Please provide feedback on whether you think it should slow you more/less at low/high speeds
 * IMPROVED: gm long control now accounts for vehicle pitch for better gas/brake control on incline/decline (thanks qadmus!)
  * Pitch is calculated using current and predicted pitch to provide a smoothed version of the current pitch with no delay!
 * IMPROVED: gm better gas/brake lookup tables so that the car produces the accel/decel openpilot thinks it will for a given gas/brake command (qadmus again!!!)

Version tw-0.8.12-1_tws (2022-04-27)
========================
 * NEW: [optional w/ toggle] Torque-based steering control for Volt and Acadia
 * IMPROVED: [Volt] inproved (non-torque-based) steering performance too!
 * IMPROVED: ['18 Volt] auto-creep; still not as good as auto-resume
  * When lead car pulls away, brakes will release and car will creep
  * Press resume button or tap gas to resume
  * Car will apply brakes again if the lead stops, so it will auto-creep, but not completely resume
 * IMPROVED: improve c3 thermal control
 * IMRPOVED: increase brightness for more alerts when at low brightness
 * IMPROVED: wider path drawn when in e2e control
 * IMPROVED: fix path artifacts going over hills
 * IMPROVED: remove community features toggle

Version tw-0.8.9-3.11 (2022-03-28--2022-04-19)
========================
 * Put Comma Prime widget back on off-road screen
 * NEW: UI metric: device fanspeed (rpm)
 * NEW: Low-overhead mode for older devices
  * Sets default brightness to medium (feel free to tap DM icon to set it to stock/medium/low)
  * Disables loggerd and proclogd
  * Disables onroad uploads
  * (On C3, dropped cpu use while car turned on, in park, from 40% to 30%)
 * NEW: UI metrics: current lane width; device battery + current (non-C3)
 * IMPROVED: [Volt] smoother stopping behind lead
 ThE fAnCy-PaNtS uPdAtEd InFoGrApHiCs EdItIoN!
 * NEW: Adjustable lane positioning
   * Now with larger buttons!
   * Enable toggle, then tap onscreen (<) and (>) buttons to assume left/right position, resuming center position after 15s, 
   * or double tap to make it stick for 10 minutes
   * Tap same button again to disable immediately
   * Car will move over more in wider lanes than in narrower lanes, up to 0.8m (2'7.5") off center
 * NEW: Force Volt fingerprint using toggle
 * NEW: Toggle to ignore missing NVME SSD (for 32GB C3 units)
 * IMPROVED: Startup message with fingerprint is less visually aggressive 🐯
 * IMPROVED??? Vision braking is more aggressive
 * FIXED: White panda and Grey panda support!
   * "No GPS" error not shown for no-GPS pandas

Version tw-0.8.9-3.10 (2022-03-24)
========================
 * NEW: Color path according to (essentially) steering torque (inspired by kegman)
  * Laneful goes green to red, laneless blue to white
 * NEW: Toggle vision- AND map-based curve braking while driving by tapping the steering wheel icon
   * Extra white circle around icon indicates curve braking is enabled
   * Steering wheel changes to brake disk with eye to indicate that vision curve braking is active (turn off debug UI toggle)
 * IMPROVED: "Autotuned" PID controller for better lateral performance
 * IMPROVED: Retuned vision turn controller
  * Smoother entry/exit to/from curves
 * IMPROVED: Volt: improved low-speed long tune
  * Buttery stops or your money back!
 * IMPROVED: Smoother maintaining of cruise speed in stock/eco acceleration modes
  * No more "hunting" for the right speed while you involuntarily nod along.
 * IMPROVED?: Dynamic follow: tuning cut-in penalty
  * Please test in traffic, get cut-off, and provide feedback
  * "Honk gas honk; honk honk punch, gas gas gas." -HS

Version tw-0.8.9-3.9 (2022-03-08)
========================
 * NEW: [optional] Dynamic follow mode
   * Enable toggle to show onscreen button and indicator
   * Solid white circle on button means dynamic follow active
   * Tap button to toggle
   * Set follow distance manually to lock level for 5 minutes
   * Follow gap UI metric shows raw dynamic follow point value
 * NEW: Stop timer (inspired by sunnyhaibin)
 * NEW: Brightness control; tap driver monitoring icon to cycle brightness modes stock/medium/low
   * Screen will temporarily undim for warning/critical alerts
 * NEW: [Volt] [optional] control coasting for cruise and one-pedal mode using D and L mode with the gear shifter
   * Coasts in D, regen in L
   * Two separate toggles
   * Thanks to Jason Shuler
 * NEW: [Volt] [optional] tie one-pedal engage on gas setting to D/L position: off in D; on in L
   * Suggested by Shadowlight5
 * NEW: Display vehicle fingerprint on startup
 * NEW: UI metrics; Volt high voltage battery voltage, wattage, amperage, and volt+watt, lateral acceleration (from accelerometer and computed using steering angle + speed), vision curve brake debug output
 * NEW: Custom opgm spinner at boot
 * AGNOS 4
 * FIXED: Grey/White Panda support!
 * FIXED: [Volt] Prevent misprinting as Escalade ESV
 * FIXED: Disable Comma prime ad window in offroad view
 * IMPROVED: [Volt] Add derivative gain to lateral and longitudinal controllers with all new lateral tune (huge improvement in steering)
 * IMPROVED: Switch colored lanelines to use alert colors
 * IMPROVED: Reposition footer elements (buttons, face, and brake indicator) to keep them in view
 * IMPROVED: [Volt] Improved lateral control by adding derivative gain
 * IMPROVED: Tamer "stock" acceleration profile
 * IMPROVED: Brake indicator now shows regen/engine braking while cruising, or regen while in one-pedal regen mode
 * IMPROVED: UI Metrics now larger when four or fewer present (suggested by C Tyrell)
 * IMPROVED: Cherry-picked a number of improvements from upstream
    * Fix longitudinal oscillations.
    * Interpolate lateral plan for smoother steering
    * Road Roll Compensation Rebased (#23251)
    * params learner fix per
    * fix gm brake noise per
    * controlsd: remove redundant condition
    * use roll std from locationd
    * Controlsd: fix bug in curv rate limit
    * paramsd: Sort messages in each update iteration before processing
    * locationd: Gyro bias initial covariance
    * pitch and roll to car control
    * pitch roll to car control
    * locationd : Acceleration Bias in locationd 
    * decrease lateral planner max cost
    * Revert ecef std until real fix
    * longitudinal: only apply overshoot prevention when braking
    * Mesh3D: Add accelerometer bias to loc_kf)
    * tici: use powersave CPU governor while offroad
    * tici: higher cpu freq while offroad
    * alert text simplification

Version tw-0.8.9-3.8 (2022-01-24)
========================
 * FIXED: Now works with rev2 (shipped "with" 0.8.11) Comma Three units (sensor error resolved)
 * UNIMPROVED: Revert to "stock" behavior for disengaging move-fast speed limit control using cruise +/- buttons
 * IMPROVED: Reset speed while OP engaged using "set" button if 10+ mph over current set speed
 * IMPROVED: Retuned eco acceleration profile
 * IMPROVED: [Volt] reintroduce integral gain to lateral controller; better centering under constant force (e.g. slanted road or prevailing crosswind) and significantly better following of planned path on unmarked/covered roads
 * IMPROVED: Silky smooth transitions when switching between one-pedal brake modes

Version tw-0.8.9-3.7 (2021-12-27)
========================
 * NEW: 0.8.12 new OP sounds as default sound set
 * NEW: Alternate sounds now includes alternate sound for repeat warning (a.k.a. RSOD sound)
 * IMPROVED: Draw acceleration mode and laneless mode buttons even if there's an alert, since you can change them while alerts are shown
 * IMPROVED: Custom sounds and silencing engage/disengage take immediate effect
 * IMPROVED: Suppress move-fast map-based speed limit notifications when pressing/depressing gas in one-pedal mode
 * IMPROVED: Over-speed coasting properly uses move-fast speed limit
 * IMPROVED: 0.8.12 comma3 volume limits

Version tw-0.8.9-3.6 (2021-12-16)
========================
 * NEW: 0.8.12 new driving model (with higher max speed and fully integrated C3 training data!) and AGNOS 3
 * NEW: Acceleration mode button on-screen cycles between normal/sport/eco/creep acceleration profiles
 * NEW: Added toggle to disable uploads when on-road (and waits for 15 minutes after entering offroad state before starting uploads). Save that hotspot data!
 * NEW UI METRIC: instantaneous percent grade UI metric (the old one is now "GRADE (GPS)")
 * NEW UI METRIC: Engine coolant temperature (one for °C and one for °F)
 * NEW UI METRIC: Engine RPM with coolant temperature on the side (one for °C and one for °F)
 * FIXED: Engine RPM UI metric no longer rolls over to zero at 4095 rpms
 * IMPROVED: Engine RPM UI metric rounds to 10 rpms now instead of 100
 * IMPROVED: One-pedal mode braking is more consistent on incline/decline

Version tw-0.8.9-3.5 (2021-11-10)
========================
 * NEW: Latest 0.8.10 supercombo (laneless + laneful) model
 * NEW: Latest 0.8.10 driver monitoring model
 * NEW: Update to AGNOS2 for Comma3 users
 * NEW: Added follow level UI metric
 * IMPROVED: Regular/sport acceleration modes now work.
 * IMPROVED: Toggle coast mode while driving by tapping max speed (indicated by "+" after max speed; ring around brake indicator no longer shown)
 * IMPROVED: Disable one-pedal friction braking with double press of follow distance button when stopped or when gas pedal pressed
 * IMPROVED: Toggle one-pedal engage on gas feature while driving by tapping pedal icon (indicated by ring around pedal icon)
 * IMPROVED: Toggle auto-pause of auto-steer (when in one-pedal mode below 30mph with blinker) while driving by tapping wheel icon (indicated by ring around wheel icon while one-pedal mode active)
 * IMPROVED: Better (but still hackey) follow braking when coasting behind lead downhill
 * FIXED: After using one-pedal hard braking during a driving session, it no longer defaults to moderate braking for future one-pedal braking engagements
 * FIXED: JShuler panda gm steering fix

Version 0.8.9-tw-3.4 (2021-10-31)
========================
 Happy Halloween!
 * Revamped safety layer for avoiding collisions between coasting/one-pedal mode and braking when following lead, that reverts to stock longitidunal behavior for/when:
   1. Repidly approaching lead
   2. Slow/stopped lead
   3. Very close lead
   4. Low time to collision (i.e. rapidly approaching close lead)
   5. Following at less than target follow distance
 * Add time to collision (ttc) ui metric
 * Use moving average percent grade for smoother reading

Version 0.8.9-tw-3.3 (2021-10-29)
========================
 * lockout of coasting logic is now based on absolute and relative lead velocity. 
   * Start phasing out my coasting/one-pedal logic for lead approaching at 5mph+, completely returning to stock logic by 12mph+ (i.e. for a lead approaching at more than 12mph, coasting/one-pedal logic is entirely skipped over). 
   * Similar for absolute lead v, phasing out coast/one-pedal for lead driving less than 8mph, and completely locked out for lead speed less than 4mph. So when approaching a stopped lead, stock logic is in use. 
   * However, with one-pedal mode, the greater of one-pedal or cruise braking is applied, and if pro-braking is enabled, this continues to block ALL OP other braking.
 * TESTERS:
   * Please continue to play around with coasting and lead following.
   * Like to know if this is shippable to main branch with followup tuning or if problems need to be resolved beforehand.


Version 0.8.9-tw-3.2 (2021-10-29)
========================
 * disabling lockout of coasting logic after non-cruise braking is used. Was resulting in oscillations when over set speed. Please try following over set speed with coasting enabled to see what the behavior is. Concerned it may be too jerky and that the lead mpc braking will be too quickly applied/removed.

Version 0.8.9-tw-3.1 (2021-10-28)
========================
 * + fixing one-pedal distance-button engagement behavior to alwasy use regen/engine braking when you first engage
 * + coasting ring around brake indicator is more apparent
 * TESTERS: Lots of changes here in behavior and user interface.
   * I'd like more feedback on follow profiles; my driving doesn't include enough weird situations
   * The revamped coasting (with over-speed braking enabled) is a really cool feature, but only if it feels ok once OP starts applying brakes. I need feedback on how this actually performs. If it's dangerous I'll remove the feature from the fork until a proper mpc-based solution is complete
 1. Coasting means coasting! Before, the coasting toggle was merely disabling OP friction braking for maintaining set cruise speed. Now coasting really means coasting. If brake indicator enabled, then when coasting is enabled a circle will show around the brake indicator icon.
   1. Enable the over-speed braking toggle so that OP will start applying regen/engine braking when 9-11mph over set speed, and friction braking 12+ over
   2. Toggle coasting while driving by tapping the brake indicator icon
   3. You cannot disable coasting while driving above set speed unless engine/regen braking toggle is enabled
 2. Engine/regen braking toggle does what the coasting toggle used to do, disables OP friction braking for maintaining set cruise speed
 3. One-pedal mode changes
   1. Now only requires the "Disable disengage on gas" toggle be enabled
   2. One-pedal mode toggle does not need to be on! You toggle it while driving.
   3. One-pedal mode can now be engaged in three ways:
     1. While cruise is set, press and hold the follow distance button for 0.5s (continue to hold for immediate hard braking if necessary)
     2. If one-pedal engage on gas toggle is enabled, press gas while cruise is set and traveling above 1mph
     3. While cruise is set, lower cruise speed to 1
   4. Adjust braking intensity while one-pedal mode is active by pressing follow distance button to toggle light/moderate braking, or holding for heavy braking
   5. Toggle friction/engine/regen braking in two ways:
     1. Double-press follow distance button while gas is pressed to disable friction braking, or single press it while friction braking is disabled to reenable it
     2. Tap the pedal icon on your device screen
 4. Smoother following when at med/far distance behind lead car, despite the follow profile but especially smoother for far follow, thanks to dynamic acceleration costing
 5. Slightly more aggressive braking when approaching very slow/stopped lead car for med/far follow profiles
 6. Stronger positive acceleration (hopefully), especially in sport mode
 

Version 0.8.9-tw-2 (2021-10-22)
========================
 * TESTERS: The big changes are for follow profiles.
   * Please test out following with all three profiles and make observations.
   * My goal with follow profiles is that it feels like having three different people drive the car; all competent and skilled, but with different styles.
   * I want you to think up a character for each follow profile and put yourself in their shoes.
   * That in mind, do the profiles feel right? 
     * Is far follow using it's extra room to smooth out the ride but also comfortably braking in time for a much slower lead?
     * Is close follow close enough to make you a bit uncomfortable but confidently glued to the lead such that you're not uncomfortable?
     * Is medium follow driving the way you picture people driving in Neutropolis, the capital city of the Neutral Planet, home of the Neutrals?
 1. When in one-pedal mode, press resume to resume cruise at the speed you were going before entering one-pedal mode
 2. Smoother follow response thanks to reimplemented dynamic follow distance cost (thanks doug!)
 3. For medium/far follow profiles, proper long-distance braking for rapidly approaching (i.e. stopped or very slow) lead
 4. Smoother following close to lead using any follow profile

Version 0.8.9-tw-1 (2021-10-20)
========================
Lots of features and improvements over stock openpilot 0.8.9. See the readme for a fuller list [✅ = optional via toggle]:
 * Vastly improved steering tune for Chevy Volt (thanks qadmus!)
 * Autohold
 * Adjustable follow distance (remembers follow mode from last drive)
 * Toggle steering with LKAS button
 * Friction braking indicator
 * Customizable, dynamic device/vehicle/follow metrics (tap current speed indicator to cycle number of metrics shown; tap each metric to change what's shown in that slot)
 * [✅] Dynamic lane profile (autoswitch between lanelines and laneless)
 * [✅] Normal/sport acceleration
 * [✅] Downhill coasting (with optional braking when 10+mph over set speed)
 * [✅] Auto-on-steering lite (enable "disable disengage on gas" and "downhill coasting")
 * [✅] One-pedal driving (control amount of braking with follow-distance button)
 * [✅] Engage auto-on-steering/one-pedal mode with gas pedal
 * [✅] One-pedal "pro brakes" mode
 * [✅] Choose between 1mph/5mph button press/hold speed change or 5mph taps
 * [✅] 3mph cruise speed offset
 * [✅] Alternate sound set
 * [✅] Mute engage/disengage sounds

Version 0.8.9 (2021-09-14)
========================
 * Improved fan control on comma three
 * AGNOS 1.5: improved stability
 * Honda e 2020 support

Version 0.8.8 (2021-08-27)
========================
 * New driving model with improved laneless performance
   * Trained on 5000+ hours of diverse driving data from 3000+ users in 40+ countries
   * Better anti-cheating methods during simulator training ensure the model hugs less when in laneless mode
   * All new desire ground-truthing stack makes the model better at lane changes
 * New driver monitoring model: improved performance on comma three
 * NEOS 18 for comma two: update packages
 * AGNOS 1.3 for comma three: fix display init at high temperatures
 * Improved auto-exposure on comma three
 * Improved longitudinal control on Honda Nidec cars
 * Hyundai Kona Hybrid 2020 support thanks to haram-KONA!
 * Hyundai Sonata Hybrid 2021 support thanks to Matt-Wash-Burn!
 * Kia Niro Hybrid 2021 support thanks to tetious!

Version 0.8.7 (2021-07-31)
========================
 * comma three support!
 * Navigation alpha for the comma three!
 * Volkswagen T-Cross 2021 support thanks to jyoung8607!

Version 0.8.6 (2021-07-21)
========================
 * Revamp lateral and longitudinal planners
   * Refactor planner output API to be more readable and verbose
   * Planners now output desired trajectories for speed, acceleration, curvature, and curvature rate
   * Use MPC for longitudinal planning when no lead car is present, makes accel and decel smoother
 * Remove "CHECK DRIVER FACE VISIBILITY" warning
 * Fixed cruise fault on some TSS2.5 Camrys and international Toyotas
 * Hyundai Elantra Hybrid 2021 support thanks to tecandrew!
 * Hyundai Ioniq PHEV 2020 support thanks to YawWashout!
 * Kia Niro Hybrid 2019 support thanks to jyoung8607!
 * Škoda Octavia RS 2016 support thanks to jyoung8607!
 * Toyota Alphard 2020 support thanks to belm0!
 * Volkswagen Golf SportWagen 2015 support thanks to jona96!
 * Volkswagen Touran 2017 support thanks to jyoung8607!

Version 0.8.5 (2021-06-11)
========================
 * NEOS update: improved reliability and stability with better voltage regulator configuration
 * Smart model-based Forward Collision Warning
 * CAN-based fingerprinting moved behind community features toggle
 * Improved longitudinal control on Toyotas with a comma pedal
 * Improved auto-brightness using road-facing camera
 * Added "Software" settings page with updater controls
 * Audi Q2 2018 support thanks to jyoung8607!
 * Hyundai Elantra 2021 support thanks to CruiseBrantley!
 * Lexus UX Hybrid 2019-2020 support thanks to brianhaugen2!
 * Toyota Avalon Hybrid 2019 support thanks to jbates9011!
 * SEAT Leon 2017 & 2020 support thanks to jyoung8607!
 * Škoda Octavia 2015 & 2019 support thanks to jyoung8607!

Version 0.8.4 (2021-05-17)
========================
 * Delay controls start until system is ready
 * Fuzzy car identification, enabled with Community Features toggle
 * Localizer optimized for increased precision and less CPU usage
 * Retuned lateral control to be more aggressive when model is confident
 * Toyota Mirai 2021 support
 * Lexus NX 300 2020 support thanks to goesreallyfast!
 * Volkswagen Atlas 2018-19 support thanks to jyoung8607!

Version 0.8.3 (2021-04-01)
========================
 * New model
   * Trained on new diverse dataset from 2000+ users from 30+ countries
   * Trained with improved segnet from the comma-pencil community project
   * 🥬 Dramatically improved end-to-end lateral performance 🥬
 * Toggle added to disable the use of lanelines
 * NEOS update: update packages and support for new UI
 * New offroad UI based on Qt
 * Default SSH key only used for setup
 * Kia Ceed 2019 support thanks to ZanZaD13!
 * Kia Seltos 2021 support thanks to speedking456!
 * Added support for many Volkswagen and Škoda models thanks to jyoung8607!

Version 0.8.2 (2021-02-26)
========================
 * Use model points directly in MPC (no more polyfits), making lateral planning more accurate
 * Use model heading prediction for smoother lateral control
 * Smarter actuator delay compensation
 * Improve qcamera resolution for improved video in explorer and connect
 * Adjust maximum engagement speed to better fit the model's training distribution
 * New driver monitoring model trained with 3x more diverse data
 * Improved face detection with masks
 * More predictable DM alerts when visibility is bad
 * Rewritten video streaming between openpilot processes
 * Improved longitudinal tuning on TSS2 Corolla and Rav4 thanks to briskspirit!
 * Audi A3 2015 and 2017 support thanks to keeleysam!
 * Nissan Altima 2020 support thanks to avolmensky!
 * Lexus ES Hybrid 2018 support thanks to TheInventorMan!
 * Toyota Camry Hybrid 2021 support thanks to alancyau!

Version 0.8.1 (2020-12-21)
========================
 * Original EON is deprecated, upgrade to comma two
 * Better model performance in heavy rain
 * Better lane positioning in turns
 * Fixed bug where model would cut turns on empty roads at night
 * Fixed issue where some Toyotas would not completely stop thanks to briskspirit!
 * Toyota Camry 2021 with TSS2.5 support
 * Hyundai Ioniq Electric 2020 support thanks to baldwalker!

Version 0.8.0 (2020-11-30)
========================
 * New driving model: fully 3D and improved cut-in detection
 * UI draws 2 road edges, 4 lanelines and paths in 3D
 * Major fixes to cut-in detection for openpilot longitudinal
 * Grey panda is no longer supported, upgrade to comma two or black panda
 * Lexus NX 2018 support thanks to matt12eagles!
 * Kia Niro EV 2020 support thanks to nickn17!
 * Toyota Prius 2021 support thanks to rav4kumar!
 * Improved lane positioning with uncertain lanelines, wide lanes and exits
 * Improved lateral control for Prius and Subaru

Version 0.7.10 (2020-10-29)
========================
 * Grey panda is deprecated, upgrade to comma two or black panda
 * NEOS update: update to Python 3.8.2 and lower CPU frequency
 * Improved thermals due to reduced CPU frequency
 * Update SNPE to 1.41.0
 * Reduced offroad power consumption
 * Various system stability improvements
 * Acura RDX 2020 support thanks to csouers!

Version 0.7.9 (2020-10-09)
========================
 * Improved car battery power management
 * Improved updater robustness
 * Improved realtime performance
 * Reduced UI and modeld lags
 * Increased torque on 2020 Hyundai Sonata and Palisade

Version 0.7.8 (2020-08-19)
========================
 * New driver monitoring model: improved face detection and better compatibility with sunglasses
 * Download NEOS operating system updates in the background
 * Improved updater reliability and responsiveness
 * Hyundai Kona 2020, Veloster 2019, and Genesis G70 2018 support thanks to xps-genesis!

Version 0.7.7 (2020-07-20)
========================
 * White panda is no longer supported, upgrade to comma two or black panda
 * Improved vehicle model estimation using high precision localizer
 * Improved thermal management on comma two
 * Improved autofocus for road-facing camera
 * Improved noise performance for driver-facing camera
 * Block lane change start using blindspot monitor on select Toyota, Hyundai, and Subaru
 * Fix GM ignition detection
 * Code cleanup and smaller release sizes
 * Hyundai Sonata 2020 promoted to officially supported car
 * Hyundai Ioniq Electric Limited 2019 and Ioniq SE 2020 support thanks to baldwalker!
 * Subaru Forester 2019 and Ascent 2019 support thanks to martinl!

Version 0.7.6.1 (2020-06-16)
========================
 * Hotfix: update kernel on some comma twos (orders #8570-#8680)

Version 0.7.6 (2020-06-05)
========================
 * White panda is deprecated, upgrade to comma two or black panda
 * 2017 Nissan X-Trail, 2018-19 Leaf and 2019 Rogue support thanks to avolmensky!
 * 2017 Mazda CX-5 support in dashcam mode thanks to Jafaral!
 * Huge CPU savings in modeld by using thneed!
 * Lots of code cleanup and refactors

Version 0.7.5 (2020-05-13)
========================
 * Right-Hand Drive support for both driving and driver monitoring!
 * New driving model: improved at sharp turns and lead speed estimation
 * New driver monitoring model: overall improvement on comma two
 * Driver camera preview in settings to improve mounting position
 * Added support for many Hyundai, Kia, Genesis models thanks to xx979xx!
 * Improved lateral tuning for 2020 Toyota Rav 4 (hybrid)

Version 0.7.4 (2020-03-20)
========================
 * New driving model: improved lane changes and lead car detection
 * Improved driver monitoring model: improve eye detection
 * Improved calibration stability
 * Improved lateral control on some 2019 and 2020 Toyota Prius
 * Improved lateral control on VW Golf: 20% more steering torque
 * Fixed bug where some 2017 and 2018 Toyota C-HR would use the wrong steering angle sensor
 * Support for Honda Insight thanks to theantihero!
 * Code cleanup in car abstraction layers and ui

Version 0.7.3 (2020-02-21)
========================
 * Support for 2020 Highlander thanks to che220!
 * Support for 2018 Lexus NX 300h thanks to kengggg!
 * Speed up ECU firmware query
 * Fix bug where manager would sometimes hang after shutting down the car

Version 0.7.2 (2020-02-07)
========================
 * ECU firmware version based fingerprinting for Honda & Toyota
 * New driving model: improved path prediction during turns and lane changes and better lead speed tracking
 * Improve driver monitoring under extreme lighting and add low accuracy alert
 * Support for 2019 Rav4 Hybrid thanks to illumiN8i!
 * Support for 2016, 2017 and 2020 Lexus RX thanks to illumiN8i!
 * Support for 2020 Chrysler Pacifica Hybrid thanks to adhintz!

Version 0.7.1 (2020-01-20)
========================
 * comma two support!
 * Lane Change Assist above 45 mph!
 * Replace zmq with custom messaging library, msgq!
 * Supercombo model: calibration and driving models are combined for better lead estimate
 * More robust updater thanks to jyoung8607! Requires NEOS update
 * Improve low speed ACC tuning

Version 0.7 (2019-12-13)
========================
 * Move to SCons build system!
 * Add Lane Departure Warning (LDW) for all supported vehicles!
 * NEOS update: increase wifi speed thanks to jyoung8607!
 * Adaptive driver monitoring based on scene
 * New driving model trained end-to-end: improve lane lines and lead detection
 * Smarter torque limit alerts for all cars
 * Improve GM longitudinal control: proper computations for 15Hz radar
 * Move GM port, Toyota with DSU removed, comma pedal in community features; toggle switch required
 * Remove upload over cellular toggle: only upload qlog and qcamera files if not on wifi
 * Refactor Panda code towards ISO26262 and SIL2 compliancy
 * Forward stock FCW for Honda Nidec
 * Volkswagen port now standard: comma Harness intercepts stock camera

Version 0.6.6 (2019-11-05)
========================
 * Volkswagen support thanks to jyoung8607!
 * Toyota Corolla Hybrid with TSS 2.0 support thanks to u8511049!
 * Lexus ES with TSS 2.0 support thanks to energee!
 * Fix GM ignition detection and lock safety mode not required anymore
 * Log panda firmware and dongle ID thanks to martinl!
 * New driving model: improve path prediction and lead detection
 * New driver monitoring model, 4x smaller and running on DSP
 * Display an alert and don't start openpilot if panda has wrong firmware
 * Fix bug preventing EON from terminating processes after a drive
 * Remove support for Toyota giraffe without the 120Ohm resistor

Version 0.6.5 (2019-10-07)
========================
 * NEOS update: upgrade to Python3 and new installer!
 * comma Harness support!
 * New driving model: improve path prediction
 * New driver monitoring model: more accurate face and eye detection
 * Redesign offroad screen to display updates and alerts
 * Increase maximum allowed acceleration
 * Prevent car 12V battery drain by cutting off EON charge after 3 days of no drive
 * Lexus CT Hybrid support thanks to thomaspich!
 * Louder chime for critical alerts
 * Add toggle to switch to dashcam mode
 * Fix "invalid vehicle params" error on DSU-less Toyota

Version 0.6.4 (2019-09-08)
========================
 * Forward stock AEB for Honda Nidec
 * Improve lane centering on banked roads
 * Always-on forward collision warning
 * Always-on driver monitoring, except for right hand drive countries
 * Driver monitoring learns the user's normal driving position
 * Honda Fit support thanks to energee!
 * Lexus IS support

Version 0.6.3 (2019-08-12)
========================
 * Alert sounds from EON: requires NEOS update
 * Improve driver monitoring: eye tracking and improved awareness logic
 * Improve path prediction with new driving model
 * Improve lane positioning with wide lanes and exits
 * Improve lateral control on RAV4
 * Slow down for turns using model
 * Open sourced regression test to verify outputs against reference logs
 * Open sourced regression test to sanity check all car models

Version 0.6.2 (2019-07-29)
========================
 * New driving model!
 * Improve lane tracking with double lines
 * Strongly improve stationary vehicle detection
 * Strongly reduce cases of braking due to false leads
 * Better lead tracking around turns
 * Improve cut-in prediction by using neural network
 * Improve lateral control on Toyota Camry and C-HR thanks to zorrobyte!
 * Fix unintended openpilot disengagements on Jeep thanks to adhintz!
 * Fix delayed transition to offroad when car is turned off

Version 0.6.1 (2019-07-21)
========================
 * Remote SSH with comma prime and [ssh.comma.ai](https://ssh.comma.ai)
 * Panda code Misra-c2012 compliance, tested against cppcheck coverage
 * Lockout openpilot after 3 terminal alerts for driver distracted or unresponsive
 * Toyota Sienna support thanks to wocsor!

Version 0.6 (2019-07-01)
========================
 * New model, with double the pixels and ten times the temporal context!
 * Car should not take exits when in the right lane
 * openpilot uses only ~65% of the CPU (down from 75%)
 * Routes visible in connect/explorer after only 0.2% is uploaded (qlogs)
 * loggerd and sensord are open source, every line of openpilot is now open
 * Panda safety code is MISRA compliant and ships with a signed version on release2
 * New NEOS is 500MB smaller and has a reproducible usr/pipenv
 * Lexus ES Hybrid support thanks to wocsor!
 * Improve tuning for supported Toyota with TSS 2.0
 * Various other stability improvements

Version 0.5.13 (2019-05-31)
==========================
 * Reduce panda power consumption by 70%, down to 80mW, when car is off (not for GM)
 * Reduce EON power consumption by 40%, down to 1100mW, when car is off
 * Reduce CPU utilization by 20% and improve stability
 * Temporarily remove mapd functionalities to improve stability
 * Add openpilot record-only mode for unsupported cars
 * Synchronize controlsd to boardd to reduce latency
 * Remove panda support for Subaru giraffe

Version 0.5.12 (2019-05-16)
==========================
 * Improve lateral control for the Prius and Prius Prime
 * Compress logs before writing to disk
 * Remove old driving data when storage reaches 90% full
 * Fix small offset in following distance
 * Various small CPU optimizations
 * Improve offroad power consumption: require NEOS Update
 * Add default speed limits for Estonia thanks to martinl!
 * Subaru Crosstrek support thanks to martinl!
 * Toyota Avalon support thanks to njbrown09!
 * Toyota Rav4 with TSS 2.0 support thanks to wocsor!
 * Toyota Corolla with TSS 2.0 support thanks to wocsor!

Version 0.5.11 (2019-04-17)
========================
 * Add support for Subaru
 * Reduce panda power consumption by 60% when car is off
 * Fix controlsd lag every 6 minutes. This would sometimes cause disengagements
 * Fix bug in controls with new angle-offset learner in MPC
 * Reduce cpu consumption of ubloxd by rewriting it in C++
 * Improve driver monitoring model and face detection
 * Improve performance of visiond and ui
 * Honda Passport 2019 support
 * Lexus RX Hybrid 2019 support thanks to schomems!
 * Improve road selection heuristic in mapd
 * Add Lane Departure Warning to dashboard for Toyota thanks to arne182

Version 0.5.10 (2019-03-19)
========================
 * Self-tuning vehicle parameters: steering offset, tire stiffness and steering ratio
 * Improve longitudinal control at low speed when lead vehicle harshly decelerates
 * Fix panda bug going unexpectedly in DCP mode when EON is connected
 * Reduce white panda power consumption by 500mW when EON is disconnected by turning off WIFI
 * New Driver Monitoring Model
 * Support QR codes for login using comma connect
 * Refactor comma pedal FW and use CRC-8 checksum algorithm for safety. Reflashing pedal is required.
   Please see `#hw-pedal` on [discord](discord.comma.ai) for assistance updating comma pedal.
 * Additional speed limit rules for Germany thanks to arne182
 * Allow negative speed limit offsets

Version 0.5.9 (2019-02-10)
========================
 * Improve calibration using a dedicated neural network
 * Abstract planner in its own process to remove lags in controls process
 * Improve speed limits with country/region defaults by road type
 * Reduce mapd data usage with gzip thanks to eFiniLan
 * Zip log files in the background to reduce disk usage
 * Kia Optima support thanks to emmertex!
 * Buick Regal 2018 support thanks to HOYS!
 * Comma pedal support for Toyota thanks to wocsor! Note: tuning needed and not maintained by comma
 * Chrysler Pacifica and Jeep Grand Cherokee support thanks to adhintz!

Version 0.5.8 (2019-01-17)
========================
 * Open sourced visiond
 * Auto-slowdown for upcoming turns
 * Chrysler/Jeep/Fiat support thanks to adhintz!
 * Honda Civic 2019 support thanks to csouers!
 * Improve use of car display in Toyota thanks to arne182!
 * No data upload when connected to Android or iOS hotspots and "Enable Upload Over Cellular" setting is off
 * EON stops charging when 12V battery drops below 11.8V

Version 0.5.7 (2018-12-06)
========================
 * Speed limit from OpenStreetMap added to UI
 * Highlight speed limit when speed exceeds road speed limit plus a delta
 * Option to limit openpilot max speed to road speed limit plus a delta
 * Cadillac ATS support thanks to vntarasov!
 * GMC Acadia support thanks to CryptoKylan!
 * Decrease GPU power consumption
 * NEOSv8 autoupdate

Version 0.5.6 (2018-11-16)
========================
 * Refresh settings layout and add feature descriptions
 * In Honda, keep stock camera on for logging and extra stock features; new openpilot giraffe setting is 0111!
 * In Toyota, option to keep stock camera on for logging and extra stock features (e.g. AHB); 120Ohm resistor required on giraffe.
 * Improve camera calibration stability
 * More tuning to Honda positive accelerations
 * Reduce brake pump use on Hondas
 * Chevrolet Malibu support thanks to tylergets!
 * Holden Astra support thanks to AlexHill!

Version 0.5.5 (2018-10-20)
========================
 * Increase allowed Honda positive accelerations
 * Fix sporadic unexpected braking when passing semi-trucks in Toyota
 * Fix gear reading bug in Hyundai Elantra thanks to emmertex!

Version 0.5.4 (2018-09-25)
========================
 * New Driving Model
 * New Driver Monitoring Model
 * Improve longitudinal mpc in mid-low speed braking
 * Honda Accord hybrid support thanks to energee!
 * Ship mpc binaries and sensibly reduce build time
 * Calibration more stable
 * More Hyundai and Kia cars supported thanks to emmertex!
 * Various GM Volt improvements thanks to vntarasov!

Version 0.5.3 (2018-09-03)
========================
 * Hyundai Santa Fe support!
 * Honda Pilot 2019 support thanks to energee!
 * Toyota Highlander support thanks to daehahn!
 * Improve steering tuning for Honda Odyssey

Version 0.5.2 (2018-08-16)
========================
 * New calibration: more accurate, a lot faster, open source!
 * Enable orbd
 * Add little endian support to CAN packer
 * Fix fingerprint for Honda Accord 1.5T
 * Improve driver monitoring model

Version 0.5.1 (2018-08-01)
========================
 * Fix radar error on Civic sedan 2018
 * Improve thermal management logic
 * Alpha Toyota C-HR and Camry support!
 * Auto-switch Driver Monitoring to 3 min counter when inaccurate

Version 0.5 (2018-07-11)
========================
 * Driver Monitoring (beta) option in settings!
 * Make visiond, loggerd and UI use less resources
 * 60 FPS UI
 * Better car parameters for most cars
 * New sidebar with stats
 * Remove Waze and Spotify to free up system resources
 * Remove rear view mirror option
 * Calibration 3x faster

Version 0.4.7.2 (2018-06-25)
==========================
 * Fix loggerd lag issue
 * No longer prompt for updates
 * Mitigate right lane hugging for properly mounted EON (procedure on wiki)

Version 0.4.7.1 (2018-06-18)
==========================
 * Fix Acura ILX steer faults
 * Fix bug in mock car

Version 0.4.7 (2018-06-15)
==========================
 * New model!
 * GM Volt (and CT6 lateral) support!
 * Honda Bosch lateral support!
 * Improve actuator modeling to reduce lateral wobble
 * Minor refactor of car abstraction layer
 * Hack around orbd startup issue

Version 0.4.6 (2018-05-18)
==========================
 * NEOSv6 required! Will autoupdate
 * Stability improvements
 * Fix all memory leaks
 * Update C++ compiler to clang6
 * Improve front camera exposure

Version 0.4.5 (2018-04-27)
==========================
 * Release notes added to the update popup
 * Improve auto shut-off logic to disallow empty battery
 * Added onboarding instructions
 * Include orbd, the first piece of new calibration algorithm
 * Show remaining upload data instead of file numbers
 * Fix UI bugs
 * Fix memory leaks

Version 0.4.4 (2018-04-13)
==========================
 * EON are flipped! Flip your EON's mount!
 * Alpha Honda Ridgeline support thanks to energee!
 * Support optional front camera recording
 * Upload over cellular toggle now applies to all files, not just video
 * Increase acceleration when closing lead gap
 * User now prompted for future updates
 * NEO no longer supported :(

Version 0.4.3.2 (2018-03-29)
============================
 * Improve autofocus
 * Improve driving when only one lane line is detected
 * Added fingerprint for Toyota Corolla LE
 * Fixed Toyota Corolla steer error
 * Full-screen driving UI
 * Improved path drawing

Version 0.4.3.1 (2018-03-19)
============================
 * Improve autofocus
 * Add check for MPC solution error
 * Make first distracted warning visual only

Version 0.4.3 (2018-03-13)
==========================
 * Add HDR and autofocus
 * Update UI aesthetic
 * Grey panda works in Waze
 * Add alpha support for 2017 Honda Pilot
 * Slight increase in acceleration response from stop
 * Switch CAN sending to use CANPacker
 * Fix pulsing acceleration regression on Honda
 * Fix openpilot bugs when stock system is in use
 * Change starting logic for chffrplus to use battery voltage

Version 0.4.2 (2018-02-05)
==========================
 * Add alpha support for 2017 Lexus RX Hybrid
 * Add alpha support for 2018 ACURA RDX
 * Updated fingerprint to include Toyota Rav4 SE and Prius Prime
 * Bugfixes for Acura ILX and Honda Odyssey

Version 0.4.1 (2018-01-30)
==========================
 * Add alpha support for 2017 Toyota Corolla
 * Add alpha support for 2018 Honda Odyssey with Honda Sensing
 * Add alpha support for Grey Panda
 * Refactored car abstraction layer to make car ports easier
 * Increased steering torque limit on Honda CR-V by 30%

Version 0.4.0.2 (2018-01-18)
==========================
 * Add focus adjustment slider
 * Minor bugfixes

Version 0.4.0.1 (2017-12-21)
==========================
 * New UI to match chffrplus
 * Improved lateral control tuning to fix oscillations on Civic
 * Add alpha support for 2017 Toyota Rav4 Hybrid
 * Reduced CPU usage
 * Removed unnecessary utilization of fan at max speed
 * Minor bug fixes

Version 0.3.9 (2017-11-21)
==========================
 * Add alpha support for 2017 Toyota Prius
 * Improved longitudinal control using model predictive control
 * Enable Forward Collision Warning
 * Acura ILX now maintains openpilot engaged at standstill when brakes are applied

Version 0.3.8.2 (2017-10-30)
==========================
 * Add alpha support for 2017 Toyota RAV4
 * Smoother lateral control
 * Stay silent if stock system is connected through giraffe
 * Minor bug fixes

Version 0.3.7 (2017-09-30)
==========================
 * Improved lateral control using model predictive control
 * Improved lane centering
 * Improved GPS
 * Reduced tendency of path deviation near right side exits
 * Enable engagement while the accelerator pedal is pressed
 * Enable engagement while the brake pedal is pressed, when stationary and with lead vehicle within 5m
 * Disable engagement when park brake or brake hold are active
 * Fixed sporadic longitudinal pulsing in Civic
 * Cleanups to vehicle interface

Version 0.3.6.1 (2017-08-15)
============================
 * Mitigate low speed steering oscillations on some vehicles
 * Include board steering check for CR-V

Version 0.3.6 (2017-08-08)
==========================
 * Fix alpha CR-V support
 * Improved GPS
 * Fix display of target speed not always matching HUD
 * Increased acceleration after stop
 * Mitigated some vehicles driving too close to the right line

Version 0.3.5 (2017-07-30)
==========================
 * Fix bug where new devices would not begin calibration
 * Minor robustness improvements

Version 0.3.4 (2017-07-28)
==========================
 * Improved model trained on more data
 * Much improved controls tuning
 * Performance improvements
 * Bugfixes and improvements to calibration
 * Driving log can play back video
 * Acura only: system now stays engaged below 25mph as long as brakes are applied

Version 0.3.3  (2017-06-28)
===========================
 * Improved model trained on more data
 * Alpha CR-V support thanks to energee and johnnwvs!
 * Using the opendbc project for DBC files
 * Minor performance improvements
 * UI update thanks to pjlao307
 * Power off button
 * 6% more torque on the Civic

Version 0.3.2  (2017-05-22)
===========================
 * Minor stability bugfixes
 * Added metrics and rear view mirror disable to settings
 * Update model with more crowdsourced data

Version 0.3.1  (2017-05-17)
===========================
 * visiond stability bugfix
 * Add logging for angle and flashing

Version 0.3.0  (2017-05-12)
===========================
 * Add CarParams struct to improve the abstraction layer
 * Refactor visiond IPC to support multiple clients
 * Add raw GPS and beginning support for navigation
 * Improve model in visiond using crowdsourced data
 * Add improved system logging to diagnose instability
 * Rewrite baseui in React Native
 * Moved calibration to the cloud

Version 0.2.9  (2017-03-01)
===========================
 * Retain compatibility with NEOS v1

Version 0.2.8  (2017-02-27)
===========================
 * Fix bug where frames were being dropped in minute 71

Version 0.2.7  (2017-02-08)
===========================
 * Better performance and pictures at night
 * Fix ptr alignment issue in boardd
 * Fix brake error light, fix crash if too cold

Version 0.2.6  (2017-01-31)
===========================
 * Fix bug in visiond model execution

Version 0.2.5  (2017-01-30)
===========================
 * Fix race condition in manager

Version 0.2.4  (2017-01-27)
===========================
 * OnePlus 3T support
 * Enable installation as NEOS app
 * Various minor bugfixes

Version 0.2.3  (2017-01-11)
===========================
 * Reduce space usage by 80%
 * Add better logging
 * Add Travis CI

Version 0.2.2  (2017-01-10)
===========================
 * Board triggers started signal on CAN messages
 * Improved autoexposure
 * Handle out of space, improve upload status

Version 0.2.1  (2016-12-14)
===========================
 * Performance improvements, removal of more numpy
 * Fix boardd process priority
 * Make counter timer reset on use of steering wheel

Version 0.2  (2016-12-12)
=========================
 * Car/Radar abstraction layers have shipped, see cereal/car.capnp
 * controlsd has been refactored
 * Shipped plant model and testing maneuvers
 * visiond exits more gracefully now
 * Hardware encoder in visiond should always init
 * ui now turns off the screen after 30 seconds
 * Switch to openpilot release branch for future releases
 * Added preliminary Docker container to run tests on PC

Version 0.1  (2016-11-29)
=========================
 * Initial release of openpilot
 * Adaptive cruise control is working
 * Lane keep assist is working
 * Support for Acura ILX 2016 with AcuraWatch Plus
 * Support for Honda Civic 2016 Touring Edition

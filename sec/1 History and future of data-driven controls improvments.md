# History and future of data-driven controls improvements

This document describes the chronology of the overall project, as well as descriptions of how any why each type of improvement worked.

## Table of contents
- [History and future of data-driven controls improvements](#history-and-future-of-data-driven-controls-improvements)
  - [Table of contents](#table-of-contents)
- [Lateral controls](#lateral-controls)
  - [Where it started: Hewers with linear and sigmoidal functional feedforward fits using steering data](#where-it-started-hewers-with-linear-and-sigmoidal-functional-feedforward-fits-using-steering-data)
  - [How it got here: Improved steer angle fits; lateral acceleration, jerk, and gravitational acceleration fits](#how-it-got-here-improved-steer-angle-fits-lateral-acceleration-jerk-and-gravitational-acceleration-fits)
    - [Enough is never enough](#enough-is-never-enough)
    - [Improved lat-pid Volt fits](#improved-lat-pid-volt-fits)
    - [Other GM cars](#other-gm-cars)
    - [Enter torque controller](#enter-torque-controller)
    - [Torque controller feedforward fits](#torque-controller-feedforward-fits)
    - [More other cars](#more-other-cars)
    - [Lateral jerk fits](#lateral-jerk-fits)
      - [Only commanding "deliberate" lateral jerk](#only-commanding-deliberate-lateral-jerk)
    - [Lateral gravitational acceleration (roll compensation) fits](#lateral-gravitational-acceleration-roll-compensation-fits)
    - [tl;dr Three-part composite lateral feedforward](#tldr-three-part-composite-lateral-feedforward)
  - [Where it is now: Neural networks to do everything at once, using code that was 95% written by GPT4 (but I helped!)](#where-it-is-now-neural-networks-to-do-everything-at-once-using-code-that-was-95-written-by-gpt4-but-i-helped)
    - [Neural network to do the three-part composite feedforward all at once](#neural-network-to-do-the-three-part-composite-feedforward-all-at-once)
    - [Past/future context into the model](#pastfuture-context-into-the-model)
      - [Past/future data in training](#pastfuture-data-in-training)
    - [On-road runner](#on-road-runner)
      - [Error "friction" response](#error-friction-response)
      - [Full NN error response ("mk II")](#full-nn-error-response-mk-ii)
      - [NN error response "mk III"](#nn-error-response-mk-iii)
    - [Longitudinal acceleration response](#longitudinal-acceleration-response)
        - [What didn't work](#what-didnt-work)
        - [What did work: Adjust future times](#what-did-work-adjust-future-times)
      - [Community log collection ramps up](#community-log-collection-ramps-up)
      - [commaSteeringControl](#commasteeringcontrol)
  - [Where to try?](#where-to-try)
- [Future of data-driven lateral controls improvements](#future-of-data-driven-lateral-controls-improvements)

# Lateral controls

## Where it started: Hewers with linear and sigmoidal functional feedforward fits using steering data

In October 2021, [Hewers](https://github.com/hewers), submitted a [pull request](https://github.com/commaai/openpilot/pull/22621) to OpenPilot for improved lateral controls in the Chevy Volt by using a custom feedforward function derived from Volt driving data.
The resulting merged changes are still in effect for Volt upstream OpenPilot.

The standard quadratic `lat-pid` steer angle feedforward is `ff = kf * steer_angle * speed²`.
The new function is 
```ff = a * sigmoid * (speed + b),```
where
```sigmoid = (c * steer_angle) / (1 + ||c * steer_angle||)```
and `a`, `b`, and `c` are optimized using driving data.

Hewers arrived at this new fit by first noticing the terrible lateral controls in Volt at the time; constant lateral oscillations on straights, and repeated overshoot+correction in curves.
Starting (like all things good) with some of Shane's scripts, Hewers graphed steering torque as a function of steer angle at different speeds and found that assumed quadratic steering response was incorrect for Volt.

This was right as I was getting into OpenPilot in my Volt. 
I was fortunate enough that I never took a road trip on the old stock lateral controls; my first trip was on an early linear (instead of quadratic) FF Hewer's found.

-----
![hewers1](https://raw.github.com/twilsonco/openpilot/log-info/img/hewers1.png)

-----

![hewers2](https://raw.github.com/twilsonco/openpilot/log-info/img/hewers2.png)

-----

![hewers3](https://raw.github.com/twilsonco/openpilot/log-info/img/hewers3.gif)

-----

![hewers4](https://raw.github.com/twilsonco/openpilot/log-info/img/hewers4.png)

-----

Note the filtering/binning/regularization process described by Hewers, as that's very important.
*By keeping only near-zero-steer-rate points with near-zero longitudinal acceleration, the resulting data was answering the question of "how much torque does it take to **hold** a given angle at a given constant speed."*

The eventual fit used looked like this. The "old" red line is the steer torque calculated using the pre-Hewers feedforward. Hitting the dots is a good thing; if the line goes above the dots, then at that speed/angle you would expect the feedforward to introduce positive error (oversteer), or understeer if the line goes below the dots. The big deal here was that this was the only car to deviate from the `kf×(steer angle)×(speed)²` version of lat-pid feedforward
![fits1](https://user-images.githubusercontent.com/42746943/137976763-8ac0a839-bc67-48d3-b2fb-6d161243b889.gif)

(Hewers also did work on fitting a steer rate component to the feedforward and he produced a change that's still implemented in my fork for the lat-pid controller.)

## How it got here: Improved steer angle fits; lateral acceleration, jerk, and gravitational acceleration fits

### Enough is never enough

I've been improving controls by small and large steps, starting with Hewers' work (the largest single improvement), ever since, and I have some wisdom to share:
When something is really good, like 95% good, that last 5% may seem numerically small relative to the 95% goodness, but it turns out that humans care more and more about that last little bit of error the smaller it gets.
In every other walk of life, we lose our marbles over someone breaking a record by 0.01%.

The same goes for lateral controls in a car.
The difference between a 99.99% competent driver and a 98% competent driver cannot be seen on video but is palpable to the occupants of the vehicle.
There are subtle changes in controls (i.e. slightly steering or braking) a driver can apply to signal their level of awareness and their intent to the passengers.

For example, when a passenger has identified that the road will curve slightly to the right and dip to the left, they expect an appropriate response from the driver.
Each second they wait for a response, they wonder whether the driver is actually aware of the necessary action, and it causes them stress.

In the case of OpenPilot, you're constantly gauging whether intervention will become necessary.
The amount of time an OpenPilot user should have to spend wondering whether OpenPilot will act on some is zero.
Just as we fixate on hitting red lights over green lights, we fixate on the times when OpenPilot keeps us on our toes wondering if we should take over.

A 98% competent OpenPilot that keeps you on your toes 2% of the time is not chill.

### Improved lat-pid Volt fits

Since enough is never enough, and since Hewers has to work to live, I continued with his work and generated improved lat-pid feedforward fits using steer angle data from my own car. 

The improvements came from finer-tuning of the fit function, and from the fact that I was now fitting higher-quality steering data collected using the already improved sigmoid fit.
After around 4 iterations, the improved quality of the collected data from the improving fits finally converged to the best performing steer angle fit for Volt.

![better lat pid fit](https://raw.github.com/twilsonco/openpilot/log-info/data/8%20Manual%20preliminary%20data%20fits/volt%20steer%20angle%20twilsonco.gif)

The new fit function has 6 parameters instead of 3:

```
ff = a * sigmoid * (speed + b)^c + d * linear,
```
where 
```
sigmoid = steer_angle' / (1 + ||steer_angle'||),
steer_angle' = e * steer_angle / (speed + 0.01),
```
and
```
linear = f * steer_angle - atan(f * steer_angle)
```

Not surprisingly, the curve that fits the data better also performs better on-road in the form of fewer and smaller corrections.

### Other GM cars

Hewers also did a fit for GMC Acadia that was upstreamed.
I expanded the effort and ended up making lat-pid fits for several cars from manually collected rlogs:
 * GMC Acadia, Bolt EV, Bolt EUV, Chevy Silverado/GMC Sierra, Cadillac Escalade, Buick LaCrosse, Chevy Suburban, Hyundai Sonata

### Enter torque controller

A big change to OpenPilot in the last few years was the switch for many cars to use the "torque" lateral controller. 
The difference between the torque controller and the old, `lat-pid` "angle" controller, is in the quantity whose error is being controlled.

* For the angle controller, it is the difference between desired and actual **steering angle** that defines the error
* For the torque controller, it is the difference between desired and actual **lateral acceleration** that defines the error.
* *The current torque controller has changed; it now converts from lateral acceleration to steering torque, so that it is the difference in disired vs actual steer torque that is controlled.*

Now, the lat-pid "angle" controller gets a desired path curvature from the model/planner, and then uses the vehicle model to convert that path curvature to a steering angle in order to obtain the error quantity by comparing to the car's steering angle.
The torque controller, on the other hand, converts the desired path curvature into a lateral acceleration, and the has to use a vehicle model to convert from the car's current steer angle to path curvature, which is then converted to lateral acceleration in order to get the error quantity compared to desired lateral acceleration.
So, both controllers are using the car's steering angle to gauge the actual state, and the model/planner desired curvature as the desired state, *so what's the difference?*

The difference is in the typical values of lateral acceleration vs steer command. 
At low speeds, steer angle may range beyond ±360° at low speeds, but only to ±15° at higher speeds. 
This makes it difficult to ascribe a torque response based on steer error *at any speed*, since that error may be 3000% larger at low speeds.

Lateral acceleration, however, which includes longitudinal velocity in its calculation, is typically within ±3m/s² at any speeds, so the error response of the controller becomes much easier to engineer!
Instead, at low speeds, the error in the torque controller is too low at low speeds, so a "low speed factor" is used to make the error more sensitive in low-speed turns.
It shifts the difficult part of the error response to the low end, where accuracy is less important, while giving a consistent and safe error response at higher speeds.

### Torque controller feedforward fits

I tried out the torque controller when it was introduced, and it was already very good, but not as good as the lat-pid controller with Hewers' custom sigmoidal feedforward, so I endeavored to create a similar fit feedforward, but for steer torque as a function of lateral acceleration and speed rather than steer angle and speed.
This complicated the log-processing part of the process, since it's necessary to maintain a vehicle model in order to account for road roll and extract the "flat road" lateral acceleration data from OpenPilot logs.

But I got it figured out, and the resulting custom torque controller feedforward setup gave even better performance than the lat-pid setup. These fits used an expanded version of the same filtering/binning/regularizing method used by Hewers, and answer the question of "how much steer torque does it take to hold a given lateral acceleration at a given speed.

![Volt torque fit](https://raw.github.com/twilsonco/openpilot/log-info/data/8%20Manual%20preliminary%20data%20fits/volt%20lateral%20accel.gif)

Here the big distinction is two-fold. First, the relationship between torque and lateral acceleration is non-linear, and second, that there is a strong speed dependence, while the stock torque controller assumes a linear relationship with no speed dependence. 

The torque fit feedforward has 5 parameters:
```
ff = a^2 * sigmoid + b * (steer_angle + c),
```
where
```
sigmoid = erf(steer_angle'),
```
and
```
steer_angle' = d * (steer_angle + c) * (40 / (0.01 + speed))^e
```

The `a^2` should be just an `a`, but for some reason the `scipy` optimizer was able to better fit the function when that parameter was squared.
Squaring it makes the function more sensitive to changes in the parameter, but I would have expected the optimizer to have no problem changing `a` by small enough values to make it work anyways 🤷‍♂️

### More other cars

During this stage of the project I reached out to drivers of other makes to collect rlogs and ended up doing torque FF fits for a [number of cars](link to manual fits):

* All of the above plus: VW Golf 7th gen, VW Passat NMS, Hyundai Palisade, Dodge Ram 1500, Chrylser Palisade

### Lateral jerk fits

Recall that the lateral acceleration fits (and the angle fits before them) were made using data with near-zero steer rate (i.e. near-zero lateral jerk), so that the data tell us how much torque is necessary to *hold a given angle with the steering wheel still*.
That means that the amount of steer torque *necessary to make the steering wheel move* is being explicitly left out of the data.
And feedfoward's job is to predict the steer torque based on *whatever context determines it*.

With an accurate lateral acceleration-based feedforward in hand, I was was able to take that non-zero lateral jerk data and adjust it, subtracting off the amount of steer torque due to the lateral acceleration dependence, leaving only the steer torque due to the corresponding lateral jerk.

The resulting data, and fit, look like this.

| manual fits | neural network fit |
| --- | --- |
| ![volt lat jerk fit same sign](https://raw.github.com/twilsonco/openpilot/log-info/data/8%20Manual%20preliminary%20data%20fits/volt%20lateral%20jerk%20same%20sign.gif) | ![volt lat jerk response nnff](https://raw.github.com/twilsonco/openpilot/log-info/img/volt-lat-jerk-nnff.png) |
| ![volt lat jerk fit opposite sign](https://raw.github.com/twilsonco/openpilot/log-info/data/8%20Manual%20preliminary%20data%20fits/volt%20lateral%20jerk%20opposite%20sign.gif) | Above is the lateral jerk response learned by the NN. Pretty close to the manual fit! |

First, there are two plots because there are two primary cases to consider, based on whether the signs of lateral acceleration and lateral jerk are the same or opposite.
If they are the same, then it means you're turning the wheel into the curve, entering the curve.
If the sign of lateral acceleration and lateral jerk differ, then it means you are exiting the curve, returning the steering wheel back to center (or perhaps turning back into the next curve).

The data tell us a few things

1. At most speeds, the extra steer torque necessary to move the steering wheel is essentially constant with respect to the amount of lateral jerk. That is, it takes as much additional steer torque to make the steering wheel move fast as it does to move it more slowly; the real distinction is whether it's moving or not. Put another way, once you've given it enough steer torque to establish movement, you don't need to provide any more torque to move the wheel faster; that's determined by the angle you're trying to achieve.
2. The amount of torque necessary to initiate steering wheel movement *into a curve, increasing lateral acceleration* is different than the amount necessary to start the steering wheel moving *back to center, out of a curve*.
3. At low speeds, much more torque is required to achieve lateral jerk, in order to overcome a higher resistance from the steering rack and from the tires pivoting compared to at higher speeds where it's the redirection of the vehicle's considerable momentum, and at high speeds the suspension dynamics, that determine the lateral jerk response.

Neat.

I'm not going to rewrite the actual functions here, but you can check them out [directly](https://github.com/twilsonco/openpilot/blob/6832a74874a3a4caaff0a601200bba2926c0038f/selfdrive/car/gm/interface.py#L162) I'm like the picasso of writing fit functions 😁 They both use the sum of two sigmoids, but the opposite sign case includes an additional speed-based scalar in order to capture the slope increase at low speeds.

#### Only commanding "deliberate" lateral jerk  

Now, one issue with this lateral jerk feedforward is that there is an important missing piece of context, which is that *not all instances of desired lateral jerk should result in additional (or less) steering torque.*
Lateral jerk is the instantaneous rate of change of lateral acceleration.
Here it works to think of lateral accel as the steer angle itself and lateral jerk the steer rate, since they're similar.

A simple implementation of lateral jerk feedforward passes the model/planner desired lateral jerk to the feedforward function, which outputs some amount of torque due to the lateral jerk which is added to the lateral acceleration feedforward torque and commanded.
Because the lateral jerk data is essentially a `signum` function, it becomes a problem that nearly *any* amount of desired lateral jerk results in an increase or decrease of 15-50% steer torque. Especially if any overshoot results from the response, in which case you're oscillating back and forth with this "binary" torque output.

To solve this, I considered how a human drives—the best human; me, in the best car; the Volt.
When then best human drives, they do indeed apply additional torque in order to overcome steering column/rack friction, *but only when the anticipated angle change is large enough*.
For slight changes in steering angle, no such extra push is necessary (at least not in this case, since the best car has very tight steering).
Mathematically, any change in desired lateral acceleration must come with a non-zero desired lateral jerk, but we know they shouldn't always result in more/less steer torque, and the human only "commands torque" when the desired lateral jerk will be sustained over some period (i.e. the angle change is large enough).

Just like how a human knows how far they intend to change the angle, so too does the OpenPilot lateral planner provide the anticipated lateral acceleration and lateral jerk values for the next 2.5s.
So, I compute  ["lookahead"](https://github.com/twilsonco/openpilot/blob/6832a74874a3a4caaff0a601200bba2926c0038f/selfdrive/controls/lib/latcontrol_torque.py#L37) desired lateral jerk by taking the minimum magnitude planed lateral jerk value over the future plan. 
The time distance into the future [varies with speed](https://github.com/twilsonco/openpilot/blob/6832a74874a3a4caaff0a601200bba2926c0038f/selfdrive/controls/lib/latcontrol_torque.py#L162C45-L162C45) so that at high speeds, farther ahead values are used.
This reduces false-positive commanding of lateral jerk torque at high speeds, while still keeping the snappy response provided by lateral jerk feedforward at low speeds. 

The method works very well.
In the image below the plots show a 8s long curve at ~28mph, where three jerks of the steering wheel due to slight in-curve corrections were completely avoided, and instead the lateral jerk FF gave one deliberate push into the curve and one out of the curve just like you would expect.

![lat jerk ff plotjuggler](https://raw.github.com/twilsonco/openpilot/log-info/img/lookahead%20jerk.png)


### Lateral gravitational acceleration (roll compensation) fits

I then realized I could combine the heavy filtering of the lateral acceleration fits with the steer torque data correction used for the lateral jerk fits, in order to extract the steer torque due to lateral gravitational acceleration. 
That is, extract the observed roll compensation.

This time, it works by using only data points with no steer rate (no lateral jerk), but also subtracting off the steer torque due to lateral acceleration dependence.
The resulting data can be assumed to have no steer torque due to lateral jerk, because we only take points with (sustained) near-zero lateral jerk, and no steer torque due to lateral accel (how hard you're turning) since we subtract off the "known" amount of torque using our existing lateral acceleration feedforward.
That leaves only the steer torque due to the amount of road roll.

![volt roll fit](https://raw.github.com/twilsonco/openpilot/log-info/data/8%20Manual%20preliminary%20data%20fits/volt%20lateral%20gravitational%20accel.gif)

Here we see that, as implied in Comma's torque controller, the roll compensation is linear with respect to the amount of roll. There also seems to be a speed dependence.

*I thought of this early the morning of a road trip. I altered the preprocessing script, analyzed the data, created the fit, and wrote the new feedforward runner in OpenPilot in 3 hours and then drove it from Denver to Santa Fe 😅 (great drive!)*

The roll compensation fit function is simpler, with two parameters.

```
ff_roll = a * roll / (speed + 0.01)^b
```

### tl;dr Three-part composite lateral feedforward

So, there’s three parts: 

1. Lateral acceleration (or steer angle) FF fit from data points with zero steer rate and zero long acceleration. The fit is for steer command as a function of speed and desired lateral acceleration.  This alone gets lateral to 98% sufficient performance, and PID tune becomes unimportant since most error is simply avoided. 
2. Lateral jerk (or steer rate) FF fit from data points with zero long acceleration. Here the FF from (1) is used to subtract off the steer command due to lateral accel (or steer angle), so you isolate the steer command due to speed and lateral jerk. Now you have a two component FF
3. Roll compensation fit using again points with zero steer rate and zero long acceleration, but also using the FF from (1) to subtract off steer command due to steer angle. This data now shows the steer torque due to road roll (which varies with speed). 

## Where it is now: Neural networks to do everything at once, using code that was 95% written by GPT4 (but I helped!)

For the initial neural network feedforward (NNFF) fit, the process was simplified relative to the individual fits. Filtering was simplified, rejecting only points with too high magnitude of values (of lateral acceleration/jerk, roll, or longitudinal acceleration). Beyond that, we now desire to have all the points with different values of lateral acceleration, lateral jerk, speed, and road roll, so that the model can "learn" how different combinations of values relate to the resulting steer torque.

I create the NNFF on GPU using the Flux.jl Julia package ([link to repo](https://github.com/twilsonco/OP_ML_FF/blob/6822f2dc57d5a0db15086f6c7bc83c0fbc4c84cb/latmodel_temporal.jl#L350)).

Here's the [Discord discussion](https://discord.com/channels/469524606043160576/884811574773157949/1095108554387632170) where I lay out how GPT4 was used in this process. 

### Neural network to do the three-part composite feedforward all at once

With a relatively simple setup, I was able to recover preliminary lateral acceleration, lateral jerk, and roll compensation responses.

![nnff1](https://raw.github.com/twilsonco/openpilot/log-info/img/nnff1.png)

These early results showed promise, but lacked the desired symmetry of a feedforward function (it should take the same amount of torque to turn the same amount in either direction)

Then, after discussions with @nworby and others, I applied constraints to the training process to create what is called a physically informed neural network (PINN). The following physical constraints were applied:

1. The model output torque should increase for increasing lateral acceleration or lateral jerk, and decrease with increasing road roll (i.e. the first partial derivative of the model output with respect to those independent variables should be everywhere positive, except roll which should be negative)
2. The model output should be odd "at once" with respect to every independent variable other than longitudinal velocity. That is, if X lateral acceleration, with Y lateral jerk, and Z road roll requires T steer torque, then we expect that -T torque will be required to achieve -X lateral acceleration, -Y lateral jerk, and -Z road roll.
3. The model output should be zero when lateral acceleration, lateral jerk, and road road are all zero. That is, it should output zero torque when the expected causes of required torque are all zero.

In order to apply these "constraints" to the model, we create a multidimensional grid of input points that represent the different things we want to enforce. 
For (1) we make a grid for each partial derivative that needs to be checked, with points just slightly offset from the points of the "starting" grid such that we can approximate the first derivative with respect to each independent variable.
The difference in model output between corresponding points in the grids, e.g. the lateral acceleration offset test grid vs the starting test grid, provides an approximate first derivative.
Then we add up all the negative values (since we want it everywhere positive) and use those as a penalty added to the loss function.
Here's a link to the [relevant code](https://github.com/twilsonco/OP_ML_FF/blob/6822f2dc57d5a0db15086f6c7bc83c0fbc4c84cb/latmodel_temporal.jl#L449).

A similar process works to penalize the loss for deviation from oddness and passing through the origin.

The test grid used to calculate these "physically informed" loss function terms [extends beyond expected operating conditions](https://github.com/twilsonco/OP_ML_FF/blob/6822f2dc57d5a0db15086f6c7bc83c0fbc4c84cb/latmodel_temporal.jl#L359), so that we can have some confidence that the model is going to behave as expected at all times on-road.

The trained model is plotted and output in two formats.
One is a simple JSON structure that GPT4 just came up with when I said to [write it out to JSON](https://github.com/twilsonco/OP_ML_FF/blob/6822f2dc57d5a0db15086f6c7bc83c0fbc4c84cb/latmodel_temporal.jl#L917) so that I could read it into a Python program (JSON because I didn't want to have to install any new Python libraries to run the model, and it's cheap to run anyhow).
Then I asked for a [corresponding Python class](https://github.com/twilsonco/openpilot/blob/84668d0104998071572ceef5ba89bf81bd5daab5/selfdrive/car/interfaces.py#L60), `FluxModel`

Through this process, I was able to get nicer looking fits that very nearly 100% satisfy the "constraints" and drove better than the three-part composite FF.

![nnff2 with PINN](https://raw.github.com/twilsonco/openpilot/log-info/img/nnff2.png)

### Past/future context into the model

This version of NNFF was already an improvement.
Contrast to the standalone feedforward fits that employed heavy use of filtering in order to limit the context, the NNFF accounts for all the different combinations of lateral acceleration, lateral jerk, and road roll in order to recover the relationships between the different variables.
Additionally, it's easier to train a single model than to create the stepwise, interdependent feedforward fits.

However, this NNFF version still required the "lookahead" approach in order to avoid commanding steer torque for spurious instances of desired lateral jerk.
This isn't ideal since it introduces a few arbitrary parameters that may require tuning on a per-car basis; we want the model to remove all the guesswork.
A better approach is to pass past/future lateral acceleration data directly to the model.
This allows the model to figure out from the training data what constitutes "deliberate" desired lateral jerk and how that changes with other factors.

#### Past/future data in training

The anonymized lateral data files (both mine and Comma's) are in order, so if you go from point to point, you're seeing someone's actual drive, moment by moment.
To include past/future lateral acceleration and road roll in the model training data, you must derive it from the continuous time data, which amounts to interpolating through lists (or queues) of the most recent 3s of values (see [here](https://github.com/twilsonco/openpilot/blob/08ddaf2d304d5bff9650009c22683362481b57d2/tools/tuning/lat_to_csv.py#L1006) where I do this for community data, or [here](https://github.com/twilsonco/openpilot/blob/08ddaf2d304d5bff9650009c22683362481b57d2/tools/tuning/comma_csv_to_feather.py#L523) for commaSteeringControl data).

The resulting training data look like this:
| steer_cmd | v_ego   | lateral_accel | lateral_jerk | roll    | lateral_accel_m03 | lateral_accel_m02 | lateral_accel_m01 | lateral_accel_p03 | lateral_accel_p06 | lateral_accel_p10 | lateral_accel_p15 | roll_m03 | roll_m02 | roll_m01 | roll_p03 | roll_p06 | roll_p10 | roll_p15 |
|-----------|---------|---------------|--------------|---------|-------------------|-------------------|-------------------|-------------------|-------------------|-------------------|-------------------|----------|----------|----------|----------|----------|----------|----------|
| 0.2166    | 26.5287 | -0.0191       | -0.0037      | -0.0278 | -0.0120           | -0.0152           | -0.0178           | -0.0035           | 0.0024            | -0.0055           | 0.0193            | -0.0253  | -0.0263  | -0.0273  | -0.0277  | -0.0268  | -0.0280  | -0.0237  |
| 0.2130    | 11.1433 | 0.0692        | -0.0227      | -0.0245 | 0.0718            | 0.0730            | 0.0712            | 0.0665            | 0.0748            | 0.0803            | 0.0775            | -0.0223  | -0.0229  | -0.0236  | -0.0262  | -0.0235  | -0.0197  | -0.0204  |
| -0.0333   | 24.7607 | -0.0001       | 0.0902       | -0.0175 | -0.1060           | -0.0543           | -0.0169           | 0.0003            | 0.0132            | 0.0801            | 0.0861            | -0.0188  | -0.0184  | -0.0179  | -0.0197  | -0.0214  | -0.0198  | -0.0187  |
| 0.1800    | 3.8445  | -0.0050       | -0.0005      | -0.0198 | -0.0059           | -0.0052           | -0.0049           | -0.0046           | -0.0025           | -0.0013           | 0.0002            | -0.0199  | -0.0199  | -0.0199  | -0.0197  | -0.0197  | -0.0196  | -0.0196  |
| -0.0200   | 25.5436 | -0.0475       | 0.1771       | -0.0163 | -0.1139           | -0.1026           | -0.0466           | 0.0025            | -0.0022           | -0.0097           | 0.0070            | -0.0168  | -0.0168  | -0.0166  | -0.0173  | -0.0208  | -0.0210  | -0.0232  |
| -0.3132   | 24.0322 | -0.0268       | -0.0784      | 0.0037  | -0.0265           | -0.0264           | -0.0252           | -0.0513           | -0.0533           | -0.0457           | -0.0360           | 0.0049   | 0.0051   | 0.0044   | 0.0032   | 0.0019   | 0.0015   | 0.0035   |
| -0.0066   | 16.1813 | 0.1055        | 0.0994       | 0.0070  | 0.0866            | 0.0884            | 0.0837            | 0.1136            | 0.1037            | 0.1187            | 0.0652            | 0.0077   | 0.0072   | 0.0067   | 0.0071   | 0.0050   | 0.0022   | -0.0070  |
| 0.4398    | 33.6643 | 0.0326        | -0.1898      | -0.0202 | 0.0647            | 0.0861            | 0.0704            | 0.0172            | 0.0115            | -0.0026           | -0.0493           | -0.0210  | -0.0204  | -0.0197  | -0.0211  | -0.0216  | -0.0210  | -0.0214  |
| 0.0300    | 29.6182 | 0.0490        | 0.0315       | -0.0250 | 0.0359            | 0.0617            | 0.0609            | 0.0653            | 0.0331            | 0.0024            | 0.0022            | -0.0247  | -0.0247  | -0.0248  | -0.0248  | -0.0253  | -0.0252  | -0.0242  |
| 0.0099    | 16.6555 | 0.0367        | -0.1060      | 0.0054  | 0.0808            | 0.0785            | 0.0612            | 0.0388            | 0.0139            | -0.0028           | -0.0279           | 0.0064   | 0.0055   | 0.0051   | 0.0060   | 0.0081   | 0.0088   | 0.0069   |
| -0.2994   | 25.2623 | 0.1134        | -0.2740      | 0.0235  | 0.1440            | 0.1476            | 0.1423            | 0.0442            | 0.0359            | -0.0547           | -0.0603           | 0.0274   | 0.0266   | 0.0251   | 0.0199   | 0.0178   | 0.0124   | 0.0071   |
| -0.2667   | 29.2833 | 0.0406        | 0.0200       | 0.0323  | 0.0177            | 0.0423            | 0.0412            | 0.0328            | 0.1328            | 0.2385            | 0.3098            | 0.0324   | 0.0325   | 0.0325   | 0.0321   | 0.0320   | 0.0306   | 0.0299   |
| 0.0400    | 22.7731 | 0.4932        | 0.0299       | 0.0544  | 0.4889            | 0.4953            | 0.4947            | 0.5310            | 0.5364            | 0.6036            | 0.6185            | 0.0555   | 0.0552   | 0.0549   | 0.0536   | 0.0546   | 0.0566   | 0.0580   |

Then, when training the model, the physical constraints also have to be [applied to the temporal values of lateral acceleration and road roll](https://github.com/twilsonco/OP_ML_FF/blob/6822f2dc57d5a0db15086f6c7bc83c0fbc4c84cb/latmodel_temporal.jl#L385).

With this—and lots of trial & error & iterations—you get the current generation of NNFF, which includes past/future lateral acceleration and road roll.

| lateral acceleration response | error, lateral jerk, and roll responses |
| --- | --- |
| ![nnff-a](https://raw.github.com/twilsonco/openpilot/log-info/data/7%20Comma%20lateral%20torque%20NNFF%20fits/CHEVROLET_VOLT_PREMIER_2017-a.png) | ![nnff-b](https://raw.github.com/twilsonco/openpilot/log-info/data/7%20Comma%20lateral%20torque%20NNFF%20fits/CHEVROLET_VOLT_PREMIER_2017-b.png) |

See here to [understand the NNFF model plots](https://github.com/twilsonco/openpilot/blob/log-info/sec/2%20Understanding%20the%20NNFF%20model%20and%20plots.md)


### On-road runner

To run such a NNFF on-road, you [interpolate through future planned lateral acceleration and model-predicted road roll](https://github.com/twilsonco/openpilot/blob/84668d0104998071572ceef5ba89bf81bd5daab5/selfdrive/controls/lib/latcontrol_torque.py#L111), and [maintain past data in queues](https://github.com/twilsonco/openpilot/blob/84668d0104998071572ceef5ba89bf81bd5daab5/selfdrive/controls/lib/latcontrol_torque.py#L104).
With this you can fill the entire length-18 input vector.

#### Error "friction" response

The "error response" in the plot above (right) *is not an error response trained into the model*.
Rather, that is the *instantaneous* lateral jerk response, while the "lateral jerk response" is the implicit lateral jerk response of the model due to changing past/future lateral acceleration values.

So, lateral jerk is double-represented in the NNFF model, which allows us to pass the lateral acceleration error in place of instantaneous lateral jerk on-road. 
This is providing the same desired effect as "friction" in the stock torque controller, providing the little push to get the steering wheel moving.
This makes the *actual* error response more effective, since the steering wheel will be moving as the error response kicks in rather than needing the response to "build up" enough to initate movement.
It also makes the error response smoother, because the actual correction occurs sooner, preventing a yet higher error response that results in overshoot and further correction.
Instead the error stays low, and the resulting lower error response yields a smooth correction back to the target.

#### Full NN error response ("mk II")

Taking inspiration from Harald's current torque controller, I [convert from lateral acceleration error to steer torque error](https://github.com/twilsonco/openpilot/blob/84668d0104998071572ceef5ba89bf81bd5daab5/selfdrive/controls/lib/latcontrol_torque.py#L125) using the feedforward function.
With the current torque controller, this amounts to a linear scaling of the error, but with NNFF (or any non-linear fits) you get a dynamic error response that's unlike any other type of lateral controller (100% credit to Harald for how this came to be).
That that means is that [the current NN torque controller](https://github.com/twilsonco/openpilot/commit/a10ed4d637f8b6d48b1bc018688355c3f07a3ac2) error response is dynamic based on lateral acceleration error, lateral jerk error, and varies based on speed and road roll, **just like a human error response**.

#### NN error response "mk III"

Although the concept of the "mk II" error response is great, numerically it doesn't work well when there's a sigmoidal component to the lateral acceleration response, for reasons made obvious in the following image.

| o _ O | ( __ ^ __ ) |
| --- | --- |
| ![error mk2](https://raw.github.com/twilsonco/openpilot/log-info/img/error-mk2.jpg) | In tight (high lateral accelration) corners, the difference in steer torque (∆y) for a given error (here, 1m/s²) is less than in slight curves. That is, ∆y1 > ∆y2. |

So, "mk III" instead uses a simultaneously simpler and more complicated approach, where the lateral acceleration error is passed as the lateral acceleration NNFF input, and the lateral jerk error as the lateral jerk input.
This guarantees that the error response comes from the steepest (most responsive) part of the NNFF lateral acceleration slope.

The past lateral acceleration error is recorded and used, and for computing the future error, I devised a simple function that assumes that future error should tend towards zero, while using a continuous assumption based on previous error. Here's the [relevent code](https://github.com/twilsonco/openpilot/blob/d5d563d736f4ed6afa464c02b33929579fc9bc81/selfdrive/controls/lib/latcontrol_torque.py#L31), and a graphical depiction of what this future predicted error looks like for different types of past error values. 

![predicted error](https://raw.github.com/twilsonco/openpilot/log-info/img/predicted-error.png)

This approach is what I'm currently using, and it is giving the best lateral performance yet for the Volt.

### Longitudinal acceleration response

I think the majority of the steer torque necessary at a given instance can be understood almost completely from the context provided to NNFF, but recall that *enough is never enough*.
The only obvious missing piece of context here is longitudinal acceleration.

##### What didn't work

I attempted to fit a longitudinal acceleration response using the old extended Hewers method, by isolating it with clever filtering and then plotting it. I got something that looked a lot like the lateral jerk fits, or like nothing really, and the road tests were failures.

| longitudinal acceleration fit attempt 1 | long accel fit another attempt |
| --- | --- |
| ![long accel fits](https://raw.github.com/twilsonco/openpilot/log-info/img/long%20accel1.gif) | ![long accel 2](https://raw.github.com/twilsonco/openpilot/log-info/img/long%20accel2.gif) |

I also attempted to fit the longitudinal acceleration response by including it as an input to NNFF. It didn't improve the on-road experience and in the plots it shows behavior that's difficult to physically explain.

![nnff with long accel](https://raw.github.com/twilsonco/openpilot/log-info/img/nnff%20long%20accel.png)

In the right column, the different paths show the model response to longitudinal acceleration.

Thinking about the problem physically, longitudinal acceleration is related to lateral acceleration because lateral acceleration is determined by longitudinal velocity.
So, in a constant curve under positive (forward) longitudinal acceleration, you know that lateral accel has a positive rate of change, and hence you would apply more steer torque to offset the higher future anticipated lateral acceleration.
Conversely, if you're slowing in a constant curve, lateral accel has a negative rate and you would decrease steer torque in anticipation.

For higher magnitude longitudinal acceleration, vehicle (suspension) dynamics come into play in a big way.
When you hit the gas in a constant curve, the lurch to the outside might require a temporarily higher amount of required torque, and you might get some weird rapid oscillations afterwards.
If you slam on the brakes in a tight constant curve, the forward jerk will require that you temporarily increase steer torque even though you're slowing.
In the latter case, you also don't want to be telling OpenPilot to turn harder when you're braking into a corner since you might flip or something.

I expect that NNFF could capture the subtlety of this relationship, but that the data we have doesn't adequately sample the longitudinal acceleration space with the other variables.

##### What did work: Adjust future times

In lieu of the ability to extract the relationship from the data, we can rethink our expectation of a longitudinal acceleration response.
The NNFF "knows what to do" for the upcoming lateral accelerations and road rolls, and if we're accelerating then we'll get to the future values sooner than anticipated based on current velocity.
By [adjusting the lookup times](https://github.com/twilsonco/openpilot/blob/84668d0104998071572ceef5ba89bf81bd5daab5/selfdrive/controls/lib/latcontrol_torque.py#L110) for future values according to longitudinal acceleration, we can provide NNFF with the correct future values for the current longitudinal acceleration.
This allows NNFF to respond to the changing plan, and allows us to incorporate a longitudinal acceleration response via its time effect rather than its direct relationship to lateral acceleration.

#### Community log collection ramps up

As NNFF was being worked on, I expanded my [log collection](https://github.com/twilsonco/openpilot/tree/log-info) efforts. Eventually I was collecting logs from 51 users, and [adequate logs were collected](https://github.com/twilsonco/openpilot/blob/log-info/2 Community lateral data.md) to generate NNFF fits [for 42 cars](https://github.com/twilsonco/openpilot/blob/log-info/3a Community lateral torque NNFF fits steer command.md) using steer command as torque input source. 
Other fits were made using different torque sources.

#### commaSteeringControl

After being visited by a ghost, Comma felt compelled to graciously release steering data for over 100 different models in the [commaSteeringControl](https://github.com/commaai/comma-steering-control) dataset.
This resulted in [NNFF fits for 106 cars](https://github.com/twilsonco/openpilot/blob/log-info/6 Comma lateral torque NNFF fits.md).

## Where to try?

`installer.comma.ai/twilsonco/nnff-ni-driving` is the current "master"-based fork with NNFF for 122 cars total
`installer.comma.ai/twilsonco/nnff-sunny` is the SunnyPilot `dev-c3` branch with NNFF added

# Future of data-driven lateral controls improvements

Comma has expressed an interest in moving to neural network controls. Hopefully everything here makes that process a little smoother.

My current task is reimplementing my entire training stack into `tools/` and `torqued` where the same preprocessing and training code used to create fits server-side from large amounts of data with TinyGrad on GPU will be run onroad for live nnff fits.
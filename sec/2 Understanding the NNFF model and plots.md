# How to interpret the NNFF plots

The following is from an explanation I gave in the SunnyPilot discord server `#tuning-nnff` channel.

-----

![comma lat data plot](https://raw.githubusercontent.com/twilsonco/openpilot/log-info/data/6a%20Comma%20lateral%20data%20combined%20firmware/TOYOTA_RAV4_HYBRID_2019_combined_lat.png)

That's steer torque vs lat accel. There's many columns because comma provided 2 steer torque versions and 3 lat accel versions = 6 combinations I checked
The slope of the point clouds shows the amount of torque necessary to hit different lateral accelerations at different speeds.

![nnff-a plot](https://raw.githubusercontent.com/twilsonco/openpilot/log-info/data/7%20Comma%20lateral%20torque%20NNFF%20fits/TOYOTA_RAV4_HYBRID_2019-a.png)

Then here's the NN fit
Where the different lines show how the value of torque changes, for a given lateral accel and speed, if you also change some other parameter. First column shows the torque going up/down due to error response (similar to "friction" in the torque controller)
second column shows how the torque changes up or down in response to lateral jerk (i.e. moving the steering wheel at a given rate). The way the model get's this information is from past/future planned lateral accel values
right column is the road roll response
Then here's the cool part

![nnff-b plot](https://raw.githubusercontent.com/twilsonco/openpilot/log-info/data/7%20Comma%20lateral%20torque%20NNFF%20fits/TOYOTA_RAV4_HYBRID_2019-b.png)

Plot it this way and you can see the model's error response specifically (left column), and now the different lines are showing how the torque at different lateral accels would change as the error that needs to be corrected changes

Just like how you correct differently when in a curve at 80mph than you do going straight at 25
2nd column is again lateral jerk (steering angle rate) response. Here you can see that at high speeds, there's no additional torque necessary to start moving the steering wheel in the 2019 Rav4H, since the lines become flat in the bottom plots in the second column.
See how this is extracting the physical characteristics of the EPS? 🤓

![error response close](https://raw.github.com/twilsonco/openpilot/log-info/img/nnff-talk-error-response-close.png)

forgot to specify visually. here's the error response. So left/right is error to the left/right, and the stacked lines show the torque required to correct for that error at different lateral accelerations (i.e. correcting for error on a straight road vs in a curve)

Here's the lateral jerk response leveling off as you increase speed. high slope at low speed. flat at high speed

![lat jerk response whole column](https://raw.github.com/twilsonco/openpilot/log-info/img/nnff-talk-lat-accel-temporal.png)

Which matches with what you would expect

Third column is again road roll. Pay attention because this one's cool. The middle line shows that you increase torque (steer right) to compensate for negative roll (leaning left), which tracks. 

And here the high lateral accel lines (the greener lines for ±2m/s^2) are flat on one side but not on the other. Looking just at the +2m/s^2 line on top (steering to the right), it's flat to the left (leaning left) and not to the right (leaning right). This indicates that if you're turning to the right, then you're already leaning to the left (negative roll) due to the hard turn, so additional negative roll to the left doesn't increase the required torque very much. But in that same turn, if the road roll were positive, to the right, then it would counteract the lateral acceleration from the curve, so for positive road roll the required torque decreases. That is, you don't have to turn as hard on a banked curve. Here we see it's been captured by the model quite well.

![road roll close](https://raw.github.com/twilsonco/openpilot/log-info/img/nnff-talk-roll.png)

Right column is the most fun to think about. This is the roll rate response, i.e. the roll is changing over time, leaning first to the left and then shifting to the the right (or vice versa). Positive roll rate means you're shifting to the right.

![roll rate column](https://raw.github.com/twilsonco/openpilot/log-info/img/nnff-talk-roll-rate.png)

So, if you were on a road that was like a track in F-Zero, where the roll was quickly changing from left to right, you would preemptively steer left so that you maintained center while the roll changed. 
This is precisely what the plots show us. The stacked lines here are showing different roll rates, and then going left/right is different road roll. So you're at a certain roll right now, and the rate is changing by some amount (or it's not changing in which case you're looking at the middle line which is identical to the middle line of the third column)
So looking at one of the magenta lines (+0.2 roll rate). This tells us that, compared to the zero roll rate middle line, we'd have less steer torque (steer left) in response to positive roll rate (going from leaning left to leaning right). This is exactly what we just decided would be the human response to changing roll rate above!
THEN!!!! As you go down the plots (from low to high speed) you see that the amount of compensation for this is less at high speed where the vehicle dynamics (and perhaps the EPS firmware) compensate for the changing roll.
So you can see how this NN dissects the characteristics of the EPS
and with a very small number of parameters.

And the best part is it’s immediately obvious to literally anyone who’s spent the last 2 1/2 years fixated on this process!

The full list of model inputs is listed at the top of the main plots. the "m03" and "p15" etc. stand for "time minus 0.3s" and "time plus 1.5s" respectively.

```steer_cmd, v_ego , lateral_accel, lateral_jerk, roll , lateral_accel_m03, lateral_accel_m02, lateral_accel_m01, lateral_accel_p03, lateral_accel_p06, lateral_accel_p10, lateral_accel_p15, roll_m03, roll_m02, roll_m01, roll_p03, roll_p06, roll_p10, roll_p15```

The "error response" (left columns of the NNFF plot grids) is merely me passing the lateral accel/jerk error in as the lateral_jerk input. That is, it's actually the model's instantanous lateral jerk response, but when using the model onroad, I pass in the error instead of lateral jerk into that input.

Results
=======

What Worked
-----------

Overall, our robot was very successful in pathing toward waypoints, recalculating paths if it was 
unable to reach them in time. We were able to achieve successful runs 

Our robot was able to complete the course in an average of _ seconds, making it the second fastest overall.
We were also able to push both cups out of their circles, scoring a point bonus.

Three videos are included below, showing our best runs.

`LINK TO VIDEOS <https://drive.google.com/drive/folders/1Ekb3K-w9zkbQRXt73vfwutiFQ6gePv0x?usp=sharing>`__

What Didn't Work
----------------

While our robot was able to complete the course a few times, it was far less reliable than we had hoped.
Our state-estimation based approach to orienting our robot caused us to lose track of our position when
our wheels slipped at all, which happened often at high-speed turning. This forced us to lower our speed
on turns, increasing our run time.

Our heading was also calibrated at the beginning of the course, causing massive drifts from expected
positions when misaligned at all during calibration. This led to many runs where our robot would hit the
left or right side of the garage section, confidently driving to a waypoint that it thought was closer to
the walls.

What We Would Fix
-----------------

If given more time to develop our robot, we definitely would have implemented both a better calibration 
procedure, as this was a main source of failure later in the course. A fixture to reliably set up romi in the
same orientation every time, or a multi-point calibration would help us trust our location better.

Another goal that we would have pursued would be the implementation of a position tracking sensor, 
possibly with a distance sensor to allow us to update our location from input of known landmarks.
With accurate and reliable positions, we believe that our robot would be able to more consistently reach
each waypoint, allowing us to push our speeds.
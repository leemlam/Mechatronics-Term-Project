Line Sensing Task
=================

Overview
--------

As the course that our robot navigates uses black lines on a white background
to show the path that we must take, our group mounted an 11-Channel, 4mm Pitch, Analog
infrared sensor array on the front of our robot. `Link to Product <https://www.pololu.com/product/4411>`__

.. image:: linesensor.jpg

This sensor array allowed us to take readings of line presence, weight, and position within the 44mm
span of the array. 

Our readings from all 11 channels were normalized from calibration values on the range of 0-1.
This then allowed us to find the centroid and average value of the entire array, giving us line 
position and magnitude of presence. If the average value of the normalized sensor readings was below
the threshold of 15% (experimentally determined), yaw rate requests would not be sent, causing the 
robot to carry on in a straight line.


The line sensor centroid readings and average value are calculated as shown below::

    #find centroid and weight
        total = 0
        weighted = 0
        for i in range(self.num_chan):
            total+=self.norm_vals[i]
            weighted+=self.norm_vals[i]*(i-((self.num_chan-1)/2))*(spacing)
        self.centroid = -weighted / total
        return self.centroid, self.average

Calibration Process
-------------------

In order to set calibration values for white and black, our robot was placed over a pure white
and pure black sheet, recording individual sensor values. These calibration values were recorded
for future use, allowing use of the line sensor between resets as long as lighting conditions between
runs remained consistent. The values used for our final test are as follows::

    self.black_cal=[3460.3,3192.7,3295.1,3134.9,2442.8,2336.0,2034.7,2531.2,1555.3,1919.8,2838.7]
    self.white_cal=[1806.9,1514.5,1787.5,1505.5,1124.1,625.9,483.7,804.1,332.0,664.9,1461.6]

While all sensor channels were pointing at the same surface, their individual values varied significantly
between eachother. Without calibration, the overall centroid would be wildly off.

Next, to find the normalized values of each of these channels, the following code was run, comparing the channel
reading to its calibration value on both white and black.::

    #interpolate with raw vals, white cal, and black cal
    for i in range(self.num_chan):
    self.norm_vals[i] = (self.avg_vals[i] - self.white_cal[i]) / (self.black_cal[i]-self.white_cal[i])


Yaw Rate Request
----------------

After finding the measured centroid position from our line sensor, we wanted our robot to be able to path toward
this consistently, quickly, and without too much oscillatory behavior. 
This is where our line-sensor :doc:`PID` was involved. We used our PID class to tune yaw rate requests from our
centroid position, only needing position and integral control to achieve reliable responses.


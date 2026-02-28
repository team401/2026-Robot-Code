
We need to add a feature to the settings_gui application that will help us
with in-shop shooter tuning.  The FRC robot has a shooting mechanism that will shoot 
yellow balls into a target called a "hub".
This should be added as a new tab "Shot Map Tuning"

The goal is to facilitate a user collecting a series of data points consisting of 
- distance
- shooter rpm
- hood angle in degree
- flight time

1. Distance will be computed by getting the robot's odometry (use FRC nettables
   for that) and taking the 2d-projected distance to the center of the hub.
   (the x, y coordinates of the hub center will be entered as a constant, initialized
   to 4.02844, 4.00050)

2. Shooter rpm will also be read from net tables (use the average of
    velocityRadiansPerSecond from Shooter/LeadMotorInputs and Shooter/FollowerMotorInputs )

3. Hood angle: get this from Hood/input - positionRadians

4. flight time. We will obtain the flight time from videos that are recorded by the app.

(Make sure that the nettables can connect to simulation on localhost or a robot
using standard FRC conventions). Team number is in .wpilib/wpilib_preferences.json

Implement a UI that provides the user with a "start" and "stop" button.
When start is hit, it should start recording video in WebM format until stop
is hit. It should also record distance + shooter rpm + hood angle at the point
in time when start is pressed.

After the user hits stop, the user may decide to either discard this attempt 
or store it.  If it's discarded, all data and the video is discarded.
Otherwise it is kept.

The app needs to keep track only of attempts the user decided to store.
The user should be able to replay the recordings. The recording should be 
augmented with a timestamp if possible and a UI way for the user to advance
frame by frame.  When stopped, the user should be able to mark timestamps 
in the video as either 
- "leaves shooter"
- "hit target" 
A flight time field should be updated to contain the difference between the two
iff both are set.

The user must be able to manage stored attempts - delete them, for instance.

Eventually, this shooter tuning tab will be connected to the existing Shot Maps
tab as follows: the user can add a complete data point to either the hub shot map or the
passing shot map, at their discretion.  (But keep the stores for both separate so that
the user can decide when to add them from one to the other.)

Design a suitable persistent on-filesystem storage for video clips and collected and entered 
data (separate from the existing .json files in the Shot Map tab).

You should implement most of the app in JavaScript in settings_gui, communicating
with the running or simulated robot via nettables.  Use standard browser APIs to access the 
web cameras and record the clips.


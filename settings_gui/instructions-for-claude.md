
You are in a vite/react/typescript project freshly created from the standard template.

Let's build an app in here.

This app will be used to support the display and update of configurations
of a FRC robot, in simulation and actual deployment.  This robot provides 
a number of REST endpoints (GET/PUT).
Look at /home/gback/robotics/218-add-http-put/parameter_tools for how it's 
implemented.

For instance, in simulation
http://localhost:8088/comp/shotmaps
retrieves a result like that shown in the @shotmaps.json file in this directory.
Here, `comp` is known as a deployment configuration and 
`shotmaps` is an end point. (The user needs to be able to select the deployment
configuration from a menu).

In simulation, the address is http://localhost:8088/ but when the robot
is deployed, it should support connecting to http://10.TE.AM.2:8088/ instead.
We're team 401.  The user needs to be able to select whether to connect
to simulation or to the robot.

Create an app that allows us to retrieve various configuration parameters
on the robot.  Let's make the design extensible (and the UI should allow choosing
which set of configuration parameters should be edited); but let's start with
shotmaps.json.

It is expected that the options for each endpoint (such as shotmaps) change
over the time. The initial format is shown in ../src/main/deploy/constants/comp/ShotMaps.json
Towards that end, add a utility script that reads a file such as ../src/main/deploy/constants/comp/ShotMaps.json
and creates a typescript definition (.d.ts) file for it.  This file shall then be
used in the app.

On the app page for the shotmap configuration, the user needs to be able to
add, remove, and edit both hubDataPoints and passDataPoints.

Since the app is for internal use, we expect it to be mostly used in development mode.
It should be primarily client-side + SPA; no SSR, and it should have a proxy that forwards
requests to the robot's REST endpoints.

It should support a Sun and a Moon theme and use the material UI library (mui).

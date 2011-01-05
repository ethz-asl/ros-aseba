asebaros is a bridge between [ASEBA] and [ROS].
It allows to load source code, inspect the network structure, read and write variables, and send and receive events from [ROS].
It maps the [ASEBA] named events to [ROS] topics in a dynamic way, when loading source code.
asebaros will compile on platforms supported by [ROS].

Compilation
-----------

Get the [ethzasl_aseba] stack.
Make sure that it is included in your `ROS_PACKAGE_PATH`.

Then, to make sure that you have the required development libraries installed, execute:
	rosdep satisfy asebaros

Finally, to compilet asebaros, execute:
	rosmake asebaros

[ASEBA]: http://mobots.epfl.ch/aseba.html
[ROS]: http://www.ros.org
[ethzasl_aseba]: http://www.ros.org/wiki/ethzasl_aseba


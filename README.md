asebaros is a bridge between [ASEBA] and [ROS].
It allows to load source code, inspect the network structure, read and write variables, and send and receive events from [ROS].
It maps the [ASEBA] named events to [ROS] topics in a dynamic way, when loading source code.

Compilation
-----------

Considering that [ASEBA], [dashel] and [ROS] are correctly installed, just type:

	make

in the source tree to build asebaros.
If something goes wrong, make sur that both [ASEBA] and [dashel] are installed.
You can find packages for Ubuntu Lucid and Maverick on my [PPA].
Moreover, make sure that the directory of asebaros' source code is included in `ROS_PACKAGE_PATH`.

[ASEBA]: http://mobots.epfl.ch/aseba.html
[ROS]: http://www.ros.org
[dashel]: http://gna.org/projects/dashel
[PPA]: http://launchpad.net/~stephane.magnenat

## RIDE

An IDE for [ROS](http://ros.org/) that runs in the web browser and edits the graph of nodes while it's running. This is an independent study project in progress and is not yet ready to use. To run ride, run the following commands in a new terminal and visit [http://localhost:8000/](http://localhost:8000/).

    rosmake ride
    roslaunch ride ride.launch

## Notes

* RIDE unfortunately can't use the official [rosbridge](http://www.ros.org/wiki/rosbridge) package because that one drops messages by design, and RIDE assumes the messages it sends won't be dropped. RIDE includes a modified rosbridge node that can be configured to not drop messages.
* This is not secure! ROS is a complex system and there is a high probability of remote code execution. Do not attempt to make RIDE available over the Internet.

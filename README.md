# Swarm Behaviors Library

The swarm behaviors library contains implementations of swarm algorithms. It is part of the [swarm library](https://github.com/topics/swarm-library).

![Behavior Library Structure](library_structure.png)

## Getting Started
The behavior library is based on the latest ROS long-term support release [ROS Kinetic Kame](https://wiki.ros.org/kinetic/). Newer versions may also work.

To run swarm behaviors of this library, the [swarm functions library](https://github.com/cpswarm/swarm_functions) and the abstraction library are required. The abstraction library consists of three sub-libraries:
* [hardware functions](https://github.com/cpswarm/hardware_functions)
* [sensing and actuation](https://github.com/cpswarm/sensing_actuation)
* hardware drivers

The communication between CPSs is based on the [CPSwarm Communication Library](https://github.com/cpswarm/swarmio).

Furthermore, the [cpswarm_msgs](https://github.com/cpswarm/cpswarm_msgs/) are required by most packages in this library.

For detailed usage instructions, please refer to the individual ROS packages in this repository.

## Contributing
Contributions are welcome. 

Please fork, make your changes, and submit a pull request. For major changes, please open an issue first and discuss it with the other authors.

## Affiliation
![CPSwarm](https://github.com/cpswarm/template/raw/master/cpswarm.png)
This work is supported by the European Commission through the [CPSwarm H2020 project](https://cpswarm.eu) under grant no. 731946.

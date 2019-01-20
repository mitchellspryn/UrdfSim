# Construct an XML file

## Overview
The [Univeral Robot Description File](http://wiki.ros.org/urdf) (URDF) is an XML file that describes the geometry and kinematics of the robot to be simulated. With this file, it is possible to describe an arbitrary robot to be simulated inside of Unreal. The structure of the XML file and the meaning of each element node is described in detail [on this page](http://wiki.ros.org/urdf/XML). Once this file is constructed, it can be used with AirSim as described in [AirSim Settings for URDF simulation](UrdfSettings.md). Full examples of XML files can be found in the examples folder at Examples/UrdfBot. 

## Deviations from the standard
Although we attempt to retain compatibility with the URDF specification, there are some deviations that must be made due to the internal workings of the Unreal engine:

* Only <joint>, <link>, and <material> nodes and their properties are supported. Additional nodes types (such as <actuator>) will be ignored. All units are SI (M / KG / S).
* XAcro is not supported.
* <collision> is only supported for links that have a visual geometry of type <mesh>. That is, for box, cylinder, and spherical nodes, the collision boundary will be the same as the visual boundary. Those nodes should not have a <collision> node.
* For <joint>, the sub-nodes <mimic> and <safety_controller> are not supported. They are currently parsed, but their values are unused.
* For <link>, the <material> sub-node should have a "name" attribute that is the full path to an unreal material. If you are using the compiled binaries, the full list of supported materials is [here](). The <color> element is not suported, and will be ignored.
* If using a custom mesh for a <link> node (via the <mesh> sub-node in either the <visual> or <geometry> sub-node), be sure to read the notes in [Using Custom Meshes in a Urdf Bot](CustomMesh.md).
* For some <joint> types, one can create a <limit> tag to allow it to be controlled via positional or velocity offsets. This is useful for emulating devices such as linear actuators. The type of control depends on the type of joint:
    * For a joint of type <prismatic>, specifying a <limit> will allow control of motion along the single linear axis. This is similar to a linear actuator. A signal of '0' will correspond to an offset of 'lower', and a signal of '1' will correspond to an offset of 'upper.' The 'velocity' field controls the maximum rate of motion, and the 'effort' controls the maximum amount of force that can be applied by the joint.
    * For a joint of type <revolute>, specifying a <limit> will allow control of rotation around the specified axis. This is similar to a servo. A signal of '0' will correspond to an angular offset of 'lower', and a signal of '1' will correspond to an offset of 'upper'. The 'velocity' field controls the maximum rate of motion, and the 'effort' controls the maximum amount of force that can be applied by the joint.
    * For a joint of type <continuous>, specifying a <limit> will allow control of angular velocity around the specified axis. This is similar to a motor. A signal of '0' will correspond to an angular velocity of 'lower', and a signal of '1' will correspond to an angular velocity of 'upper'. The 'velocity' field is ignored, and the 'effort' controls the maximum amount of force that can be applied by the joint.
    * For all other joint types, this property is ignored.
* For planar joints, the limits actually represent a circle rather than a box. This is a PhysX limitation.
    
## Tips and tricks
Some tips in order to make the authoring experience easier
* Try to simplify your XML as much as possible to simulate only the critical components of your bot. The more pieces there are to simulate, the slower the simulation will run. 
* The PhysX engine does not deal well with long chains of fixed constraints - this can lead to odd simulation artefacts like oscillations. To overcome this, consider using a custom mesh for a component composed of many fixed constraints rather than using many separate links.
* Use [debug mode](UrdfDebugMode.md) when initially constructing the bot to ensure that the coordinate transformations are as expected.
* Sometimes, it may be useful to add additional constraints to the network in order to improve stability. For example, consider three bars arranged in an A shape. Although only two fixed constraints are needed to fully constrain the system, it may be useful to add a third constraint to help the solver stabilize. 
* When simulating inside the unreal editor, make sure the editor window has focus. Otherwise, the physics simulation will lag, and the bot will be unstable.
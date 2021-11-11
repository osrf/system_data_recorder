# System Data Recorder (SDR)


A lifecycle node and executable for recording topic data to a rosbag2 bag, while simultaneously copying the split bag files to another location as each bag file is completed.
This is useful, for example, to copy bag data files to an external disc during recording as each data file is completed, rather than waiting until all data is recorded before copying the entire bag at once (an operation that can take a significant time if the bag is large).

The copying function requires that a maximum file size for bag files be enabled.
Otherwise no splitting will be performed and the files will not be copied until recording is terminated.


## Compilation

The SDR requires two features not yet available in the rosbag2 main branch or binary releases.
You will need to compile rosbag2 from source, applying the following two pull requests to the source prior to compiling it.

1. [Expose the QoS object wrapper](https://github.com/ros2/rosbag2/pull/910)
2. [Notification of significant events during bag recording and playback](https://github.com/ros2/rosbag2/pull/908)

Create a `colcon` workspace with the [SDR source code](https://github.com/osrf/system_data_recorder) in it, and compile the workspace.


## Use

### Lifecycle management

Run the executable to start the node.

   ros2 run system_data_recorder system_data_recorder

In a separate terminal, use the lifecycle manager to configure and activate the node.

   ros2 lifecycle set sdr configure
   ros2 lifecycle set sdr activate

This will enable recording of data to the bag.
To pause recording, deactivate the node.

   ros2 lifecycle set sdr deactivate

From here, recording can be resumed by re-activating the node, or recording can be terminated by cleaning up the node.

    ros2 lifecycle set sdr cleanup

Once cleaned up, the node will copy the final files of the bag, as well as any metadata, to the backup destination.

### Configuring

The SDR is configured by the arguments passed to the node's constructor.
These are not currently exposed as command line arguments or ROS parameters, so they must be changed in the source code.
See the constructor's documentation block for the possible parameters.

By default, the SDR will:

- Record from the `/chatter` topic, expecting `std_msgs/msg/String` data.
- Record to a bag named `test_bag`.
- Copy bag files to the directory `copied_bag/test_bag`.
- Split bag files every 100,000 bytes.


## Lifecycle transition behaviours

### on_configure

In the on_configure state, the node sets up the rosbag2 infrastructure for
recording by creating a writer and opening the storage. It also sets up the
worker thread that will copy files in parallel to the recording of data. A
callback is registered with the writer so that the node will get informed each
time a new file is created in the bag.

### on_activate

In the on_activate transition, the node simply notifies the worker thread that
it is recording. Data will be written to the bag automatically because the
state changes to `Active`.

### on_deactivate

In the on_deactivate transition, the node simply notifies the worker thread
that it is paused. Data will not be written to the bag because the state
changes to `Inactive`.

### on_cleanup

When cleaning up, the node needs to stop recording, stop receiving data (i.e.
unsubscribe from topics), and ensure that the final files of the bag (the last
data file and the metadata file, which gets written when the writer object
destructs) are copied to the destination directory.

### on_shutdown

If not already performed, the shutdown transition performs the same functions
as the cleanup transition.

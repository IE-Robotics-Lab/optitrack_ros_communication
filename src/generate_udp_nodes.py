#!/usr/bin/env python3

import rospy
import roslaunch

def main():
    rospy.init_node('generate_udp_nodes', anonymous=True)
    
    number_of_nodes = rospy.get_param('~number_of_nodes', 2)
    base_id = rospy.get_param('~base_id', 0)
    ip = rospy.get_param('~ip', '10.205.3.224')
    base_port = rospy.get_param('~base_port', 9876)

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    for i in range(base_id, base_id + number_of_nodes):
        node = roslaunch.core.Node(
            package='optitrack_ros_communication',
            node_type='optitrack_data_handler.py',
            name=f'optitrack_data_handler_{i}',
            output='screen',
            namespace='optitrack',
            args=f'_host:={ip} _port:={base_port + i} _body:=umh_{i}'
        )
        launch.launch(node)

    rospy.spin()

if __name__ == '__main__':
    main()

# custom_interfaces
 basic code in ROS 2 writing an action server and the action client. However, the service request is received by subscribing a topic. If the topic is published at higher rate, it should be able to abort the previous goal and send a new one. The action server should wait for 5 seconds before succeeding and any abortion within 5 seconds should be properly handled

# COLAV Gateway
COLAV Gateway: A UDP-to-ROS bridge for mission requests, agent configurations obstacles updates and controller feedback.
Colav gateway is a ros-based application that listens for incominb UDP messages on predefined ports for: 
- Mission Requests
- Agent Configuration updates
- Obstacle Updates

THe gateway parses incoming JSON data and publishes it as ROS messages to corresponding ROS topics, enabling seamless integration between external systems and the COLAV framework.

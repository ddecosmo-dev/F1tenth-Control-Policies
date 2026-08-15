Lab 2 Demo link
https://youtu.be/wEX9hsWWmAQ

### NOTE
The safety_node directory has it's own launch file. This allows for the simulation to be intiailized with safety node running automatically. To start this, run docker compose to start the docker container. Then, in a seperate terminal run 'ros2 launch safety_node safety_node_launch.py'. This will start an F1 Tenth gym session with AEB running automatically. 
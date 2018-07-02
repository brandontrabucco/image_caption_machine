<<<<<<< HEAD
# ros-image-captioner
A simple ros package for captioning images streamed from a camera.
=======
# image_caption_machine
A ROS package that integrates a camera, an online image caption service, and Amazon Alexa.

# Setup

To use this package, I will assume you are using Ubuntu 14.04 LTS or a similar distribution.

Install ROS onto your computer:

http://wiki.ros.org/ROS/Installation

Then create a workspace to clone the project.

```
cd ~/
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```

Clone the repository into your workspace

```
cd ~/catkin_ws/src
git clone http://github.com/brandontrabucco/image_caption_machine
```

Make your workspace.

```
cd ~/catkin_ws
catkin_make clean
catkin_make
catkin_make install
```

And install the package dependencies.

```
pip install numpy
pip install tensorflow
pip install opencv-python
```

You should be good to go.

# Running

There are many nodes and service that must be active in order for this package to run.

You must initiate the roscore

```
roscore
```

Then you must activate the rgb camera simulator (unless you have a real camera streaming to this topic).

```
cd ~/catkin_ws
source devel/setup.sh
rosrun image_caption_machine simulate_camera.py
```

A display window should pop open to confirm this is running correctly.

You must also start the caption service and client, specifying the server url of your image captioning model, which has code in the following locations.

http://github.com/brandontrabucco/im2txt

http://github.com/brandontrabucco/image_caption_app

Run the caption service.

```
cd ~/catkin_ws
source devel/setup.sh
rosparam set model_server_url "http://ec2-xx-xx-xxx-xxx.us-xxxx-x.compute.amazonaws.com"
rosrun image_caption_machine caption_service.py
```

And also run the caption client that connects to the service.

```
cd ~/catkin_ws
source devel/setup.sh
rosparam set model_server_url "http://ec2-xx-xx-xxx-xxx.us-xxxx-x.compute.amazonaws.com"
rosrun image_caption_machine caption_client.py
```

Finally, we launch the alexa server that listens for specific user statements.

```
cd ~/catkin_ws/src/image_caption_machine/scripts
python alexa_server.py
```

And also run ngrok, which forwards the localhost port the server is listening to.

```
cd ~/catkin_ws/src/image_caption_machine/scripts
./ngrok http 5000
```

Note the subdomain of ngrok.io that your app is being forwarded to.

# Alexa

Assuming you have an Amazon Echo Dot, which is already synced to your Amazon developer account. You can proceed to the following steps. Otherwise see this link.

https://www.amazon.com/gp/help/customer/display.html/ref=aw?ie=UTF8&nodeId=201994280

Open your skills dashboard from the Alexa console.

Create a new skill, and select the custom skill option.

Upload the alexa.json interaction model to your new skill via the json editor tab.

Save and build your new skill.

Finally, on the endpoint tab, select the HTTPS option, and enter the following in the Default field.

```
HTTPS: https://<your_given_id>.ngrok.io
SSL: My development endpoint is a subdomain ...
```

With this completed, you should be able to interact with alexa and start the custom skill.

# AWS EC2 Model Server

I have deployed my image captioning model to a Ubuntu AMI from Amazon Web Services. A flask server is listening on the public domain for incoming POST requests  that contain the bytes of an encoded jpeg image. See the next link for the implementation.

https://github.com/brandontrabucco/image_caption_app

https://github.com/brandontrabucco/im2txt
>>>>>>> 513eb8e5cf844b3d758892a3cf698907a45c2f32

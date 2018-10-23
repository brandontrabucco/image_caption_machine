# Image Caption Machine

Hi there, thanks for viewing my code!

This repository is the code for my 2018 [Robotics Intitute Summer Scholars (R.I.S.S.)](https://riss.ri.cmu.edu/) internship project at Carnegie Mellon University. I worked under Dr. Jean Oh, and Dr. Ralph Hollis, Ph D. candidate Roberto Shu, and Masters student Junjiao Tian. The focus of my project was Detailed Image Captioning.

## Detailed Image Captioning

Modern approaches to Image Captioning use a neural network architecture called [Show And Tell](https://arxiv.org/abs/1609.06647).

![The Show And Tell Model Architecture](https://preview.ibb.co/b7Z0Qq/SAT.png)

The Show And Tell Model performs a translation from Images into Captions. The first part of the model, namely the *encoder*, is a Convolutional Neural Network. This project uses the [Inception V3 Convolutional Neural Network](https://arxiv.org/abs/1512.00567) for the encoder. The second part of the model, namely the *decoder*, is a Recurrent Neural Network that performs sequence generation. This project uses [Long Short Term Memory](https://www.bioinf.jku.at/publications/older/2604.pdf) to resolve the vanishing gradient problem.

While Show And Tell learns freeform image captions for many different types of images, the algorithm does not produce captions with many accurate visual details. This project proposed a method to correct this.

## Setting Up

This repository was designed to run on a mobile robot using the [Robot Operating System](http://www.ros.org/), controlled with human speech using [Amazon Alexa](https://developer.amazon.com/alexa-skills-kit) custom skills. The Image Captioning model is intended to run on a remote [AWS EC2](https://aws.amazon.com/ec2/) instance as a web service.

### Installing ROS

This project was developed using ROS Indigo on Ubuntu 14.04 LTS. Please install [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) and proceed to [setup and configure your ROS workspace](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) to ensure your installation has succeeded.

To create an empty workspace named `my_workspace` in your home directory.

```
ubuntu@server.name:~$ cd ~/
ubuntu@server.name:~$ mkdir -p ~/my_workspace/src
ubuntu@server.name:~$ cd ~/my_workspace/
ubuntu@server.name:~$ catkin_make
```

Congratulations, you have installed ROS Indigo, and are one step closer to running your very own Image Caption Machine.

### Configuring Alexa

This project was built to use an Echo Dot Generation 2, but presumably, any Alexa enabled device would be fine. First, you must [connect your echo dot to the internet](https://www.amazon.com/gp/help/customer/display.html?nodeId=202011800) to interact with skills. Second, you must [register your echo dot to your account](https://www.amazon.com/gp/help/customer/display.html?nodeId=201994280) so that you can use custom skills. 

You should now be able to say `Alexa, what time is it?` and Alexa will respond. The next step in this process is to create the custom Alexa skill that will enable you to use the Image Caption Machine.

1. Log into the [Alexa Developer Console](https://developer.amazon.com/alexa/console/ask) and select the button `Create skill` in order to begin your custom skill. Name your skill, for example *my_skill*, select the *custom* model type, and click the button `Create skill`. Choose the *Start from scratch* template and select the button `Choose`. 

2. With the skill created, click the `JSON Editor` tab in the left menu pane. In the JSON Editor tool, upload the *interaction_model.json* file, and then press the button `Save Model` first and the button `Build Model` second. You have now successfully built your custom skill. 

3. To enable testying on your echo dot, select the button `Endpoint` from the left menu pane, and select the *HTTPS* endpoint type. This project uses *ngrok* to forward a localhost flask server running in ROS to a public HTTPS address accessible by the Alexa skill. In the *Default Region* text box, enter the *ngrok* domain name, for example *https://your-ngrok-subdomain.ngrok.io* and select the SSL option *My development endpoint is a sub-domain ...*. Select the button `Save Endpoints`. 

4. You are now ready to enable testing: select the `Test` tab at the top menu bar. At the top of the screen, toggle on the `Test is disabled for this skill` switch. You are now all set up to use Alexa to control the Image Caption Machine.

### Configuring AWS

The Image Caption model is meant to run on a remote AWS EC2 instance. 

1. Log into the [AWS EC2 Console](https://us-east-2.console.aws.amazon.com) and on the EC2 Dashboard, select Launch Instance, and select the *Deep Learning AMI (Ubuntu) Version 16.0* or an equivalent instance image. Follow the instructions on AWS to initialize and connect to your instance. 

2. Once your instance is setup, follow any online tutorial to [Set up Flask and Apache on AWS EC2 Instance](https://vishnut.me/blog/ec2-flask-apache-setup.html). With your server running, copy the web server from the [Project Flask Server](http://github.com/brandontrabucco/image_caption_app) into your instance. 

3. Finally, download the release of [Image Caption Model](https://github.com/brandontrabucco/im2txt/releases/tag/v2.0.0) and install as a pip package (note that you must provide a pretrained model checkpoint yourself).

### Setting Up The Repo

... 

### Running The Repo

... 

### Acknowledgements

... 

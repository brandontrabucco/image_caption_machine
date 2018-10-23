# Image Caption Machine

Hi there, thanks for viewing my code!

This repository is the code for my 2018 [Robotics Intitute Summer Scholars (R.I.S.S.)](https://riss.ri.cmu.edu/) internship project at Carnegie Mellon University. I worked under Dr. Jean Oh, and Dr. Ralph Hollis, Ph D. candidate Roberto Shu, and Masters student Junjiao Tian. The focus of my project was Detailed Image Captioning.

## Detailed Image Captioning

Modern approaches to Image Captioning use a neural network architecture called [Show And Tell](https://arxiv.org/abs/1609.06647).

![The Show And Tell Model Architecture](https://preview.ibb.co/b7Z0Qq/SAT.png)

The Show And Tell Model performs a translation from Images into Captions. The first part of the model, namely the *encoder*, is a Convolutional Neural Network. I chose to use the [Inception V3 Convolutional Neural Network](https://arxiv.org/abs/1512.00567) for the encoder. The second part of the model, namely the *decoder*, is a Recurrent Neural Network that performs sequence generation. I chose to use [Long Short Term Memory](https://www.bioinf.jku.at/publications/older/2604.pdf) to resolve the vanishing gradient problem.

While Show And Tell learns freeform image captions for many different types of images, the algorithm does not produce captions with many accurate visual details. I proposed a method to correct this in my project.

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

You should now be able to say `Alexa, what time is it?` and Alexa will respond. The next step in this process is to create the custom Alexa skill that will enable you to use the Image Caption Machine. This skill must be testing enabled.

Log into the [Alexa Skill Developer Console](https://developer.amazon.com/alexa-skills-kit) and start a custom skill. Upload the file `alexa.json` into the site. Head to the testing panel, and enable skill testing on your account devices.

### Configuring AWS

... 

### Setting Up The Repo

... 

### Running The Repo

... 

### Acknowledgements

... 

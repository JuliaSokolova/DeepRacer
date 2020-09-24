# Driven by Reinforcement Learning: Self-Driving Car on AWS 

This project was made as my capstone for Galvanize Data Science bootcamp.
My goal was to use reinforcement learning and AWS DeepRacer service to train a virtual racing car drive itself.

<p align="center"><img width=60% src=https://github.com/JuliaSokolova/DeepRacer/blob/master/img/umka.png> 

## System Architecture

- Interface: AWS DeepRacer
- Environment: AWS RoboMaker
- Simulation: AWS SageMaker
- Storage: AWS s3
- Model: 
    - Convolutional Neural Network with 3 layers
    - 20 possible actions (changing speed and steering angle)
    - Customizable reward function with 23 params
    
## AWS DeepRacer

DeepRacer is a web interface for reinforcement learning environment that has 4 main modules: agent, racing track, action space and reward function.
You can train a model in the web interface, or run pre-designed jupyther notebooks that allow you to run the simulations manually on your AWS SageMaker account.


<p align="center"><img width=60% src=https://github.com/JuliaSokolova/DeepRacer/blob/master/img/reinforcement_learning.png> 



### The car (agent)
The agent is a scaled racing car built on Ubunty. And yes, you can buy a physical model and train it to drive around your house. 

- 1/18 scale four wheel drive car
- Front-mounted camera
- Ubuntu
- Battery-powered
- WiFi
- Machine learning enabled

### Racing track (the enviroment)

DeepRacer has a variety of tracks to train the models. Each track is built in a two-dimensional coordinate system and has parameters like track width and waypoints.

<p align="center"><img width=60% src=https://github.com/JuliaSokolova/DeepRacer/blob/master/img/enviroment.png> 
  
## The challenge

When training, I faced a challenge. My simulations were failing all the time, and soon I discovered that pre-built system is not working any more, so I had to find a back door to deploy my code into DeepRacer.

<p align="center"><img width=60% src=https://github.com/JuliaSokolova/DeepRacer/blob/master/img/forum-message-error-full.png>
  
 To overcome the issue, I did the following:
- download the model to the local  machine ->                           
- modify code for reward function & action space -> 
- export to s3 ->     
- load into DeepRacer interfacea, clone the model, and train it

## Trainign the model

For my model, I used:
- Race line optimizer
- Action space calculator
- Log analysiz

You can find more info in corresponding folders in this repo.

## Results

Optimized track line:

<p align="center"><img width=50% src=https://github.com/JuliaSokolova/DeepRacer/blob/master/img/optimized-track-lane.png>
    
Optimized action space:

<p align="center"><img width=50% src=https://github.com/JuliaSokolova/DeepRacer/blob/master/img/action-space.png>
    
Car training video:

[![DeepRacer: training Umka](https://img.youtube.com/vi/30daMObilmY/0.jpg)](https://youtu.be/30daMObilmY "DeepRacer: training Umka")


Race results:
- best time 12.689 s (winner - 7.668 s)
- rank: 270 out of 670 cars



    

    

    
    



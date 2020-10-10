# jetbot_sim

This repository contains the packages required for lane following and object avoidance simulation of jetbot.

* For running the **lane following** simulation, follow these steps:
    * Clone the jetbot_sim package.    
    * For uploading the track in gazebo, open the file **world/lane_follower.world** and then in the lines **131** & **151** change the path to the **Part1.stl** file according to your system.    
    * Build the package.    
    * Run the following commands:    
      * **roslaunch jetbot_sim gazebo.launch**
      * **roslaunch jetbot_sim lane_following.launch**
    * [Demo Video](https://www.youtube.com/watch?v=8GVjcdUdv4M&feature=youtu.be)

* For running the **object avoidance using neural networks** simulation, follow these steps:
   * Install **pytorch** on your system. (Intructions can be found from this link : https://www.liquidweb.com/kb/how-to-install-pytorch-on-ubuntu/)
   * If you want to run the model on your system's gpu then also install **cuda**.
   * Clone the jetbot_sim package.
   * Download the pretrained model file from this link: **https://drive.google.com/file/d/1UsRax8bR3R-e-0-80KfH2zAt-IyRPtnW/view**
   * Move the download pretrained model file to **jetbot_sim/scripts** folder.
   * For running the model on gpu uncomment the lines **29-31 & 84** and comment out the line **28** in **scripts/object_avoidance.py** file.
   * For uploading the model in script, open the file **scripts/object_avoidance.py** and then in the lines **28** or **29** change the path to the **best_model.pth** file according to your system.
   * Build the package.
   * Run the following commands:
     * **roslaunch jetbot_sim object_avoidance_config.launch**
     * **rosrun jetbot_sim object_avoidance.py**

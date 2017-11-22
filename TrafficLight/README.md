Traffic Light Detection
===================


This document covers the part related to the traffic light detection system which is what I (Sachit Vithaldas) contributed as part of this project.

Core Idea
-------------

I went through hell in order to get this to work. After reading about several approaches it seemed to me like using the tensor flow object detection API was the best way to solve this problem. This is because of

 1.  The availability of pre trained models (I can just apply transfer learning rather than having to train for several days)
 2.  Built in augmentation steps that account for bounding boxes
 3.  Availability of multiple models (SSD, RCNN, Faster-RCNN)


Implementation
-------------------

Given the above points I thought this would be quite a straight forward process once I have some good dataset. I initially tried the bosch traffic dataset with SSD but was getting absolutely nowhere. This never seemed to converge and after a number of days, I realized I needed a different approach. That's when I used the Faster RCNN and that seemed to work pretty well at a resolution of 400x300. I was able to detect traffic signs pretty well - each taking about 20ms to execute.


Data Collection
-------------------
I initially attempted to use the Bosch dataset but due to several difficulties with that - I found some datasets on the forums and went with those instead. Unfortunately those datasets were really small and I decided to create my own. I used LabelImg for the same and tagged a couple of images by myself and appended it to the dataset I had obtained. The dataset can be downloaded from [here](https://drive.google.com/file/d/1oe0jd9jGms198LQ2_u5VBl9S7WKbL4av/view?usp=sharing). Please extract it inside the TrafficLight folder. 

Augmentation of Data
-------------
I initially had a lot of trouble trying to work with tensor flows augmentation system. I started using the python library called [imgaug](https://github.com/aleju/imgaug) to try and augment some data. However while I was doing this, I realized why I had been doing the tensor flow augmentations all wrong and fixed this by providing the parameters to some of the augmentation schemes which I was not providing earlier. If you have a look at the config files, you will notice that I have used the following augmentations:

 1. Horizontal Flip
 2. Adjust Saturation
 3. Random Brightness Adjustments
 4. Random Cropping to Aspect Ratio

Deployment
--------------------
Once the network was trained, I quantized the weights so that my model was smaller and could run with less ram and that slight bit faster.

Image Detection
--------------------
After the system was trained, I made the classifier output it's result on the los topic '**/processed_image**' which I could monitor in rviz or rqt_image_view and this helped me tremendously when trying to debug.

Demo
--------------------
You can have a look at the demo video [here](https://youtu.be/AeTWVj-u7h0).  This shows the working of the system in the simulator as well as when playing the rosbag video.

Conclutions
--------------------
To be honest I spent most of my time trying to work with the API rather than actually working here (6-7 whole days). It was frustrating in the end since I never got SSD to work but was quite pleased with Faster RCNN. I also read so many papers on this field which was a great learning experience.

All in all, this was a really fun project. I'm glad to have completed it and I am looking forward to my graduation from the Udacity program.

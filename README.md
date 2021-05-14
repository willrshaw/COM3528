# COM3528: Cognitive and Biomimetic Robotics
## Group 8: Landmark recognition and navigation with AprilTags
**About this repository** This GitHub repository hosts group 8's project for
COM3528. We researched, designed and developed a system for the robotics
platform [MiRo](http://consequentialrobotics.com/) which uses
 [AprilTags](https://april.eecs.umich.edu/software/apriltag) to identify rooms
 in a building, generate a map of these rooms and then navigate. We would like
 to especially thank [koide3](https://github.com/koide3) and
  [AprilRobotics](https://github.com/AprilRobotics/apriltag-imgs/tree/b53998d83ce4cc4f543eec28a66cffb6372ca73e)
   for the models of the AprilTags and the Images which they are based off.
   Below is the abstract for the [paper](./Report.pdf) written to document this project.
### Abstract
Social robots such as MiRo have an important role to play in the future; they can be a very useful
 tool in care homes as they can engage with people of need at an emotional level and provide entertainment. This
 project explores the use of AprilTags in order for a MiRo robot to be able to map its surroundings and navigate to
 a given target room. To achieve this, we have implemented mapping, navigation and AprilTag detection techniques
 that takes inspiration from fiducial markers found in nature. The project has demonstrated the viability of robots
 such as MiRo at performing simple room mapping and navigation procedures in an environment such as a care
 home. The report concludes that the MiRo robot has successfully fulfilled the aims of this project, whilst detailing
 areas for future research.

### Details on using this repo.
We developed MiRo to work in the worlds called [2roomsmalltagstarget](./GazeboWorlds/2roomsmalltagstarget) and
[3roomsmalltagstarget](./GazeboWorlds/3roomsmalltagstarget).

Please keep these world files in the same folder as AprilTag36_11_000{00...15}s (with the s),  
otherwise the AprilTags will not load in the world.  

To run the [Script](./src/AprilTagRecognition.py), please start up Roscore, the simulator and then type
$ python AprilTagRecognition.py.  

If this isn't working, type this:  

$chmod +x ./AprilTagRecognition.py  

While in the same directory as the file.

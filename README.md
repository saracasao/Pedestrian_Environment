# Framework for Fast Prototyping of Photo-realistic Environments with Multiple Pedestrians

We share a framework built on Unreal Engine and AirSim to easily generate dynamic scenarios with pedestrians. You can download the complete project in [here](https://unizares-my.sharepoint.com/:u:/g/personal/scasao_unizar_es/EU0t68nG85hKsRQp3oqhrU8Bj58UnW1xwzto3GjynvJ7lw?e=C9z7dH). The project includes: 
- Trajectory plugin with different tools to create custom and random trajectories. 
- Set of pedestrian models ready-to-used by simply dragging and dropping them into the Unreal world. 
- Downtown West photo-realistic map that can be found for free download in the Unreal Engine Launcher [weblink](https://www.unrealengine.com/marketplace/en-US/product/6bb93c7515e148a1a0a0ec263db67d5b). In order to work with lighter maps, we create two copies: 
  1. Font map (one of the streets has been removed). This map includes a simple example of custom trajectories for two pedestrian models.
  2. Street map (font area has been remove). This map includes a simple example of creating continuous paths. 
  
  
 ## Requirements
  
 The project has been tested in Ubuntu 20.04. 
 - Unreal Engine 4.27
 - AirSim 1.7 
  
 ## Common issues
 
 If a compilation error message appears when you open the project because of a different version of Unreal Engine, the following solution given in [here](https://github.com/microsoft/AirSim/issues/4535) worked for me with Blocks.uproject from AirSim: 
 1. Cleaned the PedestrianEnv project ```cd ~/PedestrianEnv && ./clean.sh```
 2. Installed mono-complete which lets you run .exe files on Linux
 3. Made some symlinks: ```sudo ln -s /usr/bin/mono /bin/mono && sudo ln -s /usr/lib/mono /lib/mono```
 4. Ran the following command to rebuild the Blocks project: ```~/UnrealEngine-4.27/Engine/Binaries/ThirdParty/Mono/Linux/bin/mono ~/UnrealEngine-4.27/Engine/Binaries/DotNET/UnrealBuildTool.exe Development Linux -Project=/home/scasao/Documents/UnrealProjects/PedestrianEnv/PedestrianEnv.uproject -TargetType=Editor -Progress```
 5. Started UE4Editor from command line: ```~/UnrealEngine-4.27/Engine/Binaries/Linux/UE4Editor```
 6. Clicked More > Browse > /home/scasao/Documents/UnrealProjects/PedestrianEnv/PedestrianEnv.uproject 
 
 ## Tutorials 
[![How to open the project](video_tutorials/open_project.mp4)]
[![How to add AirSim plugin to a new Unreal project](video_tutorials/airsim_plugin.mp4)]
[![How to export tools to another project](video_tutorials/export_tools.mp4)]
[![How to use the tools](video_tutorials/use_tools.mp4)]

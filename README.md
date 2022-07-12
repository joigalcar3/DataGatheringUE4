# DataGatheringUE4
Code for generating drone sensor data under sensor and actuator faults using the AirSim simulator within Unreal Engine 4

## Installation steps

4 software components are required for set-up of the fault detection and identification simulation framework

* Unreal Engine 4
* AirSim
* C++ APIs
* Python APIs

With this instruction manual you should have access to:
* Dedicated AirSim copy with the physics engine of Bebop 2 and the INDI controller. 
For that, please contact José Ignacio ([j.i.dealvearcardenas@student.tudelft.nl](mailto:j.i.dealvearcardenas@student.tudelft.nl)), 
who can provide you with a copy. In the future this software will be available in GitHub.
* Coen de Visser login details for Unreal Engine. For that purpose, 
please contact Coen de Visser ([c.c.devisser@tudelft.nl](mailto:c.c.devisser@tudelft.nl)).

Next, the installation and set-up steps for each of the aforementioned software components will be explained.

### Python and C++ APIs

1. Download [Anaconda](https://www.anaconda.com/) in order to obtain Python and easy access to all scientific libraries.
2. Download [Pycharm (Professional)](https://www.jetbrains.com/pycharm/) as an IDE for Python.
3. Create a Conda environment with Python 3.7.
4. Install the libraries from *AirSim/PythonClient/multirotor/Occupancy_grid/requirements.txt*.
5. Go to ...\anaconda3\envs\NameCondaEnvironment\Lib\site-packages\airsim and substitute the files there
(at least client.py) with the files stored in *AirSim/PythonClient/multirotor/Occupancy_grid/Airsim_lib_mod*. 
NameCondaEnvironment is the name that you gave to your conda environment.
6. Download [Microsoft Visual Studio Community 2019](https://visualstudio.microsoft.com/vs/older-downloads/) (versions 16.9 or 16.11) as IDE for C++.


### Unreal Engine 4 and AirSim

1. Download the [Unreal Engine Launcher](https://www.unrealengine.com/en-US/download) in order to be able to install the UE4 engine and all the required assets.
2. Open the Unreal Engine Launcher with the login details of Coen de Visser.
3. Download Unreal Engine 4.25.4 by clicking in the plus button as pointed in the image below.

![UE4 install](Doc_images/Install_UE4.png)

4. Open a clean empty project by clicking on the 4.25.4 UE4 engine launch, as shown in the image below. 

![New environment](Doc_images/new_env.png)

5. Select "*Games*" project category. Leave it as a "*Blank*" Blueprint project, available for Desktop/Console and maximum quality. 
The only setting you need to change is that you do not want it “*With Starter Content*”, 
so change this option to "*No Starter Content*". Select where you want to save this new project and give it a sensible 
and memorable name. All the settings can be observed in the next image.

![Creation of a project](Doc_images/project_creation.PNG)

6. Add the Next Gen Modular City V3 environment to the new project you created from the Unreal Engine Launcher 
by clicking in “Add to Project”, as shown in the image below. Select the project you just created.

![Add city bundle to your project](Doc_images/add_city_bundle.png)

7. Follow the steps in ["Creating and Setting Up Unreal Environment"](https://microsoft.github.io/AirSim/unreal_custenv/) from the AirSim documentation, 
using the project you created instead of LandscapeMountains, so you can skip steps 1 and 2. 
If you have already opened your new project in UE4, then you can also skip step 3. In step 5, 
it suggests copying the Unreal/Plugins folder into your new environment folder.
Instead, we copy the *Airsim/Unreal/Environments/Blocks/Plugins* folder, 
since it has the built modified AirSim version we want to use. At the end of this step, you should
have the City environment opened from the C++, not from the UE4 launcher.
 
8. In the UE4 editor you should see the “*ModularCity*” folder in the Content Browser tab, as in the image below. 
Open “*Content/ModularCity/maps/ModularCity*” by double clicking on the corresponding icon. Then you will see the loaded
landscape without any building or roads assets.

![City level](Doc_images/city_level.png)
![Load modular city](Doc_images/load_modular_city.png)

9. When you load the "*ModularCity*" landscape, it looks as if the lights are switched off. To solve this problem, go to 
the World Outliner in the top right corner and look for "PostProcessVolumeSky". Then, go to the Details tab located
at the bottom right and search for "Exposure Compensation" in the search tab and tick off this option. Then the
lights will be switched on.

11. Once the ModularCity basic map has loaded, you can load the buildings and roads by loading the 
different levels. In order to see the levels, you can click on Levels within the Window tab next to the
Edit Tab. Then you can load each of the levels one by one by right clicking on each one and selecting
 “Load”. 

![Levels](Doc_images/levels.png)
![Load levels](Doc_images/load_levels.png)

<!---"MC_PRG_Road_Level" is the most computationally expensive level since it is composed of roads created 
 with the Procedural Road Generator (PRG). Hence, instead of being static meshes, they are actors that can be modified.
This possibility of modifying the object slows down the simulation. This can be solved by selecting multiple roads,
right on one of the selected roads and clicking on merge actors. By lumping multiple roads into a single object,
the draw calls to the same material (for instance, the asphalt colour) is reduced from the x number of selected
roads to 1.-->

"MC_PRG_Road_Level" is the most computationally expensive level since it is composed of roads created 
 with the Procedural Road Generator (PRG). Hence, instead of being static meshes, they are actors that can be modified.
This possibility of modifying the object slows down the simulation. This can be solved by moving the PRG level
one folder up such that the world composition does not take it into account. However, you must copy the cross-road
objects from the PRG level to the editable road level, since they do not exist in the second one. The editable road level can be made
more efficient by merging spline meshes actors (not static meshes). The static mesh actors could be merged using the 
batch merging, which will turn them into instanced static meshes that is ultra performant when using the
nanites rendering technology of UE5. 

Additionally, it might be the case that you do not want to have the complete city environment for the FDD
simulation, since part of the FDD pipeline is to retrieve the meshes from UE4 in order to create the 
occupancy map. Having many actors may cause the FDD to crash depending on your machine (laptop/PC) specs. Hence,
you can also move one folder up all those levels that you do not require to use (for instance levels 0, 1, 3, 4, 5)
and you can delete or hide the unnecessary roads from the editable road level.

Such process of merging roads can take up to a day of (monotonous) work. In order to avoid wasting your time, contact
José Ignacio ([j.i.dealvearcardenas@student.tudelft.nl](mailto:j.i.dealvearcardenas@student.tudelft.nl)) who
has already suffered this process and has merged all the PRG roads in order to have smooth simulations. He
could provide you with the modified City environment.

10. Now that you have a working environment launched from C++, you can click "Play" in order to launch the simulation, 
as shown in the image below. Then you should see a drone hovering.

![Load levels](Doc_images/play.png)

11. If you want to launch the data collection, first check whether the settings in user_input.py suit you.
The definition of each of the inputs can be read in the file. Once you are ready, run DataGathering.py.
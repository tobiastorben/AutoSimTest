# AutoSimTest
A Matlab library for Automatic Simulation-based Testing using Gaussian Processes and Temporal Logic.

# Installation
1) Pull/Download this Git repository project to your local PC
2) Add the AutoSimTest folder to the Matlab Path with subfolders
3) [Download](https://sites.google.com/a/asu.edu/s-taliro/s-taliro/download) the S-Taliro dependendy and follow its installation guidelines.
4) If you wish to run the Collision Avoidance examples you have to download a dataset of prerun simulations. The datasets can be downloaded from [here](https://studntnu-my.sharepoint.com/:u:/g/personal/tobiasvt_ntnu_no/ETUPnDWr42hOi9Zu1f46EMEBjelr8u45an01-6Gsc08RcA?e=hPi6jm). Unzip the archive and add the datasets folder to "AutoSimTest/examples/Collision Avoidance".

# Usage
The testing algorithm is implemented in the auto_sim_test.m Matlab function. See "examples/Collision Avoidance/headon_auto_sim_test.m" and "examples/Collision Avoidance/radial_auto_sim_test.m" for examples of usage. This demonstrates the definition of STL requirements, setting of hyper parameters, how to run simulations and return the results in the correct formats as well as visualization and validation.

# Publication
The automatic simulation-based testing algorithm is described in the following paper (Add link when published).

# Support/Contribution
Don't hesitate to contact me, [Tobias Rye Torben](https://www.ntnu.edu/employees/tobias.torben), for questions. Contributions are welcome as suggestions or pull requests.

© Norwegian University of Science and Technologu (NTNU), Department of Marine Technology. The software is developed under the ORCAS Project by Tobias Rye Torben

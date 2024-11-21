# ML-Framework-for-NR-U-MAC-Protocol-Design-Multi-Agent #

## Introduction

The NR

## Features

* Support of a wide range of channel models, including the model based on 3GPP TR 38.901 for frequencies between 0.5 and 100 GHz. Ray tracing and measured traces can also be used.

* Custom PHY and MAC classes supporting the 3GPP NR frame structure and numerologies.

* Custom schedulers for supporting dynamic TDD formats

* Carrier Aggregation at the MAC layer

* Enhancements to the RLC layer with re-segmentation of packets for retransmissions

* Dual Connectivity with LTE base stations, with fast secondary cell handover and channel tracking

* Simulation of core network elements (with also the MME as a real node)

## Installation

Check out [install.md](./docs/install.md) for how to install and setup the NR-AI-MAC module.

## Usage example

You can use the following command to run the training and evaluation with NR-AI-MAC module in different scenarios

1. Training the PPO model for different scenarios

a. CTCE scenario

```bash
cd contrib/ai/examples/nr-ai-mac/use-gym
python3 run_nr_mac.py --agent central
```

b. CTDE scenario

```bash
cd contrib/ai/examples/nr-ai-mac/use-gym
python3 run_nr_mac.py --agent multi
```

c. DTDE scenario

```bash
cd contrib/ai/examples/nr-ai-mac/use-gym
python3 run_nr_mac.py --agent multi --separate_agent_nns
```

2. Inserting trained policies for evaluation 

a. CTCE scenario

```bash
cd contrib/ai/examples/nr-ai-mac/use-gym
python3 evaluation.py --agent central --outputDir "YOUR_OUTPUT_DIR" --rllibDir "YOUR_RAY_CHECKPOINT"
```

b. CTDE scenario

```bash
cd contrib/ai/examples/nr-ai-mac/use-gym
python3 evaluation.py --agent multi --outputDir "YOUR_OUTPUT_DIR" --rllibDir "YOUR_RAY_CHECKPOINT"
```

c. DTDE scenario

```bash
cd contrib/ai/examples/nr-ai-mac/use-gym
python3 evaluation.py --agent multi --separate_agent_nns --outputDir "YOUR_OUTPUT_DIR" --rllibDir "YOUR_RAY_CHECKPOINT"
```

Note : "YOUR_RAY_CHECKPOINT" is by default saved in home/user/ray_results/, e.g. home/user/ray_results/PPO_2024-08-05_23-22-17

3. Debugging possible errors 

a. ns3-ai environemnt path in [environments.py](./contrib/ai/examples/nr-ai-mac/use-gym/environments.py) sometimes needs to be defined explicitly

```bash
self.dummyEnv = Ns3Env(targetName="ns3ai_nrmac_gym",
            ns3Path="YOUR_NS3_PATH", ns3Settings=ns3Settings, envNumber = self.envNumber)
```
            
b. The path to pathloss matrix in [nr-ai-mac.cc](./contrib/ai/examples/nr-ai-mac/use-gym/nr-ai-mac.cc) sometimes need to be defined explicitly

```bash
std::string pathlossDir = "YOUR_NS3_PATH/freespacePL/";
```


## Documentation

The documentation of this module is available at [this link](./src/mmwave/doc/mmwave-doc.md).

## Related modules

In order to implement the NR-AI-MAC module, we make use and modify the following sub-modules :

- nr-u is an ns-3 module for for the simulation of NR-U non-standalone cellular networks. Check [nr-u](https://gitlab.com/cttc-lena/nr-u) for further details.
- ns3-ai is an ns-3 module to enable interaction between ns-3 and Python scripts. Check [ns3-ai](https://github.com/hust-diangroup/ns3-ai) for further details.
- ns-3-vr-app is an implementation of a traffic model for AR/VR applications on ns-3. Check [ns-3-vr-app](https://github.com/signetlabdei/ns-3-vr-app) for further details.

## References 
The following papers describe in detail the features implemented in the mmWave
module:
- [End-to-End Simulation of 5G mmWave Networks](https://ieeexplore.ieee.org/document/8344116/ "comst paper") is a comprehensive tutorial with a detailed description of the whole module. We advise the researchers interested in this module to start reading from this paper;
- [Integration of Carrier Aggregation and Dual Connectivity for the ns-3 mmWave Module](https://arxiv.org/abs/1802.06706 "wns3 2018") describes the Carrier Aggregation implementation;
- [Implementation of A Spatial Channel Model for ns-3](https://arxiv.org/abs/2002.09341 "wns3 2020") describes the integration of the spatial channel model based on the 3GPP specifications TR 38.901 V15.0.0;
- [Performance Comparison of Dual Connectivity and Hard Handover for LTE-5G Tight Integration](https://arxiv.org/abs/1607.05425 "simutools paper") describes the Dual Connectivity feature.

These other papers describe features that were implemented in older releases: 
- [ns-3 Implementation of the 3GPP MIMO Channel Model for Frequency Spectrum above 6 GHz](https://dl.acm.org/citation.cfm?id=3067678 "wns3 2017") describes the implementation of the 3GPP channel model based on TR 38.900;
- [Multi-Sector and Multi-Panel Performance in 5G mmWave Cellular Networks](https://arxiv.org/abs/1808.04905 "globecom2018") describes the multi-sector addition to the 3GPP channel model;

If you use this module in your research, please cite:

M. Mezzavilla, M. Zhang, M. Polese, R. Ford, S. Dutta, S. Rangan, M. Zorzi, _"End-to-End Simulation of 5G mmWave Networks,"_ in IEEE Communications Surveys & Tutorials, vol. 20, no. 3, pp. 2237-2263, thirdquarter 2018. [bibtex available here](https://ieeexplore.ieee.org/document/8344116/)

## Future work
We are actively developing new features for the mmWave module, including:
- 3GPP NR beam tracking
- 3GPP NR Integrated Access and Backhaul feature (see [this repo](https://github.com/signetlabdei/ns3-mmwave-iab) for more details)

## About
This module is being developed by [NYU Wireless](http://wireless.engineering.nyu.edu/) and the [University of Padova](http://mmwave.dei.unipd.it/).
This  work  was  supported  in  part by  the  U.S.  Department  of  Commerce  National  Institute  of  Standards  and Technology through the Project “An End-to-End Research Platform for Public Safety  Communications  above  6  GHz”  under  Award  70NANB17H16.



<!-- The new-handover branch offers integration between LTE and mmWave and dual connectivity features.
 -->

## Authors ##

The ns-3 mmWave module is the result of the development effort carried out by different people. The main contributors are: 
- Tommaso Zugno, University of Padova
- Michele Polese, University of Padova
- Matteo Pagin, University of Padova
- Mattia Lecci, University of Padova
- Matteo Drago, University of Padova
- Mattia Rebato, University of Padova
- Menglei Zhang, NYU Wireless
- Marco Giordani, University of Padova
- Marco Mezzavilla, NYU Wireless
- Sourjya Dutta, NYU Wireless
- Russell Ford, NYU Wireless
- Gabriel Arrobo, Intel

## License ##

This software is licensed under the terms of the GNU GPLv2, as like as ns-3. See the LICENSE file for more details.

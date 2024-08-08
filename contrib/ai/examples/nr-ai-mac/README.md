# NR-MAC-AI

### Installation

This installation works on Ubuntu 22.04 and macOS 13.0 or higher

1. ns3 Prerequisites

https://www.nsnam.org/wiki/Installation

2. Boost C++ libraries

sudo apt install libboost-all-dev

3. Protocol buffers

sudo apt install libprotobuf-dev protobuf-compiler

4. pybind11

sudo apt install pybind11-dev

5. Ray RLlib 2.11.0

pip install -U "ray[data,train,tune,serve]==2.11.0"

6. Configure and build the ns-3 library

```shell
cd YOUR_NS3_DIRECTORY
./ns3 configure --enable-examples --disable-werror
./ns3 build ns3ai_nrmac_gym
```

7. Setting up the python interfaces

```bash
pip install -e contrib/ai/python_utils
pip install -e contrib/ai/model/gym-interface/py
```

### Adjustments

### Running the program

1. Training the ML model for different scenarios

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

3. Evaluation 

### Possible Errors

1. ns3-ai environemnt path needs to be defined explicitly

2. Change the pathlossDir in nr-ai-ma.cc

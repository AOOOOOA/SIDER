# CARLA Simulation Testing Tool - Setup & Usage Guide

## Overview
This repository contains a Bayesian Optimization-based testing tool for 3D driving simulator. The tool (`BO.py`) can automatically discover simulator bugs through systematic parameter exploration and clustering analysis.

## Prerequisites
- Ubuntu 18.04.
- Python 3.7+
- NVIDIA GPU with CUDA support (recommended)
- At least 16GB RAM
- 150GB+ free disk space

## Installation Guide

### 1. Environment Setup

```bash
# Update system packages
sudo apt-get update
sudo apt-get upgrade -y

# Install required system dependencies
sudo apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    unzip \
    python3-pip \
    python3-dev \
    libomp-dev \
    nvidia-docker2

# Install Python dependencies
pip3 install --upgrade pip
pip3 install -r requirements.txt
```

Create `requirements.txt`:
```txt
numpy>=1.19.0
scikit-optimize>=0.9.0
scikit-learn>=1.0.0
matplotlib>=3.3.0
pandas>=1.3.0
carla>=0.9.14
psutil>=5.8.0
scipy>=1.7.0
seaborn>=0.11.0
```

### 2. CARLA Installation

```bash
# Create workspace directory
mkdir -p ~/workspace/carla_apollo
cd ~/workspace/carla_apollo

# Download CARLA 0.9.14
wget https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/CARLA_0.9.14.tar.gz

# Extract CARLA
tar -xzf CARLA_0.9.14.tar.gz
mv CARLA_0.9.14 ./

# Make CARLA executable
chmod +x CARLA_0.9.14/CarlaUE4.sh

# Test CARLA installation
cd CARLA_0.9.14
./CarlaUE4.sh -windowed -ResX=800 -ResY=600
# Press Ctrl+C to stop after verifying it starts
```

### 3. Apollo Installation

```bash
cd ~/workspace/carla_apollo

# Clone Apollo repository
git clone https://github.com/ApolloAuto/apollo.git
cd apollo

# Install Apollo dependencies
bash docker/scripts/dev_start.sh
bash docker/scripts/dev_into.sh

# Inside Apollo container, build Apollo
./apollo.sh build
```

### 4. CARLA-Apollo Bridge Installation

```bash
cd ~/workspace/carla_apollo

# Clone carla-apollo bridge
git clone https://github.com/guardstrikelab/carla_apollo_bridge.git carla-apollo-bridge
cd carla-apollo-bridge

# Install bridge dependencies
# pip3 install -r requirements.txt

Follow the Instructions in Carla Apollo Bridge.
```

### 5. Setup Testing Tool

```bash
# Clone your testing repository
cd ~/workspace
git clone https://github.com/AOOOOOA/SIDER.git 
cd SIDER/carla

# Verify the main testing script exists
ls -la BO_extra_ops.py
```

## Usage Guide

### Basic Testing (CARLA Only)

1. **Start CARLA Server:**
```bash
cd ~/workspace/carla_apollo/CARLA_0.9.14
./CarlaUE4.sh -windowed -ResX=1280 -ResY=720 -carla-rpc-port=2000
```

2. **Run Basic BO Testing:**
```bash
cd ~/SIDER/carla_tester
python3 BO.py
```

### Advanced Testing (CARLA + Apollo)

1. **Start CARLA:**
```bash
cd ~/workspace/carla_apollo/CARLA_0.9.14
./CarlaUE4.sh -windowed -ResX=1280 -ResY=720 -carla-rpc-port=2000
```

2. **Start Apollo (in another terminal):**
```bash
cd ~/workspace/carla_apollo/apollo
bash docker/scripts/dev_start.sh
bash docker/scripts/dev_into.sh
# Inside container:
./scripts/bootstrap.sh
```

3. **Start CARLA-Apollo Bridge (in another terminal):**
```bash
cd ~/workspace/carla_apollo/apollo/modules/carla_bridge
python3 main.py
```

4. **Run BO Testing with Apollo:**
```bash
cd ~/workspace/carla/SIDER/carla_apollo
python3 BO_carla_apollo.py 
```


### Advanced Testing (MetaDrive)
Please follow the instructions in github repo of SIDER (https://github.com/AOOOOOA/SIDER).


```
carla_tester/
├── experiments/
│   ├── YYYYMMDD_HHMMSS/          # Experiment timestamp
│   │   ├── logs/                  # Execution logs
│   │   ├── data/                  # Raw experiment data
│   │   └── results/               # Analysis results
├── stage1_data.pkl                # Stage 1 results
├── cluster_analysis/              # Clustering results
├── data_analysis/                 # Analysis scripts and plots
```

## Troubleshooting

### Common Issues:

1. **CARLA won't start:**
```bash
# Check if port is occupied
netstat -tulpn | grep 2000
# Kill existing CARLA processes
pkill -f CarlaUE4
```

2. **Python module errors:**
```bash
# Add CARLA Python API to path
export PYTHONPATH=$PYTHONPATH:~/workspace/carla_apollo/CARLA_0.9.14/PythonAPI/carla/dist/carla-0.9.14-py3.7-linux-x86_64.egg
```


3. **GPU memory issues:**
```bash
# Run CARLA with reduced graphics
./CarlaUE4.sh -quality-level=Low -windowed -ResX=800 -ResY=600
```

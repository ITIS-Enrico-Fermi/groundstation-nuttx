# GroundStation NuttX Kernel

This repository contains a custom version of the NuttX kernel for the ESP32 microcontroller, with custom edits to add LoRa support.
The kernel is designed to be used as a ground receiving station for radio communication with a CanSat.

## Getting Started

To get started, you will need to clone this repository to your local machine in an empty directory:

```sh
mkdir groundstation-nx
cd groundstation-nx
git clone https://github.com/ITIS-Enrico-Fermi/groundstation-nxkernel.git nuttx
```

Once you have cloned the repository, you can build the kernel using the ESP toolchain for Nuttx.

```sh
source setup.sh
cd nuttx
```

Finally, build the kernel using the make command:

```sh
$ make menuconfig     # Configure NuttX with the desired options
$ make -j8            # Build the kernel
```

## Features
This custom version of the NuttX kernel for the ESP32 includes the following features:

- LoRa support via the RFM9x transceiver

- Support for CanSat radio communication protocol

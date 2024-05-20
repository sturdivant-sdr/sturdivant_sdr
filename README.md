<h1 align="center">STURDIVANT SDR
  <div align="center">

![GitHub Repo stars](https://img.shields.io/github/stars/sturdivant20/sturdivant_sdr)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](/LICENSE)
![GitHub pull requests](https://img.shields.io/github/issues-pr/sturdivant20/sturdivant_sdr)
![GitHub issues](https://img.shields.io/github/issues/sturdivant20/sturdivant_sdr)
![GitHub contributors](https://img.shields.io/github/contributors/sturdivant20/sturdivant_sdr)

  </div>
</h1>

# Docs

- [TODO](#todo)
- [Installation](#installation)
- [Building](#building)
- [Executing](#Executing)

## TODO

1. Experiment with non-coherent tracking accumulation.
2. Finish connections between SDR blocks.
3. Optimize threading.

## Installation

Clone the project into desired folder and `cd` into it:

```shell
git clone git@github.com:sturdivant20/sturdivant_sdr/sturdivant_sdr.git sturdivant_sdr
cd sturdivant_sdr
```

Run the `install.sh` script to ensure the correct packages are installed on your system. You may need to allow execution privileges to the script.

```shell
chmod +x install.sh
./install.sh
```

## Building

This project builds simply with cmake. The included `build.sh` script includes basic commands to build the package in a `build` folder.

```shell
chmod +x build.sh
./build.sh
```

## Executing

Each executable and unit test are located directly inside the `build/bin` folder. For example, to test a GPS L1CA channel, run:

```shell
./build/bin/test_channel
```

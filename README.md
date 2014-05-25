Embedded Software for Soldercubes Modular Robots
================================================

This repository contains the source code for the embedded software running on each module of the [Soldercube Modular Robot System](http://creativemachines.cornell.edu/soldercubes). This is written to run on a microcontroller from the Atmel Atmega1284 series, but might work on other devices too. The full hardware documentation for Soldercubes is available [here](https://github.com/jonemo/soldercubes).

### How to compile

The easiest way to work with and compile this code is to use Atmel Studio version 6 or higher. Atmel Studio is Windows only software available for free on the Atmel website: http://www.atmel.com/microsite/atmel_studio6/default.aspx (requires registration). 

Once Atmel Studio installed, open `CubeBrain.atsln`, then press F7 (or `Build` -> `Build Solution`) to compile. 

There are many other tools to compile and run this source code. If you chose to not use Atmel Studio, you will probably ignore all files ending with `.atsln`, `.atsuo`, and `.cproj`.

### How to program a Soldercube

To program the Soldercubes main controller board, you will need an a piece of hardware that support ISP programming, for example the Atmel [STK500](http://www.atmel.com/tools/STK500.aspx) development board, and of course a Soldercube main controller board. 

In Atmel Studio, select `Tools` -> `Add STK500...` to set up the STK500 once. Subsequently, use the `Tools` -> `Device Programming` menu to program the micrcontroller. 

#### Links

 * Project website: http://creativemachines.cornell.edu/soldercubes
 * Hardware design files for Soldercubes: https://github.com/jonemo/soldercubes

### License

This software is licensed under the MIT License. 

### Contact

Project maintainer: [Jonas Neubert](http://www.jonasneubert.com), jn283@cornell.edu

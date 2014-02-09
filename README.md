soldercubes-controller
======================

Embedded code for Soldercubes modules. 

### How to compile

The easiest way to work with and compile this code is to use Atmel Studio version 6 or higher. Atmel Studio is Windows only software available for free on the Atmel website: http://www.atmel.com/microsite/atmel_studio6/default.aspx (requires registration). 

Once Atmel Studio installed, open `CubeBrain.atsln`, then press F7 (or `Build` -> `Build Solution`) to compile. 

### How to program a Soldercube

To program the Soldercubes main controller board, you will need an a piece of hardware that support ISP programming, for example the Atmel [STK500](http://www.atmel.com/tools/STK500.aspx) development board, and of course a Soldercube main controller board. 

In Atmel Studio, select `Tools` -> `Add STK500...` to set up the STK500 once. Subsequently, use the `Tools` -> `Device Programming` menu to program the micrcontroller. 

#### Links

 * Project website: http://creativemachines.cornell.edu/soldercubes
 * Hardware design files for Soldercubes: http://github.com/jonemo/soldercubes

Project maintainer: Jonas Neubert, jn283@cornell.edu, http://www.jonasneubert.com

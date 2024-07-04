<a name="readme-top"></a>

<!-- PROJECT SHIELDS -->
<!--
- Markdown "reference style" links for readability.
- Reference links are enclosed in brackets [ ] instead of parentheses ( ).
  (See the bottom of this document for the declaration of the reference variables)
-->
[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![MIT License][license-shield]][license-url]


<!-- PROJECT LOGO -->
<br />
<div align="center">
<h3 align="center">xESC YardForce (Rev4) adapter for <a href="https://github.com/ClemensElflein/OpenMower">OpenMower</a></h3>

  <p align="center">
    Simple xESC adapter for stock Rev4 (built 2017) blade-motor<br>
    (the one with a internal motor controller under his cap)
    <br />
  </p>

  <a href="https://github.com/ClemensElflein/xESC_YF_rev4-adapter">
    <img src="PCB/xESC_YF_r4.png" alt="Logo">
  </a>


</div>


<!-- ABOUT THE PROJECT -->
## About The Project

<a href="assets/rev4-mow-motor.jpg">
  <img align="right" width="30%" src="assets/rev4-mow-motor.jpg" title="Rev.4 Mow Motor"/></a>

Those who want use <a href="https://github.com/ClemensElflein/OpenMower">OpenMower</a> with an older YardForce "Rev. 4" (built 2017) model, fail with the stock Blade-Motor as it has an embedded motor-controller under it's cap.

It's not very difficult to use this motor:
- Remove stock driver controller
- Wire the coil cables directly
- Add a 5 pin cable to the motor hall sensors
- Connect them to OM's mow-motor-hall-input plug
and use OM's xESC.

But it's somehow "crafted" and you need to be capable to do these things.

As an alternative, this "Adapter" is thought to replace OM's default Mow- xESC.<br>
In addition to this adapter, it's nevertheless **required to exchange two cables of the mow motor cabling**. But this should be also possible for the untalented (see [Requirements](#requirements)).


<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- GETTING STARTED -->
## Getting Started

> [!WARNING]  
> This project is in **early development state** and only generics like start, stop & break do work now!

## Requirements

<a href="assets/cross-gn-bk.jpg">
  <img align="right" width="30%" src="assets/cross-gn-bk.jpg" title="Mandatory cable x-change"/></a>

**It's mandatory to cross the green and black mow-motor-cables!**

Either exchange the pins within the plug, or do it the lazy way like me, with a luster terminal.
The latter has the advantage that you immediately see that you already crossed green/black.

## Prerequisites

* You need an ST-Link debugger/programmer, like one of the cheap "ST-Link V2" probes from one of the big online retailer.
* [STLINK Tools](https://github.com/stlink-org/stlink) >= v1.8.0 (earlier versions do **not** support our MCU)
* As of writing, your openmower image has to be on edge version >= commit [0bfd489](https://github.com/ClemensElflein/open_mower_ros/commit/0bfd489401b3e659d183e6c1f9b557cb0d59d3c1) which happen on 2024-07-04

## Installation

1. Download and unzip the firmware from [Releases](/releases) section (see/open 'Assets')
2. Connect your ST-Link probe to the adapter PCB. Do **not** connect the 3.3V pin if your adapter is already assembled to the OpenMower Mainboard and get powered by it!
3. Flash the adapter:
   ```sh
   st-flash write firmware_xesc_yf_rev4.bin 0x08000000
   ```
   When done, st-flash should report 'Flash written and verified! jolly good!' (or similar)
4. Adapt your mower_config. Here's the relevant section out of mower_config.sh.example:
    ```
   # Select your ESC type
   # Supported values as of today:
   # xesc_mini: for the STM32 version (VESC)
   # xesc_mini_w_r4ma: for the STM32 version (VESC), but with Rev4 (Mow) Motor Adapter (only available for YardForceSA650 mower type)
   # xesc_2040: for the RP2040 version (very experimental!)
   # xesc_2040_w_r4ma: for the RP2040 version (very experimental!), but with Rev4 (Mow) Motor Adapter (only available for YardForceSA650 mower type)
   export OM_MOWER_ESC_TYPE="xesc_mini"
    ```
5. Restart openmower via:
   ```sh
   sudo systemctl restart openmower
   ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>


## LED codes

<table>
  <tr><th>Green</th><th>Red</th><th>Description</th></tr>
  <tr><td colspan="2" align="center">3 * flash</td><td>'Power up' successful</td></tr>
  <tr><td>on</td><td></td><td>Ready and connected by xesc_ros</td></tr>
  <!-- <tr><td>flash</td><td></td><td>SA tacho flash for 90° rotation</td></tr> -->
  <tr><td></td><td>blink (1Hz)</td><td>Waiting for OpenMower (xesc_ros driver connect) or Motor/Controller Fault. Check ESC's status via `rostopic echo  /mower/status` error code</td></tr>
  <tr><td></td><td>flash</td><td>One single short flash for every host communication error, like packet or CRC error</td></tr>
  <tr><td></td><td>on</td><td>Wrong ROS driver, which result in mass packet size or CRC errors, which in turn look like 'Red=on'</td></tr>
</table>


<!--

-->


<!-- USAGE EXAMPLES -->
<!--
## Usage

Use this space to show useful examples of how a project can be used. Additional screenshots, code examples and demos work well in this space. You may also link to more resources.

_For more examples, please refer to the [Documentation](https://example.com)_

<p align="right">(<a href="#readme-top">back to top</a>)</p>
-->


<!-- ROADMAP -->
## Roadmap

(Ordered by priority)

- [x] Design schematics and PCB 
- [x] Produce PCB
- [ ] Firmware 
    - [x] Generic VM control (on/off)
    - [x] Generic motor control (start, stop, break)
    - [x] Read SA, math RPM
    - [ ] Open VMC (no motor connected) detection
    - [ ] Motor current consumption
    - [ ] Stock motor (wrong) cabling detection
    - [ ] STM32 bootloader / flash via UART support
- [x] ROS driver
    - [x] xesc_ros::xesc_yfr4

See the [open issues](https://github.com/ClemensElflein/xESC_YF_rev4-adapter/issues) for a full list of proposed features (and known issues).

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- CONTRIBUTING -->
## Contributing

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".
Don't forget to give the project a star! Thanks again!

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- LICENSE -->
## License

Distributed under the MIT License. See `LICENSE.txt` for more information.

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- CONTACT -->
## Contact

Project Link: [https://github.com/ClemensElflein/xESC_YF_rev4-adapter](https://github.com/ClemensElflein/xESC_YF_rev4-adapter)

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- ACKNOWLEDGMENTS -->
<!--
## Acknowledgments

* []()
* []()
* []()

<p align="right">(<a href="#readme-top">back to top</a>)</p>
->


<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/ClemensElflein/xESC_YF_rev4-adapter.svg?style=for-the-badge
[contributors-url]: https://github.com/ClemensElflein/xESC_YF_rev4-adapter/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/ClemensElflein/xESC_YF_rev4-adapter.svg?style=for-the-badge
[forks-url]: https://github.com/ClemensElflein/xESC_YF_rev4-adapter/network/members
[stars-shield]: https://img.shields.io/github/stars/ClemensElflein/xESC_YF_rev4-adapter.svg?style=for-the-badge
[stars-url]: https://github.com/ClemensElflein/xESC_YF_rev4-adapter/stargazers
[issues-shield]: https://img.shields.io/github/issues/ClemensElflein/xESC_YF_rev4-adapter.svg?style=for-the-badge
[issues-url]: https://github.com/ClemensElflein/xESC_YF_rev4-adapter/issues
[license-shield]: https://img.shields.io/github/license/ClemensElflein/xESC_YF_rev4-adapter.svg?style=for-the-badge
[license-url]: https://github.com/ClemensElflein/xESC_YF_rev4-adapter/blob/master/LICENSE.txt

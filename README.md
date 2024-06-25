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
> This project is in **early development state** and only generics like start, stop & break do work!

## Requirements

It's mandatory to cross the green and black mow-motor-cable.
Either exchange the pins within the plug, or do it the lazy way like me, with a luster terminal:
[![Mandatory Hardware Modification](assets/cross-gn-bk.jpg "Open in browser")](assets/cross-gn-bk.jpg)
The latter has the advantage that you immediately see that you already crossed green/black.

<!--
This is an example of how you may give instructions on setting up your project locally.
To get a local copy up and running follow these simple example steps.
-->

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
### Prerequisites

This is an example of how to list things you need to use the software and how to install them.
* npm
  ```sh
  npm install npm@latest -g
  ```

### Installation

1. Get a free API Key at [https://example.com](https://example.com)
2. Clone the repo
   ```sh
   git clone https://github.com/ClemensElflein/xESC_YF_rev4-adapter.git
   ```
3. Install NPM packages
   ```sh
   npm install
   ```
4. Enter your API in `config.js`
   ```js
   const API_KEY = 'ENTER YOUR API';
   ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>
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
    - [ ] Open VMC detection
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
[PlatformIO-shield]: https://img.shields.io/badge/build%20with-PlatformIO-orange?logo=data%3Aimage%2Fsvg%2Bxml%3Bbase64%2CPHN2ZyB3aWR0aD0iMjUwMCIgaGVpZ2h0PSIyNTAwIiB2aWV3Qm94PSIwIDAgMjU2IDI1NiIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIiBwcmVzZXJ2ZUFzcGVjdFJhdGlvPSJ4TWlkWU1pZCI+PHBhdGggZD0iTTEyOCAwQzkzLjgxIDAgNjEuNjY2IDEzLjMxNCAzNy40OSAzNy40OSAxMy4zMTQgNjEuNjY2IDAgOTMuODEgMCAxMjhjMCAzNC4xOSAxMy4zMTQgNjYuMzM0IDM3LjQ5IDkwLjUxQzYxLjY2NiAyNDIuNjg2IDkzLjgxIDI1NiAxMjggMjU2YzM0LjE5IDAgNjYuMzM0LTEzLjMxNCA5MC41MS0zNy40OUMyNDIuNjg2IDE5NC4zMzQgMjU2IDE2Mi4xOSAyNTYgMTI4YzAtMzQuMTktMTMuMzE0LTY2LjMzNC0zNy40OS05MC41MUMxOTQuMzM0IDEzLjMxNCAxNjIuMTkgMCAxMjggMCIgZmlsbD0iI0ZGN0YwMCIvPjxwYXRoIGQ9Ik0yNDkuMzg2IDEyOGMwIDY3LjA0LTU0LjM0NyAxMjEuMzg2LTEyMS4zODYgMTIxLjM4NkM2MC45NiAyNDkuMzg2IDYuNjEzIDE5NS4wNCA2LjYxMyAxMjggNi42MTMgNjAuOTYgNjAuOTYgNi42MTQgMTI4IDYuNjE0YzY3LjA0IDAgMTIxLjM4NiA1NC4zNDYgMTIxLjM4NiAxMjEuMzg2IiBmaWxsPSIjRkZGIi8+PHBhdGggZD0iTTE2MC44NjkgNzQuMDYybDUuMTQ1LTE4LjUzN2M1LjI2NC0uNDcgOS4zOTItNC44ODYgOS4zOTItMTAuMjczIDAtNS43LTQuNjItMTAuMzItMTAuMzItMTAuMzJzLTEwLjMyIDQuNjItMTAuMzIgMTAuMzJjMCAzLjc1NSAyLjAxMyA3LjAzIDUuMDEgOC44MzdsLTUuMDUgMTguMTk1Yy0xNC40MzctMy42Ny0yNi42MjUtMy4zOS0yNi42MjUtMy4zOWwtMi4yNTggMS4wMXYxNDAuODcybDIuMjU4Ljc1M2MxMy42MTQgMCA3My4xNzctNDEuMTMzIDczLjMyMy04NS4yNyAwLTMxLjYyNC0yMS4wMjMtNDUuODI1LTQwLjU1NS01Mi4xOTd6TTE0Ni41MyAxNjQuOGMtMTEuNjE3LTE4LjU1Ny02LjcwNi02MS43NTEgMjMuNjQzLTY3LjkyNSA4LjMyLTEuMzMzIDE4LjUwOSA0LjEzNCAyMS41MSAxNi4yNzkgNy41ODIgMjUuNzY2LTM3LjAxNSA2MS44NDUtNDUuMTUzIDUxLjY0NnptMTguMjE2LTM5Ljc1MmE5LjM5OSA5LjM5OSAwIDAgMC05LjM5OSA5LjM5OSA5LjM5OSA5LjM5OSAwIDAgMCA5LjQgOS4zOTkgOS4zOTkgOS4zOTkgMCAwIDAgOS4zOTgtOS40IDkuMzk5IDkuMzk5IDAgMCAwLTkuMzk5LTkuMzk4em0yLjgxIDguNjcyYTIuMzc0IDIuMzc0IDAgMSAxIDAtNC43NDkgMi4zNzQgMi4zNzQgMCAwIDEgMCA0Ljc0OXoiIGZpbGw9IiNFNTcyMDAiLz48cGF0aCBkPSJNMTAxLjM3MSA3Mi43MDlsLTUuMDIzLTE4LjkwMWMyLjg3NC0xLjgzMiA0Ljc4Ni01LjA0IDQuNzg2LTguNzAxIDAtNS43LTQuNjItMTAuMzItMTAuMzItMTAuMzItNS42OTkgMC0xMC4zMTkgNC42Mi0xMC4zMTkgMTAuMzIgMCA1LjY4MiA0LjU5MiAxMC4yODkgMTAuMjY3IDEwLjMxN0w5NS44IDc0LjM3OGMtMTkuNjA5IDYuNTEtNDAuODg1IDIwLjc0Mi00MC44ODUgNTEuODguNDM2IDQ1LjAxIDU5LjU3MiA4NS4yNjcgNzMuMTg2IDg1LjI2N1Y2OC44OTJzLTEyLjI1Mi0uMDYyLTI2LjcyOSAzLjgxN3ptMTAuMzk1IDkyLjA5Yy04LjEzOCAxMC4yLTUyLjczNS0yNS44OC00NS4xNTQtNTEuNjQ1IDMuMDAyLTEyLjE0NSAxMy4xOS0xNy42MTIgMjEuNTExLTE2LjI4IDMwLjM1IDYuMTc1IDM1LjI2IDQ5LjM2OSAyMy42NDMgNjcuOTI2em0tMTguODItMzkuNDZhOS4zOTkgOS4zOTkgMCAwIDAtOS4zOTkgOS4zOTggOS4zOTkgOS4zOTkgMCAwIDAgOS40IDkuNCA5LjM5OSA5LjM5OSAwIDAgMCA5LjM5OC05LjQgOS4zOTkgOS4zOTkgMCAwIDAtOS4zOTktOS4zOTl6bS0yLjgxIDguNjcxYTIuMzc0IDIuMzc0IDAgMSAxIDAtNC43NDggMi4zNzQgMi4zNzQgMCAwIDEgMCA0Ljc0OHoiIGZpbGw9IiNGRjdGMDAiLz48L3N2Zz4=
[PlatformIO-url]: https://platformio.org/

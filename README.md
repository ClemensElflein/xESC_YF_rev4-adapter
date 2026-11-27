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
    (the one with an internal motor controller under its cap)
    <br />
  </p>

  |             v1.x Hardware              |             v2.x Hardware              |
  | :------------------------------------: | :------------------------------------: |
  | ![HW v1.x](assets/xESC_YF_r4_v1.0.png) | ![HW v2.x](assets/xESC_YF_r4_v2.0.png) |

</div>


<!-- ABOUT THE PROJECT -->
## About The Project

<a href="assets/rev4-mow-motor.jpg">
  <img align="right" width="30%" src="assets/rev4-mow-motor.jpg" title="Rev.4 Mow Motor"/></a>

Those who want to use <a href="https://github.com/ClemensElflein/OpenMower">OpenMower</a> with an older YardForce "Rev. 4" (built 2017) model, fail with the stock Blade-Motor as it has an embedded motor-controller under its cap.

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


## Requirements


<a href="assets/cross-gn-bk.jpg">
  <img align="right" width="30%" src="assets/cross-gn-bk.jpg" title="Mandatory cable x-change"/></a>

**It's mandatory to cross the green and black mow-motor-cables!**

> [!WARNING]  
> **Never** plug or un-plug the mow-motor while the adapter is powered!

> [!WARNING]  
> Do **not** simply trust the cable colors!<br>
> Check the image and validate if your
> cables do have the same colors! If not adapt accordingly

Either exchange the pins within the plug, or do it the lazy way like me, with a luster terminal.
The latter has the advantage that you immediately see that you already crossed green/black.

## Prerequisites

* The compiled firmware, which can be found within the [Actions](https://github.com/ClemensElflein/xESC_YF_rev4-adapter/actions) section. Click the one you're interested in (newest or tagged), scroll down to 'Artifacts', download and unzip to somewhere.
* As of writing, your openmower image has to be on edge version >= commit [0bfd489](https://github.com/ClemensElflein/open_mower_ros/commit/0bfd489401b3e659d183e6c1f9b557cb0d59d3c1) which happened on 2024-07-04

## Firmware Installation

### Requirements

* You need an ST-Link debugger/programmer, like one of the cheap "ST-Link V2" probes from one of the big online retailer.
* [STLINK Tools](https://github.com/stlink-org/stlink) >= v1.8.0 (earlier versions do **not** support the used STM32C0x MCU)

### Hardware Version Branding

Starting with hardware version 2.0, the adapter's MCU needs to be branded by writing a hardware version record into the OTP area.

#### Branding Process:

1. **Generate** Hardware Version Binaries

   ```bash
   cd firmware/host-hw-config
   lbuild build
   scons
   ```

   This creates i.e.:
   - `hw_config_v1.0.bin` - Hardware v1.0 version record
   - `hw_config_v2.0.bin` - Hardware v2.0 version record
   - `hw_config_v2.1.bin` - Hardware v2.1 version record
  
1. **Connect your ST-Link** to the adapter PCB. Do **not** connect the 3.3V pin if your adapter is already assembled to the OpenMower Mainboard and gets powered by it!

1. **Write** Hardware Version Record

   ```bash
   # Write/brand v2.0 version record with ST-Link/V2
   STM32_Programmer_CLI -c port=SWD -d hw_config_v2.0.bin 0x1FFF7000

   # Write/brand v2.1 version record with ST-Link/V2 (R17 must be 510 Ω)
   STM32_Programmer_CLI -c port=SWD -d hw_config_v2.1.bin 0x1FFF7000
   ```

The branding process writes a small hardware version record to the first two OTP double words (64 bit each). **Take attention** that OTP can be written only once!

> [!WARNING]
> **Hardware v2.1.0** requires reworking the current-sense resistor **R17 = 510 Ω** (1 %, 0603).
> The firmware selects its analog profile from the OTP hardware version:
> v2.1 → 510 Ω / 2.0 A, everything else → 1 kΩ / 1.25 A.
> A v2.1 board **must** be branded with `hw_config_v2.1.bin` and needs firmware >= 0.4.0.

**Important Notes:**
- Branding only needs as well as "can" be done only **once per device**
- The branding **truly survives firmware updates** (because OTP can be written only once)
- Hardware Version 1.x boards don't need branding. Non-branded devices are assumed to be v1.0 hardware **by the v1 firmware**. A v2 binary treats a missing/invalid OTP record as wrong hardware (red LED on, motor disabled).
<br>
<br>

**Firmware Installation Variants**

We have two options to install the firmware. Either via:<br>
- ST-Link debugger/programmer, for which you need to have access to the adapter PCB's SWDIO pins, or via
- Open-Mower's UART via STM32 internal bootloader (only OpenMower V1 based hardware; not supported on V2 carrier boards)

The adapter itself runs on OpenMower V1 and V2 hardware. UART/bootloader flashing is only available on V1; on V2 use the ST-Link/SWD path.


### Firmware Installation via ST-Link debugger/programmer


#### Flash

1. Connect your ST-Link probe to the adapter PCB. Do **not** connect the 3.3V pin if your adapter is already assembled to the OpenMower Mainboard and gets powered by it!

2. Flash the adapter:
   ```sh
   st-flash --reset write firmware_xesc_yf_rev4.bin 0x08000000
   ```
   When done, st-flash should report 'Flash written and verified! jolly good!' (or similar) and immediately reboot.<br>

   **Only relevant for v1.x hardware:**

   > [!WARNING]  
   > Do not unplug now!
   
   On absolutely first firmware boot, it needs to flash some settings to the MCU. You'll see some blink codes.<br>
   Do NOT unplug till you see:
   - 3 simultaneous blinks of the green+red LED (=success), following of a
   - quick blink of the red LED (=no motor connected) as well as a
   - normal green blink (=ROS not connected).
   
   The whole procedure will only take about 10 seconds. Once the procedure got processed it won't get triggered again for this MCU on future firmware flashes.

   If the board doesn't come up with blink signs, unplug and replug it. But once you see blink signs, do **not** unplug it before getting the described LED codes of the previous paragraph.   

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Firmware Installation via serial connection (UART)

Firmware flashing via UART is currently only possible with OpenMower V1 based hardware.
The adapter also runs on OpenMower V2 hardware, but UART/bootloader flashing is not supported on the V2 carrier boards; use ST-Link/SWD there.

#### Requirements

* [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html) is the tool we need to flash via UART. But because we like to use it with our OpenMower's Raspberry-Pi, we need it in `aarch64` architecture. Unfortunately (as well as unbelievable) ST's programmer tool for ARM CPU's isn't available *for* ARM CPU's/architectures!!<br>
So we need to trick a little bit, and for this you need a non-arm Linux machine (x86_64).<br>
  1. Download [STM32CubePrg-Lin](https://www.st.com/en/development-tools/stm32cubeprog.html) package with (or to) a x86_64 Linux machine
  2. Unpack and install it on the x86_64 Linux machine
  3. Pack (zip, tar, ...) the installed binaries from where you just installed them
  4. Copy that package to your Raspberry-Pi and unpack it to somewhere. I placed mine to /opt/STM32CubeProgrammer
* [box64](https://github.com/ptitSeb/box64) to be able to run the STM32CubeProgrammer's x86_64 binary with our aarch64 architecture.<br>
Luckily they do have pre-build [Debian packages](https://github.com/ptitSeb/box64/blob/main/docs/COMPILE.md#debian-based-linux). Please follow their instruction.

Once *box64* got installed, you should be able to start the STM32Programmer CLI i.e. via: `/opt/STM32CubeProgrammer/bin/STM32_Programmer_CLI`


### Flash

1. Login to your OpenMower Raspberry-Pi and stop openmower:<br>
   `sudo systemctl stop openmower`
   
2. Configure Raspberry-PI's serial device where the adapter is connected to, which is normally */dev/ttyAMA3*:<br>
   `stty -F /dev/ttyAMA3 115200 raw`
   
3. Now trigger the current running adapter firmware to do a reboot into the bootloader via:<br>
    `echo -e "\x00Reboot into Bootloader" > /dev/ttyAMA3`

4. Check if STM32 MCU is now in bootloader mode (Step 3. was successful):<br>
   `/opt/STM32CubeProgrammer/bin/STM32_Programmer_CLI -c port=/dev/ttyAMA3`<br>
   Should output something like:<br>

   ```
      -------------------------------------------------------------------
                        STM32CubeProgrammer v2.16.0                  
      -------------------------------------------------------------------

   Serial Port /dev/ttyAMA3 is successfully opened.
   Port configuration: parity = even, baudrate = 115200, data-bit = 8,
                       stop-bit = 1.0, flow-control = off
   Activating device: OK
   Board       : --
   Chip ID: 0x443 
   BootLoader protocol version: 3.1
   Device name : STM32C01x
   Flash size  : 32 KBytes (default)
   Device type : MCU
   Revision ID : --  
   Device CPU  : Cortex-M0+
   ```

5. Flash:<br>
   `/opt/STM32CubeProgrammer/bin/STM32_Programmer_CLI -c port=/dev/ttyAMA3 -w firmware_xesc_yf_rev4.bin 0x08000000`

6. Reset the device (start our newly flashed program):<br>
   `/opt/STM32CubeProgrammer/bin/STM32_Programmer_CLI -c port=/dev/ttyAMA3 --go 0x08000000`

   **Only relevant for v1.x hardware:**

   > [!WARNING]  
   > Do not power-off now!
   
   On absolutely first firmware boot, it needs to flash some settings to the MCU. You'll see some blink codes.<br>
   Do NOT unplug till you see:
   - 3 simultaneous blinks of the green+red LED (=success), following of a
   - quick blink of the red LED (=no motor connected) as well as a
   - normal green blink (=ROS not connected).
   
   The whole procedure will only take about 10 seconds. Once the procedure got processed it won't get triggered again for this MCU on future firmware flashes.

7. Finally start openmower again:<br>
   `sudo systemctl start openmower`

#### Alternative: native `stm32loader` (no box64)

[`stm32loader`](https://github.com/florisla/stm32loader) supports the STM32C0
bootloader and runs natively on the Raspberry Pi. Install `uv` once, then flash:

```bash
# one-time: install uv/uvx (official installer)
curl -LsSf https://astral.sh/uv/install.sh | sh

# flash (the three commands)
sudo systemctl stop openmower
stty -F /dev/ttyAMA3 115200 raw
echo -e "\x00Reboot into Bootloader" > /dev/ttyAMA3
uvx stm32loader -p /dev/ttyAMA3 -b 115200 -f C0 -e -w -v firmware_xesc_yf_rev4.bin -g 0x08000000
sudo systemctl start openmower
```

Notes:
- Use `-e` (global mass erase), **not** `--auto-erase`; the C0 page size is not
  defined in `stm32loader`'s device table.
- The STM32CubeProgrammer method above is the tested fallback.
- The v1.x first-boot warning above applies to this method too.


<p align="right">(<a href="#readme-top">back to top</a>)</p>

## OpenMower Config

### OpenMower V1 based Hardware (mower_config)

Lastly, you need to add the adapter to your mower_config. Here's the relevant section out of mower_config.sh.example:

```
# Select your ESC type
# Supported values as of today:
# xesc_mini: for the STM32 version (VESC)
# xesc_mini_w_r4ma: for the STM32 version (VESC), but with Rev4 (Mow) Motor Adapter (only available for YardForceSA650 mower type)
# xesc_2040: for the RP2040 version (very experimental!)
# xesc_2040_w_r4ma: for the RP2040 version (very experimental!), but with Rev4 (Mow) Motor Adapter (only available for YardForceSA650 mower type)
export OM_MOWER_ESC_TYPE="xesc_mini"
```

> [!IMPORTANT]
> For this adapter you have to use one of the `_w_r4ma` variants, i.e.
> `export OM_MOWER_ESC_TYPE="xesc_mini_w_r4ma"` (or `"xesc_2040_w_r4ma"` with an RP2040 based xESC)
> **and** `export OM_MOWER="YardForceSA650"`. Only that combination selects
> `comms_xesc_mini_w_r4ma_params.yaml`, which drives the mower ESC with the `xesc_yfr4` driver on `/dev/ttyAMA3`.
> The plain `xesc_mini` shown above is only the default of the example file.

Once adapted, restart openmower via: `sudo systemctl restart openmower`

### OpenMower V2 based Hardware: (xCore Firmware)

Edit OpenMower stack config via `openmower configure env` and set `FIRMWARE="yardforce-v4"`

> [!NOTE]
> The adapter runs on OpenMower V2 hardware, but firmware updates via UART/bootloader are only available on V1. On V2 use ST-Link/SWD.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


## LED codes

<table>
  <tr><th>White (Green)</th><th>Red</th><th>Description</th></tr>
  <tr><td colspan="2" align="center">3 * quick blink</td><td>'Power up' successful</td></tr>
  <tr><td>1Hz blink</td><td></td><td>Waiting for init by xesc_ros</td></tr>
  <tr><td>0.5Hz flash<br>(short flash every 2 seconds)</td><td></td><td>Shutdown (Sleep)</td></tr>
  <tr><td>on</td><td></td><td>All fine (initialized, xesc_ros connected and no error)</td></tr>  
  <tr><td></td><td>1Hz blink</td><td>Waiting for OpenMower (xesc_ros driver) connect</td></tr>
  <tr><td></td><td>flash</td><td>One single short flash for every host communication error, like packet or CRC error</td></tr>
  <tr><td></td><td>on</td><td><b>Wrong hardware version firmware flashed!</b><br>Motor disabled for safety. Flash correct firmware</td></tr>
  <tr><td></td><td>5 * blink</td><td>NRST Pin not yet disabled. <b>Keep board powered</b> and watch for further LED codes</td></tr>
  <tr><td>5 * blink</td><td></td><td>NRST successful flashed</td></tr>
  <tr><td></td><td>4Hz quick blink</td><td>Open VMC (no motor connected).<br> Keeps VMC off until a motor is connected, then clears by itself (~2 s).<br> <b>WARNING:</b> Do not connect the motor while the adapter is powered!</td></tr>
  <tr><td></td><td>2Hz fast blink</td><td>VMS temperature issue or over-current detected</td></tr>
</table>

<p align="right">(<a href="#readme-top">back to top</a>)</p>


## Self-Compile

This project is build with [modm](https://modm.io). You need to install the required toolchain as described in their [Installation](https://modm.io/guide/installation/) section, before going on. Note that modm's `repo.lb` additionally requires [GitPython](https://pypi.org/project/GitPython/): `pip3 install modm GitPython`.

Once done:

1. Clone this repository (inclusive submodules):<br>
  `git clone --recurse-submodules https://github.com/ClemensElflein/xESC_YF_rev4-adapter.git`
2. Change into the project source directory:<br>
  `cd xESC_YF_rev4-adapter/firmware/src`
3. Build the modm library files for a specific hardware version:<br>
   ```bash
   # This builds for HW v2.0 and v2.1 (same binary, profile selected via OTP)
   lbuild -c hw-v2.xml build
   ```
4. Compile the firmware:<br>
  `scons build profile=<release|debug>` builds the `.elf`, `scons build bin profile=<release|debug>` additionally creates the `.bin` (which is what `st-flash` needs, see above).
5. Flash firmware:<br>
  For STM32C0 support, [xPack OpenOCD](https://xpack-dev-tools.github.io/openocd-xpack/) **>= 0.12.0-3** is required. Either install your own one, or use the bundled one (linux-x64 only; on aarch64/Raspberry Pi use a native OpenOCD >= 0.12.0-3):<br>
  `export MODM_OPENOCD_BINARY=../packages/xpack-openocd-0.12.0-3-linux-x64/bin/openocd`<br>
  Afterwards flash the firmware via:<br>
  `scons program`<br>
  `scons program` runs `program ... verify` followed by `reset run`. With the ST-Link HLA driver, OpenOCD resets via SRST (NRST). If NRST is not connected (or disabled on v1.x), the MCU may not reset; connect NRST or power-cycle the board. `scons reset` uses the same SRST path.

<p align="right">(<a href="#readme-top">back to top</a>)</p>



## Roadmap

(Ordered by priority)

- [x] Design schematics and PCB 
- [x] Produce PCB
- [ ] Firmware 
    - [x] Generic VM control (on/off)
    - [x] Generic motor control (start, stop, break)
    - [x] Read SA, math RPM
    - [x] Open VMC (no motor connected) detection
    - [x] Disable NRST pin functionality
    - [x] Motor current consumption
    - [x] PCB temperature (junction temp of STM)
    - [x] Support shutdown signal
    - [ ] ~~Stock motor (wrong) cabling detection~~
    - [x] STM32 bootloader / flash via UART support (OpenMower V1 based mainboard)
    - [ ] UART flash support for OpenMower V2 based carrier-boards
- [x] ROS driver
    - [x] xesc_ros::xesc_yfr4
    - [x] Add RPM
- [x] OpenMower Hardware V2 firmware driver

See the [open issues](https://github.com/ClemensElflein/xESC_YF_rev4-adapter/issues) for a full list of proposed features (and known issues).

<p align="right">(<a href="#readme-top">back to top</a>)</p>


## History

### Firmware

  | Version | Release Date | Info                                                                                                                                                                                                                                                                                     |
  | ------- | :----------: | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
  | 0.4.0   |  2026-11-17  | - Add support for Rev4-ESC hardware v2.1 (OTP version 2.1)<br>- Select R_CS (R17) and current limit from the OTP-based hardware profile<br>- Bump modm to 2026q3<br>- Rework Open-Load ("Open VMC") detection: calibrate the CS zero offset at boot and compare voltage-based thresholds |
  | 0.3.0   |  2025-10-25  | - Fix "Open-Load" detection for Rev4-ESC hardware v1.0<br>- Improve load current and RPM calculations<br>- Add hardware/firmware mixup protection<br>- Add support for Rev4-ESC hardware v2.x                                                                                            |
  | 0.2.3   |  2024-10-03  | - Add RPM to status package                                                                                                                                                                                                                                                              |
  | 0.2.2   |  2024-08-25  | - Support STM32 bootloader flash via UART                                                                                                                                                                                                                                                |
  | 0.2.1   |  2024-08-19  | - Add shutdown signal handling (sleep/power-off VMC)<br>- Handle over-temp and over-current                                                                                                                                                                                              |
  | 0.2.0   |  2024-08-16  | - Switch from Arduino to modm lib<br>- Integrate flash procedure to disable-NRST<br>- Add Motor current and PCB temperature                                                                                                                                                              |
  | 0.1.1   |  2024-07-10  | - Open VMC (no motor connected) detection<br>- VMC-short and thermal error detection                                                                                                                                                                                                     |
  | 0.1.0   |  2024-07-05  | Generic functionality like Start, Stop, Break                                                                                                                                                                                                                                            |

### Hardware

  | Version | Release Date | Info                                                                                                                                                                                              |
  | ------- | :----------: | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
  | 2.1.0   |  2026-12-??  | - Improve current sense ADC clamping<br>- Increase HW current limit to 3A (R14/R_CL = 820 Ω)<br>- R17 (R_CS) rework to 510 Ω for the 2 A firmware profile<br>- Minor part and routing adjustments |

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

Distributed under the MIT License. See [LICENSE](LICENSE) for more information.

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- CONTACT -->
## Contact

Project Link: [https://github.com/ClemensElflein/xESC_YF_rev4-adapter](https://github.com/ClemensElflein/xESC_YF_rev4-adapter)

<p align="right">(<a href="#readme-top">back to top</a>)</p>


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
[license-url]: https://github.com/ClemensElflein/xESC_YF_rev4-adapter/blob/master/LICENSE

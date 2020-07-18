===============
Build Materials
===============
Last modified: June 2020

CAD design of the MSL Quadrotor
------------------------------------

.. image:: https://raw.githubusercontent.com/StanfordMSL/msl_quad/master/Hardware/quadrotor_rendering.jpg
  :width: 400
  :height: 300


CAD Environment
------------------

* Solidworks 2016
* main assembly file: quadrotor.SLDASM

3D printing
------------------

* We used Ultimaker 2+

* For the sake of quality, consider slowing down the print speed, especially for large parts

* Use brim (at leat 5mm is recommended) in Cura to enhance adhesion to the print bed.

Bills of Materials
-------------------

The following parts are required per quadrotor:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

================================================================ === ================================================================================================================================
Components                                                       Qty           Link
---------------------------------------------------------------- --- --------------------------------------------------------------------------------------------------------------------------------
PixFalcon Autopilot w/ GPS and Power Module                       1        `Amazon <https://www.amazon.com/Pixfalcon-M8N-GPS-Advanced-Bundle/dp/B017YCYVRO>`_
F330 Frame                                                        1        `Amazon <https://www.amazon.com/Quadcopter-Frame-Aircraft-Accessory-Integrated/dp/B07D6K51DY>`_
Motors: Crazepony 4pcs Emax MT2213 935KV                          1        `Amazon <https://www.amazon.com/MT2213-935KV-Brushless-Motor-Quadcopter-Multirotor/dp/B00N3I9GM4>`_
8045 Propeller Set (2xCW, 2xCCW>)                                 1        `Amazon <https://www.amazon.com/DJI-Innovations-Quadcopter-Original-Propellers/dp/B00BS40S4A>`_
ESC: ZTW Spider 30A OPTO 2-6S LiPo 600HZ Simonk OneShot125 ESC    4        `Amazon <https://www.amazon.com/gp/product/B01BXS7NQ6>`_
Spektrum 9645 DSMX Remote Receiver                                1        `Amazon <https://www.amazon.com/gp/product/B004M12GY6/>`_
Turnigy 3300mah 3s 30c LiPo Battery                               1        `HobbyKing <https://hobbyking.com/en_us/turnigy-battery-3300mah-3s-30c-lipo-pack-xt-60.html>`_
Spektrum DX6i 6 Channel Transmitter                               1        `Amazon <https://www.amazon.com/gp/product/B00K1P3KYW>`_
Pololu 5V, 5A Step-Down Voltage Regulator D24V50F5                1        `Pololu <https://www.pololu.com/product/2851>`_
Odroid XU4                                                        1        `AmeriDroid <https://ameridroid.com/products/odroid-xu4>`_
64GB eMMC                                                         1        `HardKernel <https://www.hardkernel.com/shop/64gb-emmc-module-xu4-linux/>`_
Wifi Module for Odroid                                            1        `AmeriDroid <https://ameridroid.com/products/wifi-module-3>`_
RTC battery for Odroid                                            1        `AmeriDroid <https://ameridroid.com/products/rtc-battery>`_
Male-Female Threaded Hex Standoff, 20mm Length, M2.5 Thread       8        `McMaster <https://www.mcmaster.com/#98952a117/=18p51v7>`_
Female Threaded Hex Standoff, 10 mm Length, M3 Thread             4        `McMaster <https://www.mcmaster.com/#95947a006/=18p528y>`_
CP2102 Module USB 2.0 to TTL 6PIN                                 1        `Amazon <https://www.amazon.com/Honbay-CP2102-Module-Download-Converter/dp/B01A0BOGHG>`_
================================================================ === ================================================================================================================================

Buy appropriate amount of the following accessories:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
===================  ============================================================================================ 
 Components                             Link
-------------------  --------------------------------------------------------------------------------------------
XT 60 Connectors      `Amazon <https://www.amazon.com/Finware-Female-Bullet-Connectors-Battery/dp/B01ETROGP4>`_
10AWG Primary Wire    `Amazon <https://www.amazon.com/Grand-General-55261-10-Gauge-Primary/dp/B00INVF40E/>`_
16AWG Primary Wire    `Amazon <https://www.amazon.com/Grand-General-55231-16-Gauge-Primary/dp/B00INVEUS6/>`_
DC Power Plug         `Amazon <https://www.amazon.com/SIM-NAT-Pigtails-Security-Surveillance/dp/B01GPL8MVG/>`_
===================  ============================================================================================ 

Assembly Guide
----------------------

#. Attach a MT2213 935KV motor to each F330 arm.
#. Connect a ZTW Spider ESC to each motor. The ESCs should be physically attached to the arms with plastic tape so they do not interfere with propellers.
#. Solder two 10 Gauge wires (red and black) to the bottom plate of F330, connecting the end of both wires to a XT 60 connector for battery connection.
#. Solder two 16 Gauge wires (red and black) to the bottom plate of F330, on top of where the 10 Gauge wires are soldered. The other end of the wires are soldered to the input+/- terminals of the power distribution board for PixFalcon.
#. Solder two 16 Gauge wires (red and black) to the bottom plate of F330, on top of where other wires are soldered. The other end of the wires go to the input terminals of the Polulu Voltage Regulator.
#. Solder a DC power plug to the output terminals of the voltage regulator. This plug will power the Odroid. 
#. Attach the Pixfalcon power distribution board, Pololu voltage regulator, and the Odroid onto a laser-cut acrylic board. The Odroid is mounted with four 3D-printed Odroid Raiser (i.e. short standoffs) facing downwards. Attach the acrylic board to the F330 bottom plate.
#. Attach two 3D-printed Landing Gears to the F330 bottom plate so the Odroid does not touch the ground directly.
#. Sandwich the four F330 arms with the bottom and the top plate of the F330 frame, using four screws only. The other four halls are for the 20mm male-female standoffs.
#. Attach a 3D-printed Pixfalcon Holder onto the top plate and tighten the screws.
#. Stick the Pixfalcon flight controller carefully onto the Pixfalcon Holder with a piece of glue sponge that is included in the flight controller kit.
#. Put a 3D-printed Pixfalcon Cover onto the four standoffs so it covers the flight controller.
#. Use another set of four 20mm male-female standoffs to sandwich the Pixfalcon Cover.
#. Put a 3D-printed top layer on top of the four standoffs. Place the specturm receiver and the PixFalcon GPS module appropriately onto the top layer.
#. Complete the wiring. Upload a PX4 Firmware of an appropriate version to the Pixfalcon onboard. Do not attach propellers to the quads until you make sure that the motors are controlled as intended.

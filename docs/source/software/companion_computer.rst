==================
Companion Computer
==================

.. meta::
    :description lang=en: Elaborated description of Companion Computer Setup.

The Companion Computer is the onboard computer or processing unit for the quads.
In this documentation we will go over the installation setup process of our
Companion Computer.

Setup
=====

The Companion Computer can be setup and ready to use by following the below
instructions for the appropriate manufacture.

ODroid - XU4
------------

.. warning::
   The ODroid companion computer will be deprecated in a future release in
   favor of the NVIDIA TX2 Jetson computer. 

This setup requires the user to clone an image of the computer OS to an eMMC
card which will be used directly on the ODroid board.

Before we begin, we will need the following:

- The Etcher USB Cloning software (https://www.balena.io/etcher/)
- Image of the OS can be found on `kessel` at
  `kessel.stanford.edu/media/data/odroid_images/` with the file name
  `20190305_MSL_Quad_ODroidKernel4_14_Ubuntu16_04_Rootless_GPIO.img`
- An eMMC card
- An eMMC to microSD adapter
- A microSD card reader

Once the image is flashed, you will also need:

- ODroid - UX4
- Monitor (with HDMI connection)
- HDMI cable
- USB keyboard/mouse
- Power supply

Installation Steps
^^^^^^^^^^^^^^^^^^

|installation|

#. Open Etcher

#. Choose "Flash from file" and choose the location of the image file mentioned
   above.

#. Select the eMMC card under the "Select Target" option.

#. Click "Flash!"

Once the flashing begins, Etcher will inform you of it progress of flashing.
After cloning has completed, Etcher will verify the image on the eMMC before
informing the user that the flash has completed. once completed, you can
unmount the eMMC card and install it on the companion computer.

Here is an example of how Etcher should look before you begin the flash.
|example|

.. |installation| image:: /_static/images/software/odroid_installation.gif
    :target: ../_static/images/software/odroid_installation.gif
    :alt: ODroid Installation

.. |example| image:: /_static/images/software/etcher_example.png
    :target: ../_static/images/software/etcher_example.png
    :alt: Etcher Example

Finalizing the Image
^^^^^^^^^^^^^^^^^^^^

Once the eMMC is installed on the ODroid, plug the keyboard, mouse and monitor
into the ODroid and turn it on. The system will boot to the Ubuntu Mate login
screen and ask for the password. Enter the password, which is the same as the
username. You will need to update a few files on the system.

#. Change the hostname in the files below to `quad#`, where # is the number of
   the quad being made.

   #. `/etc/hostname`
   #. `/etc/hosts`

.. note::
    These files should have `quad7` for the hostnames, as the image was created
    from our "Quad 7".
===============
Wiring Diagrams
===============

Interfacing an NVIDIA TX2 on an Orbitty Carrier with a Pixracer
---------------------------------------------------------------
Using the serial interface:

================= ================ ======================== ===============
On the Orbitty Carrier, connect... to the Pixracer...
---------------------------------- ----------------------------------------
**IO Header Pin** **Description**  **TELEM2 Connector Pin** **Description**
================= ================ ======================== ===============
3                 UART0 TX 3.3v    3                        RX (IN) 3.3v
4                 UART0 RX 3.3v    2                        TX (OUT) 3.3v
16, 18, 19, or 20 GND              6                        GND
================= ================ ======================== ===============

| **Resources**:
| `Orbitty Carrier <http://connecttech.com/product/orbitty-carrier-for-nvidia-jetson-tx2-tx1/>`_ board product page with technical references.
| `Pixracer <https://docs.px4.io/v1.9.0/en/flight_controller/pixracer.html>`_ documentation page.
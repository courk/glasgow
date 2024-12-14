from amaranth import *

from .qspi_decoder import QspiDecoder


class LaserPulser(Elaboratable):
    """Generates a pulse after a configurable delay."""

    def __init__(self) -> None:
        self.pulse = Signal()
        self.start = Signal()
        self.delay = Signal(16)

    def elaborate(self, platform):
        m = Module()

        counter = Signal(16)

        with m.FSM() as fsm:
            with m.State("WAIT-START"):
                with m.If(self.start):
                    m.d.sync += counter.eq(self.delay)
                    m.next = "DELAY"

            with m.State("DELAY"):
                with m.If(counter == 0):
                    m.next = "PULSE"
                with m.Else():
                    m.d.sync += counter.eq(counter - 1)

            with m.State("PULSE"):
                m.d.comb += self.pulse.eq(1)
                m.d.sync += counter.eq(counter + 1)
                with m.If(counter[5]):  # Pulse for 32 clock cycles
                    m.next = "WAIT-START"

        return m


class GlitchEngine(Elaboratable):
    """Orchestrates the attack.

    The GlitchEngine coordinates the generation of laser pulses and
    QSPI flash selection signal.
    """

    # Define flash "waypoints" for reliable triggering
    # These are tuples of (flash_address, expected_data)
    VALID_READ_WP = (0x4, 0x9F)
    FLASH_SWAP_WP = (0x14B3, 0xAB)
    MEMCPY_START_WP = (0x1, 0x20)
    MEMCPY_END_WP = (0x13D7, 0x1C)
    XIP_WP = (0x10000, 0x0)

    def __init__(self) -> None:

        # Inputs
        self.qspi_clk = Signal()
        self.qspi_data = Signal(4)
        self.qspi_cs = Signal()

        self.delay = Signal(16)

        self.start = Signal()
        self.cancel = Signal()

        # Outputs
        self.flash_selector = Signal()
        self.laser = Signal()

        # Status Signals
        self.done = Signal()
        self.running = Signal(reset=1)
        self.success = (
            Signal()
        )  # Whether additional QSPI reads have been detected have the laser pulse
        self.xip = Signal()  # Whether the XIP custom firmware has been read
        self.start_address = Signal(24)  # First QSPI address read after the laser pulse
        self.max_address = Signal(24)  # Maximum QSPI address read after the laser pulse

    def elaborate(self, platform):
        m = Module()

        #
        # QSPI Decoder
        #
        qspi_decoder = QspiDecoder()
        m.d.comb += [
            qspi_decoder.qspi_clk.eq(self.qspi_clk),
            qspi_decoder.qspi_data.eq(self.qspi_data),
            qspi_decoder.qspi_cs.eq(self.qspi_cs),
        ]
        m.submodules += qspi_decoder

        #
        # Laser Pulser
        #
        laser_pulser = LaserPulser()
        m.d.comb += [
            laser_pulser.delay.eq(self.delay),
            self.laser.eq(laser_pulser.pulse),
        ]
        m.submodules += laser_pulser

        #
        # Main FSM
        #
        with m.FSM() as fsm:

            with m.State("WAIT-START"):
                m.d.comb += self.running.eq(0)
                with m.If(self.start):
                    m.d.sync += self.flash_selector.eq(0)
                    m.d.sync += self.max_address.eq(0)
                    m.next = "WAIT-VALID-READ-WP"

            with m.State("WAIT-VALID-READ-WP"):
                with m.If(self.cancel):
                    m.next = "WAIT-START"
                with m.Elif(qspi_decoder.output_valid):
                    with m.If(
                        (qspi_decoder.address == self.VALID_READ_WP[0])
                        & (qspi_decoder.data == self.VALID_READ_WP[1])
                    ):
                        m.next = "WAIT-FLASH-SWAP-WP"

            with m.State("WAIT-FLASH-SWAP-WP"):
                with m.If(self.cancel):
                    m.next = "WAIT-START"
                with m.Elif(qspi_decoder.output_valid):
                    with m.If(
                        (qspi_decoder.address == self.FLASH_SWAP_WP[0])
                        & (qspi_decoder.data == self.FLASH_SWAP_WP[1])
                    ):
                        m.next = "WAIT-MEMCPY-START-WP"

            with m.State("WAIT-MEMCPY-START-WP"):
                with m.If(self.cancel):
                    m.next = "WAIT-START"
                with m.Elif(qspi_decoder.output_valid):
                    with m.If(
                        (qspi_decoder.address == self.MEMCPY_START_WP[0])
                        & (qspi_decoder.data == self.MEMCPY_START_WP[1])
                    ):
                        # m.d.comb += laser_pulser.start.eq(self.pre_laser)
                        m.next = "WAIT-MEMCPY-END-WP"

            with m.State("WAIT-MEMCPY-END-WP"):
                with m.If(self.cancel):
                    m.next = "WAIT-START"
                with m.Elif(qspi_decoder.output_valid):
                    with m.If(
                        (qspi_decoder.address == self.MEMCPY_END_WP[0])
                        & (qspi_decoder.data == self.MEMCPY_END_WP[1])
                    ):
                        m.d.comb += [
                            laser_pulser.start.eq(1),
                            self.done.eq(1),
                        ]
                        m.d.sync += self.flash_selector.eq(1)
                        m.next = "WAIT-SUCCESS-WP"

            with m.State("WAIT-SUCCESS-WP"):
                with m.If(self.cancel):
                    m.next = "WAIT-START"
                with m.Elif(qspi_decoder.output_valid):
                    m.next = "CHECK-XIP-LOAD"
                    m.d.sync += self.max_address.eq(qspi_decoder.address)
                    m.d.sync += self.start_address.eq(qspi_decoder.address)
                    m.d.comb += self.success.eq(1)

            with m.State("CHECK-XIP-LOAD"):
                with m.If(self.cancel):
                    m.next = "WAIT-START"
                with m.Elif(qspi_decoder.output_valid):
                    with m.If(qspi_decoder.address > self.max_address):
                        m.d.sync += self.max_address.eq(qspi_decoder.address)

        # Check if custom firmware is read
        with m.If(qspi_decoder.output_valid):
            with m.If((qspi_decoder.address == self.XIP_WP[0])):
                m.d.comb += self.xip.eq(1)

        return m

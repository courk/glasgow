import logging
import asyncio
from amaranth import *

from ....support.bits import *
from ....support.logging import *
from ....support.endpoint import *
from ....gateware.pads import *
from ... import *

from .glitch_engine import GlitchEngine


class IoController(Elaboratable):
    def __init__(self, pads, out_fifo, in_fifo):
        self.pads = pads

        self.out_fifo = out_fifo
        self.in_fifo = in_fifo

        self.srst_o = Signal(reset=0)
        self.bootsel_o = Signal(reset=1)
        self.flash_select = Signal(reset=0)

    def elaborate(self, platform):
        m = Module()

        out_fifo = self.out_fifo
        in_fifo = self.in_fifo

        #
        # Glitch engine
        #
        glitch_engine = GlitchEngine()
        m.submodules += glitch_engine
        m.d.comb += [
            glitch_engine.qspi_clk.eq(self.pads.qspi_clk_t.i),
            glitch_engine.qspi_cs.eq(self.pads.qspi_ss_t.i),
            glitch_engine.qspi_data.eq(
                Cat(
                    self.pads.qspi_d0_t.i,
                    self.pads.qspi_d1_t.i,
                    self.pads.qspi_d2_t.i,
                    self.pads.qspi_d3_t.i,
                )
            ),
        ]

        #
        # Laser is always driven and controlled by the glitch engine
        #
        m.d.comb += [
            self.pads.laser_t.oe.eq(1),
            self.pads.laser_t.o.eq(glitch_engine.laser),
        ]

        #
        # Drive RUN low when requested, when power is enabled
        #
        m.d.comb += [
            self.pads.run_t.oe.eq(~self.srst_o & self.pads.power_en_t),
            self.pads.run_t.o.eq(0),
        ]

        #
        # Drive QSPI_SS = BOOTSEL low when requested, when power is enabled
        #
        m.d.comb += [
            self.pads.qspi_ss_t.o.eq(0),
            self.pads.qspi_ss_t.oe.eq(~self.bootsel_o & self.pads.power_en_t),
        ]

        #
        # In case the glitch engine is not running,
        # If BOOTSEL is high, QSPI_SS is an input and is connected
        # to either FLASH_SS1 or FLASH_SS2
        #
        # If the glitch engine is running, FLASH_SS1 and FLASH_SS2
        # are controlled by the glitch engine
        #
        with m.If(~glitch_engine.running):
            with m.If(self.bootsel_o):
                with m.If(self.flash_select):
                    m.d.comb += [
                        self.pads.flash_ss2_t.o.eq(self.pads.qspi_ss_t.i),
                        self.pads.flash_ss2_t.oe.eq(self.pads.power_en_t),
                    ]
                with m.Else():
                    m.d.comb += [
                        self.pads.flash_ss1_t.o.eq(self.pads.qspi_ss_t.i),
                        self.pads.flash_ss1_t.oe.eq(self.pads.power_en_t),
                    ]
        with m.Else():
            with m.If(glitch_engine.flash_selector):
                m.d.comb += [
                    self.pads.flash_ss2_t.o.eq(self.pads.qspi_ss_t.i),
                    self.pads.flash_ss2_t.oe.eq(self.pads.power_en_t),
                ]
            with m.Else():
                m.d.comb += [
                    self.pads.flash_ss1_t.o.eq(self.pads.qspi_ss_t.i),
                    self.pads.flash_ss1_t.oe.eq(self.pads.power_en_t),
                ]

        #
        # POWER_EN is always driven
        #
        m.d.comb += self.pads.power_en_t.oe.eq(1)

        #
        # Handle incoming commands
        #
        config_offset = Signal(2)
        cmd_ack = Signal()

        request_data = Signal(6)

        with m.If(out_fifo.r_rdy):
            m.d.comb += out_fifo.r_en.eq(1)
            with m.If(config_offset != 0):
                with m.Switch(config_offset):
                    with m.Case(2):
                        m.d.sync += glitch_engine.delay[0:8].eq(out_fifo.r_data)
                    with m.Case(1):
                        m.d.sync += glitch_engine.delay[8:16].eq(out_fifo.r_data)
                m.d.sync += config_offset.eq(config_offset - 1)
            with m.Else():
                m.d.comb += cmd_ack.eq(1)
                with m.Switch(out_fifo.r_data):
                    with m.Case(*b"rstu"):  # Control RUN signal
                        m.d.sync += self.srst_o.eq(out_fifo.r_data - ord(b"r"))
                    with m.Case(*b"Pp"):  # Control DUT power
                        m.d.sync += self.pads.power_en_t.o.eq(~out_fifo.r_data[5])
                    with m.Case(*b"Xx"):  # Control BOOTSEL signal
                        m.d.sync += self.bootsel_o.eq(~out_fifo.r_data[5])
                    with m.Case(*b"Ff"):  # Force select flash 1/2
                        m.d.sync += self.flash_select.eq(~out_fifo.r_data[5])
                    with m.Case(*b"A"):  # Arm the glitch engine
                        m.d.comb += glitch_engine.start.eq(1)
                    with m.Case(*b"C"):  # Cancel the glitch engine
                        m.d.comb += glitch_engine.cancel.eq(1)
                    with m.Case(*b"D"):  # Configure delay
                        m.d.sync += config_offset.eq(2)
                    # Request data
                    with m.Case(*b"v"):
                        m.d.comb += request_data[0].eq(1)
                    with m.Case(*b"V"):
                        m.d.comb += request_data[1].eq(1)
                    with m.Case(*b"W"):
                        m.d.comb += request_data[2].eq(1)
                    with m.Case(*b"G"):
                        m.d.comb += request_data[3].eq(1)
                    with m.Case(*b"H"):
                        m.d.comb += request_data[4].eq(1)
                    with m.Case(*b"J"):
                        m.d.comb += request_data[5].eq(1)
                    with m.Default():
                        # Hang if an unknown command is received.
                        m.d.comb += out_fifo.r_en.eq(0)

        #
        # Control data output (Ack, success flags, ...)
        #
        with m.If(glitch_engine.done):
            m.d.comb += in_fifo.w_en.eq(1)
            m.d.comb += in_fifo.w_data.eq(b"D"[0])
        with m.Elif(glitch_engine.xip):
            m.d.comb += in_fifo.w_en.eq(1)
            m.d.comb += in_fifo.w_data.eq(b"X"[0])
        with m.Elif(glitch_engine.success):
            m.d.comb += in_fifo.w_en.eq(1)
            m.d.comb += in_fifo.w_data.eq(b"S"[0])
        with m.Elif(request_data[0]):
            m.d.comb += in_fifo.w_en.eq(1)
            m.d.comb += in_fifo.w_data.eq(glitch_engine.max_address[0:8])
        with m.Elif(request_data[1]):
            m.d.comb += in_fifo.w_en.eq(1)
            m.d.comb += in_fifo.w_data.eq(glitch_engine.max_address[8:16])
        with m.Elif(request_data[2]):
            m.d.comb += in_fifo.w_en.eq(1)
            m.d.comb += in_fifo.w_data.eq(glitch_engine.max_address[16:24])
        with m.Elif(request_data[3]):
            m.d.comb += in_fifo.w_en.eq(1)
            m.d.comb += in_fifo.w_data.eq(glitch_engine.start_address[0:8])
        with m.Elif(request_data[4]):
            m.d.comb += in_fifo.w_en.eq(1)
            m.d.comb += in_fifo.w_data.eq(glitch_engine.start_address[8:16])
        with m.Elif(request_data[5]):
            m.d.comb += in_fifo.w_en.eq(1)
            m.d.comb += in_fifo.w_data.eq(glitch_engine.start_address[16:24])

        with m.Elif(cmd_ack):
            m.d.comb += in_fifo.w_en.eq(1)
            m.d.comb += in_fifo.w_data.eq(b"A"[0])

        return m


class Rp2350LfiApplet(GlasgowApplet):
    logger = logging.getLogger(__name__)
    help = "Interface to the electronic of the RP2350 LFI project"
    description = help

    __pins = (
        "run",
        "power_en",
        "qspi_ss",
        "qspi_clk",
        "qspi_d0",
        "qspi_d1",
        "qspi_d2",
        "qspi_d3",
        "flash_ss1",
        "flash_ss2",
        "laser",
    )

    @classmethod
    def add_build_arguments(cls, parser, access):
        super().add_build_arguments(parser, access)

        for pin in cls.__pins:
            access.add_pin_argument(parser, pin, default=True)

    def build(self, target, args):
        self.mux_interface = iface = target.multiplexer.claim_interface(self, args)
        iface.add_subtarget(
            IoController(
                pads=iface.get_pads(args, pins=self.__pins),
                out_fifo=iface.get_out_fifo(),
                in_fifo=iface.get_in_fifo(),
            )
        )

    async def run(self, device, args):
        return await device.demultiplexer.claim_interface(
            self, self.mux_interface, args
        )

    @classmethod
    def add_interact_arguments(cls, parser):
        ServerEndpoint.add_argument(parser, "ctrl_endpoint")

    async def interact(self, device, args, iface):
        ctrl_endpoint = await ServerEndpoint("socket", self.logger, args.ctrl_endpoint)

        async def ctrl_forward_out():
            while True:
                try:
                    data = await ctrl_endpoint.recv()
                    await iface.write(data)
                    await iface.flush()
                except asyncio.CancelledError:
                    pass

        async def ctrl_forward_in():
            while True:
                try:
                    data = await iface.read()
                    await ctrl_endpoint.send(data)
                except asyncio.CancelledError:
                    pass

        ctrl_forward_out_fut = asyncio.ensure_future(ctrl_forward_out())
        ctrl_forward_in_fut = asyncio.ensure_future(ctrl_forward_in())

        await asyncio.wait(
            [ctrl_forward_out_fut, ctrl_forward_in_fut],
            return_when=asyncio.FIRST_EXCEPTION,
        )

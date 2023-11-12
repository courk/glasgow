import logging
import asyncio
import math
from amaranth import *

import time

from ... import *


class WiegandSubtarget(Elaboratable):
    def __init__(self, pads, in_fifo, out_fifo, pulse_width, pulse_gap):
        self.pads     = pads
        self.in_fifo  = in_fifo
        self.out_fifo = out_fifo

        self.pulse_width = pulse_width

        self.limit = pulse_gap + pulse_width

        self.bits = Signal(Shape(width = 16), reset=26)

        self.ovf = Signal()

        self.count = Signal(range(self.limit * 10 + 1))

        self.bits_data = Signal(1024, reset=0b11111111111111111111111111)
        self.bit_to_send = Signal()

    def elaborate(self, platform):
        m = Module()

        m.d.comb += self.pads.d0_t.oe.eq(1)
        m.d.comb += self.pads.d1_t.oe.eq(1)

        m.d.comb += self.ovf.eq(self.count == self.limit)

        with m.FSM() as fsm:
            with m.State("WAIT"):
                m.d.sync += self.bits.eq(0)
                m.d.sync += self.pads.d0_t.o.eq(1)
                m.d.sync += self.pads.d1_t.o.eq(1)

                m.d.sync += self.out_fifo.r_en.eq(1)
                with m.If(self.out_fifo.r_rdy):
                    m.d.sync += self.bits.eq(self.out_fifo.r_data)
                    m.next = "READ_LEN"
            with m.State("READ_LEN"):
                with m.If(self.out_fifo.r_rdy):
                    m.d.sync += self.bits.eq(Cat(self.out_fifo.r_data, self.bits))
                    m.next = "READ_DATA"
                    m.d.sync += self.out_fifo.r_en.eq(0)
            with m.State("READ_DATA"):
                m.d.sync += self.out_fifo.r_en.eq(1)
                with m.If(self.out_fifo.r_rdy):
                    m.d.sync += self.bits_data.eq(Cat(self.out_fifo.r_data, self.bits_data))

                    m.d.sync += self.count.eq(self.count + 8)

                    with m.If(self.count >= self.bits):
                        m.d.sync += self.count.eq(0)
                        m.d.sync += self.out_fifo.r_en.eq(0)
                        m.next = "PREAMBLE_START"
            with m.State("PREAMBLE_START"):
                m.d.sync += self.count.eq(self.limit * 10)
                m.next = "PREAMBLE"
            with m.State("PREAMBLE"):
                m.d.sync += self.pads.d0_t.o.eq(1)
                m.d.sync += self.pads.d1_t.o.eq(1)

                with m.If(self.count > 0):
                    m.d.sync += self.count.eq(self.count - 1)
                with m.Else():
                    m.d.sync += self.count.eq(0)

                    m.d.sync += self.bit_to_send.eq(self.bits_data)
                    m.d.sync += self.bits_data.eq(self.bits_data.shift_right(1))

                    m.next = "SEND_BITS"
            with m.State("SEND_BITS"):
                m.d.sync += self.pads.d1_t.o.eq(1)
                m.d.sync += self.pads.d0_t.o.eq(1)

                m.d.sync += self.count.eq(self.count + 1)

                with m.If(self.count > self.pulse_width):
                    m.next = "SEND_BITS_GAP"
                with m.Else():
                    with m.If(~self.bit_to_send):
                        m.d.sync += self.pads.d0_t.o.eq(0)
                    with m.Else():
                        m.d.sync += self.pads.d1_t.o.eq(0)

                with m.If(self.bits == 0):
                    m.next = "WAIT"

            with m.State("SEND_BITS_GAP"):
                m.d.sync += self.pads.d0_t.o.eq(1)
                m.d.sync += self.pads.d1_t.o.eq(1)

                with m.If(self.ovf):
                    m.d.sync += self.bits.eq(self.bits - 1)
                    m.d.sync += self.count.eq(0)

                    m.d.sync += self.bit_to_send.eq(self.bits_data)
                    m.d.sync += self.bits_data.eq(self.bits_data.shift_right(1))

                    m.next = "SEND_BITS"
                with m.Else():
                    m.d.sync += self.count.eq(self.count + 1)

        return m


class WiegandApplet(GlasgowApplet):
    logger = logging.getLogger(__name__)
    help = "boilerplate applet"
    description = """
    An example of the boilerplate code required to implement a minimal Glasgow applet.

    The only things necessary for an applet are:
        * a subtarget class,
        * an applet class,
        * the `build` and `run` methods of the applet class.

    Everything else can be omitted and would be replaced by a placeholder implementation that does
    nothing. Similarly, there is no requirement to use IN or OUT FIFOs, or any pins at all.
    """

    __pins = ("d0", "d1")

    @classmethod
    def add_build_arguments(cls, parser, access):
        super().add_build_arguments(parser, access)

        for pin in cls.__pins:
            access.add_pin_argument(parser, pin, default=True)

    def build(self, target, args):
        self.mux_interface = iface = target.multiplexer.claim_interface(self, args)

        pulse_width = self.derive_clock(
                input_hz=target.sys_clk_freq, output_hz=11904)

        pulse_gap = self.derive_clock(
                input_hz=target.sys_clk_freq, output_hz=3875)

        iface.add_subtarget(WiegandSubtarget(
            pads=iface.get_pads(args, pins=self.__pins),
            in_fifo=iface.get_in_fifo(),
            out_fifo=iface.get_out_fifo(),
            pulse_width=pulse_width,
            pulse_gap=pulse_gap
        ))

    @classmethod
    def add_run_arguments(cls, parser, access):
        super().add_run_arguments(parser, access)

        parser.add_argument(
            "-w", "--wiegand", metavar="DATA", type=str, default='1' * 26,
            help="Wiegand data to send")

    async def run(self, device, args):
        return await device.demultiplexer.claim_interface(self, self.mux_interface, args)

    @classmethod
    def add_interact_arguments(cls, parser):
        pass

    async def interact(self, device, args, iface):
        wiegand_data = args.wiegand[::-1]

        await iface.write((len(wiegand_data)).to_bytes(2, "big"))

        wiegand_binary = int(wiegand_data, 2)
        byte_count = math.ceil(len(wiegand_data) / 8)
        await iface.write(wiegand_binary.to_bytes(byte_count, "big"))

        await iface.flush()

# -------------------------------------------------------------------------------------------------

class WiegandAppletTestCase(GlasgowAppletTestCase, applet=WiegandApplet):
    @synthesis_test
    def test_build(self):
        self.assertBuilds()

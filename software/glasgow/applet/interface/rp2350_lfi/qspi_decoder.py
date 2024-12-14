from amaranth import *
from amaranth.lib.cdc import FFSynchronizer


class QspiDecoder(Elaboratable):
    """"Decodes address and data signals from a Quad Serial Peripheral Interface (QSPI) bus.

    This module supports both quad and dual I/O read operations.
    """

    def __init__(self) -> None:
        # Inputs
        self.qspi_clk = Signal()
        self.qspi_data = Signal(4)
        self.qspi_cs = Signal()

        # Outputs
        self.address = Signal(24)
        self.data = Signal(8)
        self.output_valid = Signal()

    def elaborate(self, platform):
        m = Module()

        qspi_clk = Signal()
        qspi_data = Signal(4)
        qspi_cs = Signal()

        #
        # External signals synchronization
        #
        m.submodules += FFSynchronizer(self.qspi_clk, qspi_clk)
        m.submodules += FFSynchronizer(self.qspi_data, qspi_data)
        m.submodules += FFSynchronizer(self.qspi_cs, qspi_cs)

        #
        # CS-gated rising edge detector
        #
        clk_rising_edge = Signal()
        prev_qspi_clk = Signal()

        with m.If(~prev_qspi_clk & qspi_clk & ~qspi_cs):
            m.d.comb += clk_rising_edge.eq(1)
        m.d.sync += prev_qspi_clk.eq(qspi_clk)

        #
        # Main FSM
        #
        cmd = Signal(8)
        counter = Signal(16)
        output_valid = Signal()
        address = Signal(24)

        # Delay output_valid by a cycle
        m.d.sync += [self.output_valid.eq(output_valid), self.address.eq(address)]

        with m.FSM() as fsm:

            with m.State("WAIT-COMMAND"):
                with m.If(~qspi_cs):
                    with m.If(counter != 8):
                        with m.If(clk_rising_edge):
                            m.d.sync += [
                                counter.eq(counter + 1),
                                cmd.eq(cmd << 1 | qspi_data[0]),
                            ]
                    with m.Elif(cmd == 0xEB):  # Quad read command
                        m.d.sync += counter.eq(0)
                        m.next = "READ-ADDRESS-QUAD"

                    with m.Elif(cmd == 0xBB):  # Dual I/O read command
                        m.d.sync += counter.eq(0)
                        m.next = "READ-ADDRESS-DUAL"

                with m.Else():
                    m.d.sync += counter.eq(0)

            with m.State("READ-ADDRESS-QUAD"):
                with m.If(~qspi_cs):
                    with m.If(clk_rising_edge):
                        m.d.sync += counter.eq(counter + 1)

                        with m.Switch(counter):
                            for i in range(6):
                                with m.Case(i):
                                    m.d.sync += [
                                        address[20 - 4 * i].eq(qspi_data[0]),
                                        address[21 - 4 * i].eq(qspi_data[1]),
                                        address[22 - 4 * i].eq(qspi_data[2]),
                                        address[23 - 4 * i].eq(qspi_data[3]),
                                    ]
                    with m.Elif(counter == 12):
                        m.d.sync += counter.eq(0)
                        m.next = "READ-DATA-QUAD"

                with m.Else():
                    m.d.sync += counter.eq(0)
                    m.next = "WAIT-COMMAND"

            with m.State("READ-ADDRESS-DUAL"):
                with m.If(~qspi_cs):
                    with m.If(clk_rising_edge):
                        m.d.sync += counter.eq(counter + 1)

                        with m.Switch(counter):
                            for i in range(12):
                                with m.Case(i):
                                    m.d.sync += [
                                        address[22 - 2 * i].eq(qspi_data[0]),
                                        address[23 - 2 * i].eq(qspi_data[1]),
                                    ]
                    with m.Elif(counter == 16):
                        m.d.sync += counter.eq(0)
                        m.next = "READ-DATA-DUAL"

                with m.Else():
                    m.d.sync += counter.eq(0)
                    m.next = "WAIT-COMMAND"

            with m.State("READ-DATA-QUAD"):
                with m.If(~qspi_cs):
                    with m.If(clk_rising_edge):
                        m.d.sync += counter.eq(counter + 1)

                        with m.Switch(counter & 0b1):
                            for i in range(2):
                                with m.Case(i):
                                    m.d.sync += [
                                        self.data[4 - 4 * i].eq(qspi_data[0]),
                                        self.data[5 - 4 * i].eq(qspi_data[1]),
                                        self.data[6 - 4 * i].eq(qspi_data[2]),
                                        self.data[7 - 4 * i].eq(qspi_data[3]),
                                    ]

                        with m.If(counter & 1):
                            m.d.comb += output_valid.eq(1)
                            m.d.sync += address.eq(address + 1)

                with m.Else():
                    m.d.sync += counter.eq(0)
                    m.next = "WAIT-COMMAND"

            with m.State("READ-DATA-DUAL"):
                with m.If(~qspi_cs):
                    with m.If(clk_rising_edge):
                        m.d.sync += counter.eq(counter + 1)

                        with m.Switch(counter & 0b11):
                            for i in range(4):
                                with m.Case(i):
                                    m.d.sync += [
                                        self.data[6 - 2 * i].eq(qspi_data[0]),
                                        self.data[7 - 2 * i].eq(qspi_data[1]),
                                    ]

                        with m.If(counter & 0b11 == 0b11):
                            m.d.comb += output_valid.eq(1)
                            m.d.sync += address.eq(address + 1)

                with m.Else():
                    m.d.sync += counter.eq(0)
                    m.next = "WAIT-COMMAND"

        return m

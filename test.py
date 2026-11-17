# SPDX-License-Identifier: Apache-2.0
import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

# Comandos: {sel_a[1:0], sel_b[1:0], op[2:0]}
ADD, SUB, MUL, DIV, LD, ST, NOP = 0, 1, 2, 3, 5, 6, 7


def cmd(sel_a, sel_b, op):
    return (sel_a << 5) | (sel_b << 3) | op


class Model:
    """Modelo de referencia (misma semantica que el RTL)."""

    def __init__(self):
        self.dout, self.zero, self.err = 0, 0, 0
        self.mem = [0] * 4

    def run(self, c, d1, d2, d3):
        sa, sb, op = (c >> 5) & 3, (c >> 3) & 3, c & 7
        a = [d1, d2, d3, self.dout >> 4][sa]
        b = [d1, d2, d3, self.dout & 0xF][sb]
        inv = self.err and (sa == 3 or sb == 3)
        if op in (ADD, SUB, MUL, DIV):
            if op == ADD:
                res = a + b
            elif op == SUB:
                res = a - b
            elif op == MUL:
                res = a * b
            else:
                res = a // b if b else 0xFF
            res &= 0xFF
            e = int(bool(inv) or (op == DIV and b == 0))
            self.dout = 0xFF if e else res
            self.zero, self.err = int(res == 0), e
        elif op == LD:
            self.dout, self.zero, self.err = self.mem[a & 3], 1, int(bool(inv))
        elif op == ST:
            self.mem[a & 3] = self.dout


class Cpu:
    """Maneja el bus de escritura del procesador."""

    def __init__(self, dut):
        self.dut = dut

    @property
    def rdy(self):
        return (int(self.dut.uio_out.value) >> 3) & 1

    @property
    def zero(self):
        return (int(self.dut.uio_out.value) >> 4) & 1

    @property
    def err(self):
        return (int(self.dut.uio_out.value) >> 5) & 1

    @property
    def out(self):
        return int(self.dut.uo_out.value)

    async def write(self, addr, data):
        self.dut.ui_in.value = data
        self.dut.uio_in.value = addr
        await ClockCycles(self.dut.clk, 3)
        self.dut.uio_in.value = 0b100 | addr   # wr = 1
        await ClockCycles(self.dut.clk, 4)
        self.dut.uio_in.value = addr           # wr = 0
        await ClockCycles(self.dut.clk, 4)

    async def wait_rdy(self):
        for _ in range(100):
            if self.rdy:
                return
            await ClockCycles(self.dut.clk, 1)
        raise AssertionError("cpu_rdy nunca volvio a 1")

    async def run(self, c, d1=0, d2=0, d3=0):
        await self.wait_rdy()
        await self.write(0, c)
        await self.write(1, (d2 << 4) | d1)
        await self.write(2, d3)
        await self.write(3, 0)
        await self.wait_rdy()


async def start(dut):
    dut._log.info("Start")
    cocotb.start_soon(Clock(dut.clk, 10, unit="us").start())
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)
    return Cpu(dut)


@cocotb.test()
async def test_pins_and_reset(dut):
    cpu = await start(dut)
    assert int(dut.uio_oe.value) == 0b00111000
    assert cpu.rdy == 1
    assert (cpu.out, cpu.zero, cpu.err) == (0, 0, 0)


@cocotb.test()
async def test_directed(dut):
    cpu = await start(dut)
    cases = [
        # (comando, d1, d2, d3, salida, zero, error)
        (cmd(0, 1, ADD), 7, 5, 0, 12, 0, 0),
        (cmd(0, 1, ADD), 15, 5, 0, 20, 0, 0),
        (cmd(0, 1, SUB), 9, 9, 0, 0, 1, 0),
        (cmd(0, 1, SUB), 10, 12, 0, 0xFE, 0, 0),
        (cmd(1, 2, MUL), 0, 12, 5, 60, 0, 0),
        (cmd(0, 1, MUL), 15, 15, 0, 225, 0, 0),
        (cmd(0, 2, DIV), 13, 0, 4, 3, 0, 0),
        (cmd(0, 1, DIV), 9, 0, 0, 0xFF, 0, 1),   # division por cero
        (cmd(0, 3, ADD), 5, 0, 0, 0xFF, 0, 1),   # error propagado
        (cmd(0, 1, ADD), 3, 4, 0, 7, 0, 0),      # recuperacion
        (cmd(0, 3, ADD), 2, 0, 0, 9, 0, 0),      # acumulador: 2 + 7
    ]
    for c, d1, d2, d3, eo, ez, ee in cases:
        await cpu.run(c, d1, d2, d3)
        assert (cpu.out, cpu.zero, cpu.err) == (eo, ez, ee), f"cmd={c:07b}"


@cocotb.test()
async def test_memory(dut):
    cpu = await start(dut)
    for k in range(4):
        await cpu.run(cmd(0, 1, ADD), k + 3, 3 * k)
        await cpu.run(cmd(0, 0, ST), k)
    await cpu.run(cmd(0, 1, ADD), 1, 1)
    for k in range(4):
        await cpu.run(cmd(0, 0, LD), k)
        assert cpu.out == (k + 3) + 3 * k


@cocotb.test()
async def test_random(dut):
    cpu = await start(dut)
    model = Model()
    random.seed(1234)
    for k in range(4):  # inicializa la memoria
        for c, d1, d2 in ((cmd(0, 1, ADD), k + 1, k), (cmd(0, 0, ST), k, 0)):
            await cpu.run(c, d1, d2)
            model.run(c, d1, d2, 0)
    for _ in range(200):
        c = random.randrange(128)
        d1, d2, d3 = (random.randrange(16) for _ in range(3))
        await cpu.run(c, d1, d2, d3)
        model.run(c, d1, d2, d3)
        assert (cpu.out, cpu.zero, cpu.err) == (model.dout, model.zero, model.err), \
            f"cmd={c:07b} d=({d1},{d2},{d3})"

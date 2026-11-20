"""Small instruction interpreter for the PIO subset used by this repository.

It reads the REAL .pio instructions (not a Python rewrite of their algorithm).
Unsupported instructions fail loudly. Instruction timing is in PIO cycles;
GPIO electrical effects, bus arbitration and actual CPU latency are not modelled.
This supplements code review, not a claim of general RP2350 emulation.
"""
import re
from collections import deque

MASK = 0xFFFFFFFF

class Pio:
    def __init__(self, path, out_count=26):
        # Ignore introductory C comments and the embedded C SDK initialiser.
        text = path.read_text().split('.program', 1)[1]
        self.code, self.labels = [], {}
        self.threshold, self.wrap_target, self.wrap = 1, 0, None
        for raw in text.splitlines()[1:]:
            if raw.lstrip().startswith('%'):
                break
            line = raw.split(';', 1)[0].strip().lower()
            if not line:
                continue
            if line.startswith('.mov_status'):
                self.threshold = int(line.rsplit('<', 1)[1])
            elif line == '.wrap_target':
                self.wrap_target = len(self.code)
            elif line == '.wrap':
                self.wrap = len(self.code) - 1
            elif line.startswith('.'):
                raise ValueError(f'Unsupported PIO directive: {line}')
            elif line.endswith(':'):
                self.labels[line[:-1]] = len(self.code)
            else:
                m = re.search(r'\[(\d+)\]', line)
                delay = int(m[1]) if m else 0
                line = re.sub(r'\[\d+\]', '', line).strip()
                self.code.append((line, delay))
        if self.wrap is None:
            self.wrap = len(self.code) - 1
        self.pc = self.cycles = 0
        self.reg = dict(x=0, y=0, osr=0, isr=0)
        self.tx, self.rx = deque(), deque()
        self.writes = []
        self.out_count = out_count
        self.pins = 0

    def source(self, value):
        if value.startswith('~'):
            return ~self.source(value[1:]) & MASK
        if value == 'null':
            return 0
        if value == 'status':
            return MASK if len(self.tx) < self.threshold else 0
        return self.reg[value] if value in self.reg else int(value, 0)

    def write(self, dest, value):
        value &= MASK
        if dest == 'pins':
            self.pins = value & ((1 << self.out_count) - 1)
            self.writes.append((self.cycles, self.pins))
        elif dest != 'null':
            self.reg[dest] = value

    def step(self):
        line, delay = self.code[self.pc]
        op, *rest = line.split(maxsplit=1)
        args = rest[0].replace(',', ' ').split() if rest else []
        next_pc = self.wrap_target if self.pc == self.wrap else self.pc + 1
        if op == 'mov':
            self.write(args[0], self.source(args[1]))
        elif op == 'set':
            self.write(args[0], int(args[1], 0))
        elif op == 'out':
            n = int(args[1]); value = self.reg['osr'] & ((1 << n) - 1)
            self.write(args[0], value); self.reg['osr'] >>= n
        elif op == 'pull':
            if not self.tx:
                if args and args[0] == 'noblock':
                    self.reg['osr'] = self.reg['x']
                else:
                    self.cycles += 1
                    return
            else:
                self.reg['osr'] = self.tx.popleft()
        elif op == 'push':
            if len(self.rx) < 4:
                self.rx.append((self.cycles, self.reg['isr']))
                self.reg['isr'] = 0
            elif not args or args[0] != 'noblock':
                self.cycles += 1
                return
        elif op == 'jmp':
            take = True
            if len(args) == 2:
                condition = args[0]
                if condition in ('!x', '!y'):
                    take = not self.reg[condition[1:]]
                elif condition in ('x--', 'y--'):
                    r = condition[0]; take = self.reg[r] != 0
                    self.reg[r] = (self.reg[r] - 1) & MASK
                else:
                    raise ValueError(f'Unsupported PIO condition: {line}')
            if take:
                next_pc = self.labels[args[-1]]
        elif op != 'nop':
            raise ValueError(f'Unsupported PIO instruction: {line}')
        self.pc = next_pc
        self.cycles += 1 + delay

    def until(self, predicate, budget=100000):
        for _ in range(budget):
            if predicate():
                return
            self.step()
        raise AssertionError(f'PIO instruction budget exceeded at PC={self.pc}, cycles={self.cycles}')

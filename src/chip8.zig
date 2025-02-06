const std = @import("std");

// https://github.com/mattmikolay/chip-8/wiki/CHIP%E2%80%908-Technical-Reference#fonts
// TODO integrate
pub const font_data = [_]u8{
    0xF0, 0x90, 0x90, 0x90, 0xF0,
    0x20, 0x60, 0x20, 0x20, 0x70,
    0xF0, 0x10, 0xF0, 0x80, 0xF0,
    0xF0, 0x10, 0xF0, 0x10, 0xF0,
    0x90, 0x90, 0xF0, 0x10, 0x10,
    0xF0, 0x80, 0xF0, 0x10, 0xF0,
    0xF0, 0x80, 0xF0, 0x90, 0xF0,
    0xF0, 0x10, 0x20, 0x40, 0x40,
    0xF0, 0x90, 0xF0, 0x90, 0xF0,
    0xF0, 0x90, 0xF0, 0x10, 0xF0,
    0xF0, 0x90, 0xF0, 0x90, 0x90,
    0xE0, 0x90, 0xE0, 0x90, 0xE0,
    0xF0, 0x80, 0x80, 0x80, 0xF0,
    0xE0, 0x90, 0x90, 0x90, 0xE0,
    0xF0, 0x80, 0xF0, 0x80, 0xF0,
    0xF0, 0x80, 0xF0, 0x80, 0x80,
};

pub const Display = struct {
    pub const width = 64;
    pub const height = 32;

    screen: [height]u64 = [1]u64{0} ** height,

    pub fn clear(d: *Display) void {
        @memset(&d.screen, 0);
    }

    pub fn get(d: Display, x: usize, y: usize) u1 {
        const row = d.screen[y % height];
        const bit = row >> @intCast((width - 1) - x % width);
        return @intCast(bit & 1);
    }

    /// returns if there is any collision
    pub fn blitByte(d: *Display, x: usize, y: usize, byte: u8) bool {
        const blitmask = (@as(u64, byte) << 56) >> @intCast(x % width);
        const collided = d.screen[y % height] & blitmask != 0;
        d.screen[y % height] ^= blitmask;

        // if (x > 56) {
        //     // row overflow
        //     const offset: u6 = @intCast(64 - (x - 56));
        //     const clrmask2 = (@as(u64, ~@as(u8, 0)) << 56) >> offset;
        //     const blitmask2 = @as(u64, byte) >> offset;
        //     d.screen[(y + 1) % height] ^= clrmask2;
        //     d.screen[(y + 1) % height] |= blitmask2;
        // }

        return collided;
    }
};

pub const Key = enum(u4) { _ };
pub const Keypad = struct {
    keys: [16]bool = [1]bool{false} ** 16,

    pub fn set(kp: *Keypad, key: Key, value: bool) void {
        kp.keys[@intFromEnum(key)] = value;
    }

    pub fn isPressed(kp: Keypad, key: Key) bool {
        return kp.keys[@intFromEnum(key)];
    }
};

pub const Register = enum(u4) { _ };

pub const Address = enum(u12) {
    _,

    fn add(self: Address, n: u12) Address {
        return @enumFromInt(@intFromEnum(self) + n);
    }

    fn sub(self: Address, n: u12) Address {
        return @enumFromInt(@intFromEnum(self) - n);
    }
};

pub const Opcode = std.meta.Tag(Instruction);
pub const Instruction = union(enum) {
    const Imm = struct { register: Register, byte: u8 };
    const DstSrc = struct { dst: Register, src: Register };

    sys: Address,
    cls,
    ret,
    jp: Address,
    call: Address,
    se_imm: Imm,
    sne_imm: Imm,
    se: [2]Register,
    ld_imm: Imm,
    add_imm: Imm,
    ld: DstSrc,
    @"or": DstSrc,
    @"and": DstSrc,
    xor: DstSrc,
    add: DstSrc,
    sub: DstSrc,
    shr: DstSrc,
    subn: DstSrc,
    shl: DstSrc,
    sne: [2]Register,
    ld_i: Address,
    jp_offset: Address,
    rnd: Imm,
    drw: struct { x: Register, y: Register, nbytes: u4 },
    skp: Register,
    sknp: Register,
    ld_to_dt: Register,
    ld_key: Register,
    ld_from_dt: Register,
    ld_to_st: Register,
    add_i: Register,
    ld_f: Register,
    ld_b: Register,
    ld_from_reg: Register,
    ld_to_reg: Register,

    pub const FromBytecodeError = error{UnknownInstruction};

    /// helper to decode a bytecode sequence
    const Decoder = struct {
        bytes: [2]u8,

        fn code(self: Decoder) u16 {
            return @as(u16, self.bytes[0]) << 8 | @as(u16, self.bytes[1]);
        }

        fn nibble(self: Decoder, comptime index: comptime_int) u4 {
            return switch (index) {
                0 => @intCast(self.bytes[0] >> 4),
                1 => @truncate(self.bytes[0]),
                2 => @intCast(self.bytes[1] >> 4),
                3 => @truncate(self.bytes[1]),
                else => @compileError("there are only 4 nibbles"),
            };
        }

        fn registerAt(self: Decoder, comptime index: comptime_int) Register {
            return @enumFromInt(self.nibble(index));
        }

        fn addr(self: Decoder, comptime opcode: Opcode) Instruction {
            const raw_address: u12 = @truncate(self.code());
            const address: Address = @enumFromInt(raw_address);
            return @unionInit(Instruction, @tagName(opcode), address);
        }

        fn imm(self: Decoder, comptime opcode: Opcode) Instruction {
            const immediate = Imm{
                .register = self.registerAt(1),
                .byte = self.bytes[1],
            };
            return @unionInit(Instruction, @tagName(opcode), immediate);
        }

        fn oneReg(self: Decoder, comptime opcode: Opcode) Instruction {
            const register = self.registerAt(1);
            return @unionInit(Instruction, @tagName(opcode), register);
        }

        fn twoReg(self: Decoder, comptime opcode: Opcode) Instruction {
            const reg_a = self.registerAt(1);
            const reg_b = self.registerAt(1);
            return @unionInit(Instruction, @tagName(opcode), [2]Register{ reg_a, reg_b });
        }

        fn dstSrc(self: Decoder, comptime opcode: Opcode) Instruction {
            const dst_src = DstSrc{
                .dst = self.registerAt(1),
                .src = self.registerAt(2),
            };
            return @unionInit(Instruction, @tagName(opcode), dst_src);
        }
    };

    pub fn fromBytecode(bytes: [2]u8) FromBytecodeError!Instruction {
        const decoder = Decoder{ .bytes = bytes };
        const decoded: ?Instruction = switch (decoder.nibble(0)) {
            0x0 => switch (decoder.code()) {
                0x00E0 => .cls,
                0x00EE => .ret,
                else => null,
            },
            0x1 => decoder.addr(.jp),
            0x2 => decoder.addr(.call),
            0x3 => decoder.imm(.se_imm),
            0x4 => decoder.imm(.sne_imm),
            0x5 => switch (decoder.nibble(3)) {
                0x0 => decoder.twoReg(.se),
                else => null,
            },
            0x6 => decoder.imm(.ld_imm),
            0x7 => decoder.imm(.add_imm),
            0x8 => switch (decoder.nibble(3)) {
                0x0 => decoder.dstSrc(.ld),
                0x1 => decoder.dstSrc(.@"or"),
                0x2 => decoder.dstSrc(.@"and"),
                0x3 => decoder.dstSrc(.xor),
                0x4 => decoder.dstSrc(.add),
                0x5 => decoder.dstSrc(.sub),
                0x6 => decoder.dstSrc(.shr),
                0x7 => decoder.dstSrc(.subn),
                0xE => decoder.dstSrc(.shl),
                else => null,
            },
            0x9 => switch (decoder.nibble(3)) {
                0x0 => decoder.twoReg(.sne),
                else => null,
            },
            0xA => decoder.addr(.ld_i),
            0xB => decoder.addr(.jp_offset),
            0xC => decoder.imm(.rnd),
            0xD => .{ .drw = .{
                .x = decoder.registerAt(1),
                .y = decoder.registerAt(2),
                .nbytes = decoder.nibble(3),
            } },
            0xE => switch (bytes[1]) {
                0x9E => decoder.oneReg(.skp),
                0xA1 => decoder.oneReg(.sknp),
                else => null,
            },
            0xF => switch (bytes[1]) {
                0x07 => decoder.oneReg(.ld_from_dt),
                0x0A => decoder.oneReg(.ld_key),
                0x15 => decoder.oneReg(.ld_to_dt),
                0x18 => decoder.oneReg(.ld_to_st),
                0x1E => decoder.oneReg(.add_i),
                0x29 => decoder.oneReg(.ld_f),
                0x33 => decoder.oneReg(.ld_b),
                0x55 => decoder.oneReg(.ld_to_reg),
                0x65 => decoder.oneReg(.ld_from_reg),
                else => null,
            },
        };

        if (decoded) |inst| {
            return inst;
        }

        std.debug.print("UNKNOWN INSTRUCTION: {X}\n", .{decoder.code()});

        std.process.exit(0);
    }
};

pub const memory_bytes = 0x1000;
pub const max_program_bytes = memory_bytes - @as(comptime_int, @intFromEnum(program_load_address));
pub const program_load_address: Address = @enumFromInt(0x200);

pub const Interpreter = struct {
    const Self = @This();

    pub const stack_address: Address = @enumFromInt(0x0);
    pub const font_address: Address = @enumFromInt(0x40);

    rng: std.rand.DefaultPrng,
    memory: *[memory_bytes]u8,
    registers: [16]u8 = undefined,
    /// delay timer
    dt: u8 = 0,
    /// sound timer
    st: u8 = 0,
    /// program counter
    pc: Address = undefined,
    /// stack pointer
    sp: Address = undefined,
    /// special I register
    i: Address = undefined,

    pub fn init(memory: *[memory_bytes]u8) Self {
        const time = std.time.milliTimestamp();
        const rng = std.rand.DefaultPrng.init(@bitCast(time));
        return Self{ .rng = rng, .memory = memory };
    }

    pub fn reset(self: *Self) void {
        self.dt = 0;
        self.st = 0;
        self.pc = program_load_address;
        self.sp = stack_address;

        self.write(font_address, &font_data);
    }

    pub const LoadRomError = error{
        RomTooLarge,
    };

    /// loads a rom and resets interpreter state to prepare for execution
    pub fn loadRom(self: *Self, data: []const u8) LoadRomError!void {
        if (data.len > max_program_bytes) {
            return error.RomTooLarge;
        }
        const program_memory = self.memory[@intFromEnum(program_load_address)..];
        @memcpy(program_memory[0..data.len], data);
        self.reset();
    }

    pub fn tickTimers(self: *Self) void {
        self.dt = @max(self.dt, 1) - 1;
        self.st = @max(self.st, 1) - 1;
    }

    fn setRegister(self: *Self, reg: Register, value: u8) void {
        self.registers[@intFromEnum(reg)] = value;
    }

    fn getRegister(self: Self, reg: Register) u8 {
        return self.registers[@intFromEnum(reg)];
    }

    fn read(self: Self, addr: Address, buf: []u8) void {
        const offset = @intFromEnum(addr);
        const slice = self.memory[offset .. offset + buf.len];
        @memcpy(buf, slice);
    }

    fn write(self: Self, addr: Address, data: []const u8) void {
        // TODO bounds check
        const offset = @intFromEnum(addr);
        const dst = self.memory[offset .. offset + data.len];
        @memcpy(dst, data);
    }

    fn stackPush(self: *Self, addr: Address) void {
        // TODO bounds check
        const value: u16 = @intFromEnum(addr);
        self.write(self.sp, std.mem.asBytes(&value));
        self.sp = self.sp.add(2);
    }

    fn stackPop(self: *Self) Address {
        // TODO bounds check
        self.sp = self.sp.sub(2);
        var bytes: [2]u8 = undefined;
        self.read(self.sp, &bytes);
        const value = std.mem.bytesToValue(u16, &bytes);
        return @enumFromInt(@as(u12, @intCast(value)));
    }

    pub const ExecuteError = Instruction.FromBytecodeError || error{};

    pub fn executeInstruction(
        self: *Self,
        display: *Display,
        keypad: *const Keypad,
    ) ExecuteError!void {
        var inst_bytes: [2]u8 = undefined;
        self.read(self.pc, &inst_bytes);
        self.pc = self.pc.add(2);

        const inst = try Instruction.fromBytecode(inst_bytes);
        switch (inst) {
            // control flow
            .jp => |addr| {
                self.pc = addr;
            },
            .call => |addr| {
                self.stackPush(self.pc);
                self.pc = addr;
            },
            .ret => {
                self.pc = self.stackPop();
            },
            .se_imm => |imm| {
                if (self.getRegister(imm.register) == imm.byte) {
                    self.pc = self.pc.add(2);
                }
            },
            .sne_imm => |imm| {
                if (self.getRegister(imm.register) != imm.byte) {
                    self.pc = self.pc.add(2);
                }
            },
            .sne => |regs| {
                if (self.getRegister(regs[0]) != self.getRegister(regs[1])) {
                    self.pc = self.pc.add(2);
                }
            },

            // I register manipulation
            .ld_i => |addr| {
                self.i = addr;
            },
            .add_i => |reg| {
                const offset = self.getRegister(reg);
                self.i = self.i.add(offset);
            },

            // value manipulation
            .ld_imm => |imm| self.setRegister(imm.register, imm.byte),
            .add_imm => |imm| {
                const operand = self.getRegister(imm.register);
                self.setRegister(imm.register, operand +% imm.byte);
            },
            .ld => |ds| {
                self.setRegister(ds.dst, self.getRegister(ds.src));
            },
            .@"and" => |ds| {
                const x = self.getRegister(ds.dst);
                const y = self.getRegister(ds.src);
                self.setRegister(ds.dst, x +% y);
            },
            .@"or" => |ds| {
                const x = self.getRegister(ds.dst);
                const y = self.getRegister(ds.src);
                self.setRegister(ds.dst, x & y);
            },
            .add => |ds| {
                const x = self.getRegister(ds.dst);
                const y = self.getRegister(ds.src);
                const res = @as(u16, x) + @as(u16, y);
                self.setRegister(ds.dst, @truncate(res));
                self.setRegister(@enumFromInt(0xF), @intFromBool(res > 255));
            },
            .sub => |ds| {
                const x = self.getRegister(ds.dst);
                const y = self.getRegister(ds.src);
                const not_borrow = x > y;
                self.setRegister(ds.dst, x -% y);
                self.setRegister(@enumFromInt(0xF), @intFromBool(not_borrow));
            },
            .subn => |ds| {
                const x = self.getRegister(ds.dst);
                const y = self.getRegister(ds.src);
                const not_borrow = y > x;
                self.setRegister(ds.dst, y -% x);
                self.setRegister(@enumFromInt(0xF), @intFromBool(not_borrow));
            },
            .shr => |ds| {
                const val = self.getRegister(ds.dst);
                self.setRegister(ds.dst, val >> 1);
                self.setRegister(@enumFromInt(0xF), val & 1);
            },
            .shl => |ds| {
                const val = self.getRegister(ds.dst);
                self.setRegister(ds.dst, val << 1);
                self.setRegister(@enumFromInt(0xF), (val >> 7) & 1);
            },

            // timers
            .ld_to_dt => |reg| {
                self.dt = self.getRegister(reg);
            },
            .ld_to_st => |reg| {
                self.st = self.getRegister(reg);
            },
            .ld_from_dt => |reg| {
                self.setRegister(reg, self.dt);
            },

            // keypad
            .skp => |reg| {
                const keycode = self.getRegister(reg);
                if (keypad.isPressed(@enumFromInt(keycode))) {
                    self.pc = self.pc.add(2);
                }
            },
            .sknp => |reg| {
                const keycode = self.getRegister(reg);
                if (!keypad.isPressed(@enumFromInt(keycode))) {
                    self.pc = self.pc.add(2);
                }
            },
            .ld_key => |reg| {
                for (keypad.keys, 0..) |pressed, i| {
                    if (pressed) {
                        // key pressed
                        self.setRegister(reg, @intCast(i));
                        break;
                    }
                } else {
                    // continue waiting
                    self.pc = self.pc.sub(2);
                }
            },

            // display
            .cls => display.clear(),
            .drw => |draw| {
                self.registers[0xF] = 0;

                const x = self.getRegister(draw.x);
                const y = self.getRegister(draw.y);
                for (0..draw.nbytes) |i| {
                    const row_addr = self.i.add(@intCast(i));
                    var row_byte: u8 = undefined;
                    self.read(row_addr, std.mem.asBytes(&row_byte));
                    if (display.blitByte(x, y + i, row_byte)) {
                        self.registers[0xF] = 1;
                    }
                }
            },
            .ld_f => |reg| {
                // font data for this digit
                const value = self.getRegister(reg);
                self.i = Interpreter.font_address.add(value * 5);
            },

            // misc
            .rnd => |imm| {
                const value = self.rng.random().int(u8);
                self.setRegister(imm.register, imm.byte & value);
            },
            .ld_b => |reg| {
                const value = self.getRegister(reg);
                const bcd = [3]u8{
                    value / 100,
                    (value / 10) % 10,
                    value % 10,
                };
                self.write(self.i, &bcd);
            },
            .ld_to_reg => |max_reg| {
                const src = self.registers[0 .. @intFromEnum(max_reg) + 1];
                self.write(self.i, src);
            },
            .ld_from_reg => |max_reg| {
                const dst = self.registers[0 .. @intFromEnum(max_reg) + 1];
                self.read(self.i, dst);
            },

            else => {
                std.debug.print("TODO impl {}\n", .{inst});
                std.process.exit(0);
            },
        }
    }
};

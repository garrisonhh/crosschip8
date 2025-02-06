const std = @import("std");
const chip8 = @import("chip8.zig");

const rl = @cImport({
    @cInclude("raylib.h");
});

const keymap = [16][]const c_int{
    &.{rl.KEY_ZERO},
    &.{rl.KEY_ONE},
    &.{ rl.KEY_TWO, rl.KEY_UP },
    &.{rl.KEY_THREE},
    &.{ rl.KEY_FOUR, rl.KEY_LEFT },
    &.{rl.KEY_FIVE},
    &.{ rl.KEY_SIX, rl.KEY_RIGHT },
    &.{rl.KEY_SEVEN},
    &.{ rl.KEY_EIGHT, rl.KEY_DOWN },
    &.{rl.KEY_NINE},
    &.{rl.KEY_F1},
    &.{rl.KEY_F2},
    &.{rl.KEY_F3},
    &.{rl.KEY_F4},
    &.{rl.KEY_F5},
    &.{rl.KEY_F6},
};

pub fn main() !void {
    var keypad = chip8.Keypad{};
    var display = chip8.Display{};
    display.clear();
    var interp_memory: [chip8.memory_bytes]u8 = undefined;
    var interp = chip8.Interpreter.init(&interp_memory);
    var rom_loaded = false;

    rl.InitWindow(680, 360, "chip8");
    defer rl.CloseWindow();
    rl.SetTargetFPS(60);

    rl.InitAudioDevice();
    defer rl.CloseAudioDevice();

    const sound = rl.LoadSound("res/tone.wav");
    defer rl.UnloadSound(sound);

    while (!rl.WindowShouldClose()) {
        if (rl.IsKeyPressed(rl.KEY_R)) {
            display.clear();
            interp.reset();
        }

        for (keymap, 0..) |mapped, i| {
            const is_down: bool = for (mapped) |mapped_key| {
                if (rl.IsKeyPressed(mapped_key)) break true;
            } else false;
            keypad.set(@enumFromInt(i), is_down);
        }

        if (rl.IsFileDropped()) drop_rom: {
            const dropped_files = rl.LoadDroppedFiles();
            defer rl.UnloadDroppedFiles(dropped_files);

            std.debug.assert(dropped_files.count > 0);
            const last_path_cstr = dropped_files.paths[dropped_files.count - 1];
            const last_path = last_path_cstr[0..std.mem.len(last_path_cstr)];

            // TODO load all roms in some manner..? idk
            var buf: [chip8.max_program_bytes]u8 = undefined;
            const rom = std.fs.cwd().readFile(last_path, &buf) catch |e| switch (e) {
                inline else => {
                    std.debug.print("error loading rom: {s}\n", .{@errorName(e)});
                    break :drop_rom;
                },
            };

            interp.loadRom(rom) catch unreachable;
            rom_loaded = true;
        }

        if (rom_loaded) {
            for (0..1000) |_| {
                try interp.executeInstruction(&display, &keypad);
            }
        }
        interp.tickTimers();

        if (interp.st > 0) {
            if (!rl.IsSoundPlaying(sound)) {
                rl.PlaySound(sound);
            }
        } else if (rl.IsSoundPlaying(sound)) {
            rl.StopSound(sound);
        }

        rl.BeginDrawing();
        defer rl.EndDrawing();

        rl.ClearBackground(rl.BEIGE);

        // draw
        for (0..chip8.Display.height) |y| {
            for (0..chip8.Display.width) |x| {
                const color = switch (display.get(x, y)) {
                    0 => rl.BLACK,
                    1 => rl.WHITE,
                };
                rl.DrawRectangle(@intCast(20 + 10 * x), @intCast(20 + 10 * y), 10, 10, color);
            }
        }

        rl.DrawFPS(0, 0);
    }
}

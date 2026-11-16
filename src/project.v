/*
 * Deterministic Dual-Channel RC Servo Controller - Tiny Tapeout top level
 * SPDX-License-Identifier: Apache-2.0
 *
 * Logic-level servo command generation only. Servo power and the servo's
 * internal position loop are external.
 *
 * Pinout
 *   ui_in[0]  HW_EN            uo_out[0] SERVO0          uio[0] CS_N  (in)
 *   ui_in[7:1] reserved        uo_out[1] SERVO1          uio[1] MOSI  (in)
 *                              uo_out[2] FRAME_SYNC      uio[2] MISO  (out)
 *                              uo_out[3] FAULT           uio[3] SCK   (in)
 *                              uo_out[4] WDT_EXPIRED     uio[7:4] unused
 *                              uo_out[5] ARMED
 *                              uo_out[6] UPDATE_PENDING
 *                              uo_out[7] LIMIT_HIT
 *
 * SPI mode 0. Byte 0 = {W, ADDR[6:0]}, then data bytes with address
 * auto-increment. Keep SCK <= clk/8.
 */

`default_nettype none

module tt_um_nishaangelinem_dual_servo (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered
    input  wire       clk,      // clock (10 MHz nominal)
    input  wire       rst_n     // reset_n - low to reset
);

  // ------------------------------------------------------------------
  // Register addresses
  // ------------------------------------------------------------------
  localparam [6:0] A_ID       = 7'h00, A_VERSION = 7'h01, A_CTRL    = 7'h02,
                   A_KEY      = 7'h03, A_STATUS  = 7'h04, A_FAULT   = 7'h05,
                   A_PRESCALE = 7'h06, A_WDT_TO  = 7'h07, A_WDT_KICK = 7'h08,
                   A_PERIOD_L = 7'h0A, A_PERIOD_H = 7'h0B,
                   A_MIN0_L   = 7'h0C, A_MIN0_H  = 7'h0D,
                   A_MAX0_L   = 7'h0E, A_MAX0_H  = 7'h0F,
                   A_MIN1_L   = 7'h10, A_MIN1_H  = 7'h11,
                   A_MAX1_L   = 7'h12, A_MAX1_H  = 7'h13,
                   A_POS0_L   = 7'h14, A_POS0_H  = 7'h15,
                   A_POS1_L   = 7'h16, A_POS1_H  = 7'h17,
                   A_ACT0_L   = 7'h18, A_ACT0_H  = 7'h19,
                   A_ACT1_L   = 7'h1A, A_ACT1_H  = 7'h1B,
                   A_FRAME_CNT = 7'h1C;

  localparam [7:0] ID_VALUE = 8'h5C, VERSION_VALUE = 8'h01;
  localparam [7:0] KEY_1 = 8'h5A, KEY_2 = 8'hA5, KICK_VALUE = 8'h5C;

  localparam [1:0] ST_DISABLED = 2'd0, ST_ARM_PENDING = 2'd1,
                   ST_RUN = 2'd2, ST_FAULT = 2'd3;

  // FAULT register bit positions
  localparam F_WDT = 0, F_HW_EN = 1, F_CFG = 2, F_KEY = 3,
             F_CLAMP0 = 4, F_CLAMP1 = 5, F_LOCK = 6;

  // ------------------------------------------------------------------
  // Input synchronisers
  // ------------------------------------------------------------------
  reg [2:0] sck_s;
  reg [1:0] cs_s, mosi_s, hwen_s;

  always @(posedge clk) begin
    if (!rst_n) begin
      sck_s  <= 3'b000;
      cs_s   <= 2'b11;
      mosi_s <= 2'b00;
      hwen_s <= 2'b00;
    end else begin
      sck_s  <= {sck_s[1:0], uio_in[3]};
      cs_s   <= {cs_s[0], uio_in[0]};
      mosi_s <= {mosi_s[0], uio_in[1]};
      hwen_s <= {hwen_s[0], ui_in[0]};
    end
  end

  wire sck_rise  = (sck_s[2:1] == 2'b01);
  wire sck_fall  = (sck_s[2:1] == 2'b10);
  wire cs_active = ~cs_s[1];
  wire mosi      = mosi_s[1];
  wire hw_en     = hwen_s[1];

  // ------------------------------------------------------------------
  // Registers
  // ------------------------------------------------------------------
  reg [1:0]  ch_en;        // CTRL[2:1]
  reg        stop_mode;    // CTRL[4]
  reg [5:0]  prescale;     // clk cycles per tick, minus 1
  reg [7:0]  wdt_timeout;  // frames, 0 = disabled
  reg [14:0] period;       // ticks per frame
  reg [11:0] min0, max0, min1, max1;
  reg [11:0] pos0, pos1;   // shadow (requested)
  reg [11:0] act0, act1;   // active (clamped)
  reg [7:0]  frame_cnt;
  reg [6:0]  fault;
  reg [1:0]  state;
  reg        pending;      // commit waiting for frame boundary
  reg        key_step;     // first key byte seen
  reg        limit_hit;
  reg [7:0]  wdt_cnt;

  wire locked    = (state == ST_ARM_PENDING) || (state == ST_RUN);
  wire any_fault = |fault[3:0];

  wire cfg_valid = (min0 <= max0) && (min1 <= max1) &&
                   ({3'b000, max0} < period) && ({3'b000, max1} < period);

  // ------------------------------------------------------------------
  // SPI slave (mode 0)
  // ------------------------------------------------------------------
  reg [2:0] bit_cnt;
  reg [6:0] shift_in;
  reg [7:0] shift_out;
  reg       data_phase;   // 0 = command byte, 1 = data bytes
  reg       spi_wr;
  reg [6:0] addr;
  reg [3:0] hi_latch;     // high byte snapshot for coherent 16-bit ACT reads

  wire [7:0] rx_byte   = {shift_in, mosi};
  wire       byte_done = cs_active && sck_rise && (bit_cnt == 3'd7);
  wire       wr_stb    = byte_done && data_phase && spi_wr;
  wire       load_tx   = cs_active && sck_fall && (bit_cnt == 3'd0);

  reg  [7:0] rd_data;     // combinational read mux, defined below

  always @(posedge clk) begin
    if (!rst_n) begin
      bit_cnt    <= 3'd0;
      shift_in   <= 7'd0;
      shift_out  <= 8'd0;
      data_phase <= 1'b0;
      spi_wr     <= 1'b0;
      addr       <= 7'd0;
      hi_latch   <= 4'd0;
    end else if (!cs_active) begin
      bit_cnt    <= 3'd0;
      data_phase <= 1'b0;
    end else begin
      if (sck_rise) begin
        shift_in <= rx_byte[6:0];
        bit_cnt  <= bit_cnt + 3'd1;
        if (bit_cnt == 3'd7) begin
          if (!data_phase) begin
            spi_wr     <= rx_byte[7];
            addr       <= rx_byte[6:0];
            data_phase <= 1'b1;
          end else begin
            addr <= addr + 7'd1;
          end
        end
      end
      if (sck_fall) begin
        if (bit_cnt == 3'd0) shift_out <= rd_data;
        else                 shift_out <= {shift_out[6:0], 1'b0};
      end
      if (load_tx && addr == A_ACT0_L) hi_latch <= act0[11:8];
      if (load_tx && addr == A_ACT1_L) hi_latch <= act1[11:8];
    end
  end

  // ------------------------------------------------------------------
  // Timebase: prescaler -> 1 us tick -> frame counter
  // ------------------------------------------------------------------
  reg [5:0]  presc_cnt;
  reg [14:0] us_cnt;

  wire        tick       = (presc_cnt >= prescale);
  wire [14:0] period_m1  = period - 15'd1;
  wire        frame_wrap = tick && (us_cnt >= period_m1);

  always @(posedge clk) begin
    if (!rst_n) begin
      presc_cnt <= 6'd0;
      us_cnt    <= 15'd0;
    end else begin
      presc_cnt <= tick ? 6'd0 : presc_cnt + 6'd1;
      if (frame_wrap) us_cnt <= 15'd0;
      else if (tick)  us_cnt <= us_cnt + 15'd1;
    end
  end

  // ------------------------------------------------------------------
  // Limit clamp
  // ------------------------------------------------------------------
  wire        lo0 = (pos0 < min0), hi0 = (pos0 > max0);
  wire        lo1 = (pos1 < min1), hi1 = (pos1 > max1);
  wire [11:0] clamp0 = lo0 ? min0 : hi0 ? max0 : pos0;
  wire [11:0] clamp1 = lo1 ? min1 : hi1 ? max1 : pos1;

  // ------------------------------------------------------------------
  // Control: host writes, frame-boundary actions, safety state machine
  // Later assignments take priority, so safety actions are placed last.
  // ------------------------------------------------------------------
  reg [1:0] gate;          // per-channel pulse gate, latched at frame boundary

  always @(posedge clk) begin
    if (!rst_n) begin
      ch_en       <= 2'b00;
      stop_mode   <= 1'b0;
      prescale    <= 6'd9;
      wdt_timeout <= 8'd5;
      period      <= 15'd20000;
      min0 <= 12'd1000;  max0 <= 12'd2000;
      min1 <= 12'd1000;  max1 <= 12'd2000;
      pos0 <= 12'd1500;  pos1 <= 12'd1500;
      act0 <= 12'd1500;  act1 <= 12'd1500;
      frame_cnt   <= 8'd0;
      fault       <= 7'd0;
      state       <= ST_DISABLED;
      pending     <= 1'b0;
      key_step    <= 1'b0;
      limit_hit   <= 1'b0;
      wdt_cnt     <= 8'd0;
      gate        <= 2'b00;
    end else begin

      // ---------------- host writes ----------------
      if (wr_stb) begin
        case (addr)
          A_CTRL: begin
            ch_en     <= rx_byte[2:1];
            stop_mode <= rx_byte[4];
            if (rx_byte[0]) pending <= 1'b1;
            if (rx_byte[3] && !locked) begin
              fault    <= 7'd0;
              state    <= ST_DISABLED;
              key_step <= 1'b0;
            end
          end

          A_KEY: begin
            key_step <= 1'b0;
            if (locked) begin
              state <= ST_DISABLED;            // any KEY write while armed disarms
            end else if (state == ST_DISABLED) begin
              if (rx_byte == KEY_1) begin
                key_step <= 1'b1;
              end else if (rx_byte == KEY_2 && key_step) begin
                if (!hw_en) begin
                  fault[F_HW_EN] <= 1'b1;  state <= ST_FAULT;
                end else if (!cfg_valid) begin
                  fault[F_CFG]   <= 1'b1;  state <= ST_FAULT;
                end else begin
                  state <= ST_ARM_PENDING;
                end
              end else begin
                fault[F_KEY] <= 1'b1;  state <= ST_FAULT;
              end
            end
          end

          A_WDT_KICK: if (rx_byte == KICK_VALUE) wdt_cnt <= wdt_timeout;

          A_POS0_L: pos0[7:0]  <= rx_byte;
          A_POS0_H: pos0[11:8] <= rx_byte[3:0];
          A_POS1_L: pos1[7:0]  <= rx_byte;
          A_POS1_H: pos1[11:8] <= rx_byte[3:0];

          A_PRESCALE, A_WDT_TO, A_PERIOD_L, A_PERIOD_H,
          A_MIN0_L, A_MIN0_H, A_MAX0_L, A_MAX0_H,
          A_MIN1_L, A_MIN1_H, A_MAX1_L, A_MAX1_H: begin
            if (locked) begin
              fault[F_LOCK] <= 1'b1;
            end else begin
              case (addr)
                A_PRESCALE: prescale     <= rx_byte[5:0];
                A_WDT_TO:   wdt_timeout  <= rx_byte;
                A_PERIOD_L: period[7:0]  <= rx_byte;
                A_PERIOD_H: period[14:8] <= rx_byte[6:0];
                A_MIN0_L:   min0[7:0]    <= rx_byte;
                A_MIN0_H:   min0[11:8]   <= rx_byte[3:0];
                A_MAX0_L:   max0[7:0]    <= rx_byte;
                A_MAX0_H:   max0[11:8]   <= rx_byte[3:0];
                A_MIN1_L:   min1[7:0]    <= rx_byte;
                A_MIN1_H:   min1[11:8]   <= rx_byte[3:0];
                A_MAX1_L:   max1[7:0]    <= rx_byte;
                A_MAX1_H:   max1[11:8]   <= rx_byte[3:0];
                default: ;
              endcase
            end
          end

          default: ;
        endcase
      end

      // ---------------- immediate stop option ----------------
      if (stop_mode && state != ST_RUN) gate <= 2'b00;

      // ---------------- frame boundary ----------------
      if (frame_wrap) begin
        frame_cnt <= frame_cnt + 8'd1;

        // atomic update of both channels
        if (pending) begin
          act0      <= clamp0;
          act1      <= clamp1;
          pending   <= 1'b0;
          limit_hit <= lo0 | hi0 | lo1 | hi1;
          if (lo0 | hi0) fault[F_CLAMP0] <= 1'b1;
          if (lo1 | hi1) fault[F_CLAMP1] <= 1'b1;
        end

        // pulse gates for the frame that starts now
        gate <= 2'b00;
        if (state == ST_ARM_PENDING) begin
          state   <= ST_RUN;
          wdt_cnt <= wdt_timeout;
          gate    <= ch_en;
        end else if (state == ST_RUN) begin
          gate <= ch_en;
          if (wdt_timeout != 8'd0) begin
            if (wdt_cnt <= 8'd1) begin
              fault[F_WDT] <= 1'b1;
              state        <= ST_FAULT;
              gate         <= 2'b00;
            end else if (!(wr_stb && addr == A_WDT_KICK && rx_byte == KICK_VALUE)) begin
              wdt_cnt <= wdt_cnt - 8'd1;
            end
          end
        end
      end

      // ---------------- hardware interlock (highest priority) ----------------
      if (locked && !hw_en) begin
        fault[F_HW_EN] <= 1'b1;
        state          <= ST_FAULT;
        if (frame_wrap) gate <= 2'b00;
      end
    end
  end

  // ------------------------------------------------------------------
  // Pulse generation (registered outputs, both channels rise together)
  // ------------------------------------------------------------------
  reg servo0_r, servo1_r, frame_sync_r;

  always @(posedge clk) begin
    if (!rst_n) begin
      servo0_r     <= 1'b0;
      servo1_r     <= 1'b0;
      frame_sync_r <= 1'b0;
    end else begin
      servo0_r     <= gate[0] && (us_cnt < {3'b000, act0});
      servo1_r     <= gate[1] && (us_cnt < {3'b000, act1});
      frame_sync_r <= (us_cnt == 15'd0);
    end
  end

  // ------------------------------------------------------------------
  // Read mux
  // ------------------------------------------------------------------
  always @(*) begin
    case (addr)
      A_ID:        rd_data = ID_VALUE;
      A_VERSION:   rd_data = VERSION_VALUE;
      A_CTRL:      rd_data = {3'b000, stop_mode, 1'b0, ch_en, 1'b0};
      A_STATUS:    rd_data = {any_fault, 1'b0, gate, hw_en, pending, state};
      A_FAULT:     rd_data = {1'b0, fault};
      A_PRESCALE:  rd_data = {2'b00, prescale};
      A_WDT_TO:    rd_data = wdt_timeout;
      A_PERIOD_L:  rd_data = period[7:0];
      A_PERIOD_H:  rd_data = {1'b0, period[14:8]};
      A_MIN0_L:    rd_data = min0[7:0];
      A_MIN0_H:    rd_data = {4'h0, min0[11:8]};
      A_MAX0_L:    rd_data = max0[7:0];
      A_MAX0_H:    rd_data = {4'h0, max0[11:8]};
      A_MIN1_L:    rd_data = min1[7:0];
      A_MIN1_H:    rd_data = {4'h0, min1[11:8]};
      A_MAX1_L:    rd_data = max1[7:0];
      A_MAX1_H:    rd_data = {4'h0, max1[11:8]};
      A_POS0_L:    rd_data = pos0[7:0];
      A_POS0_H:    rd_data = {4'h0, pos0[11:8]};
      A_POS1_L:    rd_data = pos1[7:0];
      A_POS1_H:    rd_data = {4'h0, pos1[11:8]};
      A_ACT0_L:    rd_data = act0[7:0];
      A_ACT0_H:    rd_data = {4'h0, hi_latch};
      A_ACT1_L:    rd_data = act1[7:0];
      A_ACT1_H:    rd_data = {4'h0, hi_latch};
      A_FRAME_CNT: rd_data = frame_cnt;
      default:     rd_data = 8'h00;
    endcase
  end

  // ------------------------------------------------------------------
  // Pin assignments
  // ------------------------------------------------------------------
  assign uo_out = {limit_hit, pending, (state == ST_RUN), fault[F_WDT],
                   any_fault, frame_sync_r, servo1_r, servo0_r};

  assign uio_out = {5'b00000, shift_out[7], 2'b00};  // uio[2] = MISO
  assign uio_oe  = 8'b0000_0100;

  // List all unused inputs to prevent warnings
  wire _unused = &{ena, ui_in[7:1], uio_in[7:4], uio_in[2], 1'b0};

endmodule

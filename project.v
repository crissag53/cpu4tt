/*
 * Procesador de 4 bits para Tiny Tapeout (version reducida del proyecto Synopsys)
 *
 * Cambios respecto al original:
 *   - WIDTH = 4 (operandos de 4 bits, resultado de 8 bits: {dout_high, dout_low})
 *   - Memoria de 4 entradas x 8 bits (antes 8 x 16)
 *   - Los 19 bits de entrada (cmd + din_1..3) se cargan por un unico bus de
 *     8 bits, escribiendo "registros" con un strobe (interfaz tipo bus)
 *   - Ejecucion de una sola operacion por "go" (antes corria continuamente)
 *   - Reset asincrono activo en bajo (rst_n) segun la plantilla de TT; los
 *     registros de operandos y de staging no tienen reset (ahorra area)
 *
 * ---------------------------- MAPA DE PINES ----------------------------
 *  ui_in[7:0]   bus de datos de escritura
 *  uio_in[1:0]  addr: registro a escribir
 *  uio_in[2]    wr: strobe de escritura (flanco de subida)
 *
 *  uo_out[7:0]  resultado {dout_high[3:0], dout_low[3:0]}
 *  uio_out[3]   cpu_rdy (1 = inactivo, listo para nueva operacion)
 *  uio_out[4]   zero    (el ultimo resultado de la ALU fue 0)
 *  uio_out[5]   error   (division por cero o dato invalido propagado)
 *
 * --------------------------- MAPA DE REGISTROS -------------------------
 *  addr 0 : CMD  = ui_in[6:0] = {sel_a[1:0], sel_b[1:0], op[2:0]}
 *  addr 1 : DIN12 = ui_in[3:0] = din_1, ui_in[7:4] = din_2
 *  addr 2 : DIN3  = ui_in[3:0] = din_3
 *  addr 3 : GO    (inicia la operacion; el dato del bus se ignora)
 *
 *  sel_*: 00 = din_1, 01 = din_2, 10 = din_3,
 *         11 = realimentacion (a: dout_high, b: dout_low)
 *  op   : 000 ADD, 001 SUB, 010 MUL, 011 DIV (cociente), 101 LD, 110 ST,
 *         111 NOP. LD/ST usan la direccion a[1:0] del operando A.
 *
 * ---------------------------- PROTOCOLO --------------------------------
 *  1) Esperar cpu_rdy = 1.
 *  2) Para cada escritura: poner addr y dato, esperar >= 3 ciclos de clk,
 *     subir wr, mantenerlo >= 3 ciclos, bajarlo y mantener addr/dato
 *     estables otros >= 3 ciclos. Escrituras con cpu_rdy = 0 se ignoran.
 *  3) Escribir CMD, DIN12, DIN3 (los que necesites) y por ultimo GO.
 *  4) Esperar a que cpu_rdy vuelva a 1 (4 ciclos). El resultado queda en
 *     uo_out y los flags en uio_out.
 *
 * La memoria no se inicializa: escribe una direccion (ST) antes de leerla (LD).
 */

`default_nettype none

// ---------------------------------------------------------------------
// ALU (4 bits -> 8 bits)
// ---------------------------------------------------------------------
module cpu4_alu #(parameter WIDTH = 4) (
    input  wire [WIDTH-1:0]   in1, in2,
    input  wire [3:0]         op,
    input  wire               invalid_data,
    output wire [2*WIDTH-1:0] out,
    output wire               zero,
    output wire               error
);
    localparam [3:0] OP_ADD = 4'b0001, OP_SUB = 4'b0010,
                     OP_MUL = 4'b0100, OP_DIV = 4'b1000;

    reg [2*WIDTH-1:0] res;
    always @(*) begin
        case (op)
            OP_ADD:  res = in1 + in2;
            OP_SUB:  res = in1 - in2;
            OP_MUL:  res = in1 * in2;
            OP_DIV:  res = (in2 != {WIDTH{1'b0}}) ? (in1 / in2) : {2*WIDTH{1'b1}};
            default: res = {2*WIDTH{1'b0}};
        endcase
    end

    assign error = invalid_data || ((in2 == {WIDTH{1'b0}}) && (op == OP_DIV));
    assign out   = error ? {2*WIDTH{1'b1}} : res;
    assign zero  = (res == {2*WIDTH{1'b0}});
endmodule

// ---------------------------------------------------------------------
// Memoria 4 x 8
// ---------------------------------------------------------------------
module cpu4_mem (
    input  wire       clk,
    input  wire       we,
    input  wire       re,
    input  wire [1:0] addr,
    input  wire [7:0] wd,
    output wire [7:0] rd
);
    reg [7:0] space [0:3];
    always @(posedge clk) if (we) space[addr] <= wd;
    assign rd = re ? space[addr] : 8'h00;
endmodule

// ---------------------------------------------------------------------
// Control (FSM): IDLE -> FETCH -> EXECUTE -> STORE -> IDLE
// ---------------------------------------------------------------------
module cpu4_control (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       start,
    input  wire [6:0] cmd_in,
    input  wire       p_error,
    output reg        aluin_reg_en,
    output reg        memoryWrite,
    output reg        memoryRead,
    output reg        selmux2,
    output reg        cpu_rdy,
    output reg        aluout_reg_en,
    output reg        nvalid_data,
    output wire [1:0] in_select_a,
    output wire [1:0] in_select_b,
    output reg  [3:0] opcode
);
    localparam [1:0] S_IDLE = 2'd0, S_FETCH = 2'd1, S_EXEC = 2'd2, S_STORE = 2'd3;
    localparam [2:0] ADD = 3'b000, SUB = 3'b001, MUL = 3'b010,
                     DIV = 3'b011, LD  = 3'b101, ST  = 3'b110;

    reg [1:0] state, next;
    wire [2:0] alu_op_bits = cmd_in[2:0];

    assign in_select_a = cmd_in[6:5];
    assign in_select_b = cmd_in[4:3];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) state <= S_IDLE;
        else        state <= next;
    end

    always @(*) begin
        next          = state;
        aluin_reg_en  = 1'b0;
        aluout_reg_en = 1'b0;
        memoryWrite   = 1'b0;
        memoryRead    = 1'b0;
        selmux2       = 1'b0;
        cpu_rdy       = 1'b0;
        nvalid_data   = 1'b0;
        opcode        = 4'b0000;

        case (state)
            S_IDLE: begin
                cpu_rdy = 1'b1;
                if (start) next = S_FETCH;
            end
            S_FETCH: begin
                aluin_reg_en = 1'b1;
                next = S_EXEC;
            end
            S_EXEC: begin
                nvalid_data   = p_error && (in_select_a == 2'b11 || in_select_b == 2'b11);
                aluout_reg_en = 1'b1;
                next = S_STORE;
                case (alu_op_bits)
                    ADD: opcode = 4'b0001;
                    SUB: opcode = 4'b0010;
                    MUL: opcode = 4'b0100;
                    DIV: opcode = 4'b1000;
                    LD: begin
                        memoryRead = 1'b1;
                        selmux2    = 1'b1;
                    end
                    ST: aluout_reg_en = 1'b0;   // la escritura ocurre en STORE
                    default: begin              // NOP
                        aluout_reg_en = 1'b0;
                        opcode        = 4'b0000;
                    end
                endcase
            end
            default: begin  // S_STORE
                memoryWrite = (alu_op_bits == ST);
                next = S_IDLE;
            end
        endcase
    end
endmodule

// ---------------------------------------------------------------------
// Top Tiny Tapeout
// ---------------------------------------------------------------------
module tt_um_crissag53_cpu4 (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);
    // ---- sincronizacion del strobe y decodificacion de escritura ----
    reg  [2:0] wr_q;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) wr_q <= 3'b000;
        else        wr_q <= {wr_q[1:0], uio_in[2]};
    end
    wire wr_rise = wr_q[1] & ~wr_q[2];
    wire [1:0] addr = uio_in[1:0];

    wire cpu_rdy;
    wire ld_cmd  = wr_rise & cpu_rdy & (addr == 2'd0);
    wire ld_d12  = wr_rise & cpu_rdy & (addr == 2'd1);
    wire ld_d3   = wr_rise & cpu_rdy & (addr == 2'd2);
    wire start   = wr_rise & cpu_rdy & (addr == 2'd3);

    // ---- registros de entrada (staging) ----
    reg [6:0] cmd_r;
    reg [7:0] d12;
    reg [3:0] d3;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)       cmd_r <= 7'b0000111;   // NOP
        else if (ld_cmd)  cmd_r <= ui_in[6:0];
    end
    always @(posedge clk) begin
        if (ld_d12) d12 <= ui_in;
        if (ld_d3)  d3  <= ui_in[3:0];
    end
    wire [3:0] din_1 = d12[3:0];
    wire [3:0] din_2 = d12[7:4];
    wire [3:0] din_3 = d3;

    // ---- control ----
    wire        aluin_en, aluout_en, mem_wr, mem_rd, sel_out, iv_data;
    wire [1:0]  sel_a, sel_b;
    wire [3:0]  alu_op;
    reg         zero_r, error_r;
    reg  [7:0]  dout_hl;

    cpu4_control ctl (
        .clk(clk), .rst_n(rst_n), .start(start), .cmd_in(cmd_r),
        .p_error(error_r),
        .aluin_reg_en(aluin_en), .memoryWrite(mem_wr), .memoryRead(mem_rd),
        .selmux2(sel_out), .cpu_rdy(cpu_rdy), .aluout_reg_en(aluout_en),
        .nvalid_data(iv_data), .in_select_a(sel_a), .in_select_b(sel_b),
        .opcode(alu_op)
    );

    // ---- registros de operandos (mux 4:1 + registro) ----
    reg [3:0] mux_a, mux_b, op_a, op_b;
    always @(*) begin
        case (sel_a)
            2'd0:    mux_a = din_1;
            2'd1:    mux_a = din_2;
            2'd2:    mux_a = din_3;
            default: mux_a = dout_hl[7:4];
        endcase
        case (sel_b)
            2'd0:    mux_b = din_1;
            2'd1:    mux_b = din_2;
            2'd2:    mux_b = din_3;
            default: mux_b = dout_hl[3:0];
        endcase
    end
    always @(posedge clk) begin
        if (aluin_en) begin
            op_a <= mux_a;
            op_b <= mux_b;
        end
    end

    // ---- ALU, memoria y registro de salida ----
    wire [7:0] res, mem_out;
    wire       alu_zero, alu_error;

    cpu4_alu #(.WIDTH(4)) alu (
        .in1(op_a), .in2(op_b), .op(alu_op), .invalid_data(iv_data),
        .out(res), .zero(alu_zero), .error(alu_error)
    );

    cpu4_mem mem (
        .clk(clk), .we(mem_wr), .re(mem_rd), .addr(op_a[1:0]),
        .wd(dout_hl), .rd(mem_out)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            zero_r  <= 1'b0;
            error_r <= 1'b0;
            dout_hl <= 8'h00;
        end else if (aluout_en) begin
            zero_r  <= alu_zero;
            error_r <= alu_error;
            dout_hl <= sel_out ? mem_out : res;
        end
    end

    // ---- salidas ----
    assign uo_out  = dout_hl;
    assign uio_out = {2'b00, error_r, zero_r, cpu_rdy, 3'b000};
    assign uio_oe  = 8'b0011_1000;

    wire _unused = &{ena, uio_in[7:3], 1'b0};
endmodule

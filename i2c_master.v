/* This I2C Master Controle Module */
module i2c_master
    #(parameter
        f_clk = 50_000_000,
        f_bus = 400_000
    )
    (
        input clk, reset,
        /* Host side */
        input ena,
        input rd_wrt,
        input [6:0] addr,
        input [7:0] data_wrt,
        output [7:0] data_rd,
        output reg busy,
        output reg err,
        /* Devices Side */
        inout scl, sda
    );

    /* Instantiate Tick Generator */
    wire clk_tick;
    reg clk_tick_ena;
    baud_gen 
    #(
        .f_clk(f_clk),
        .f_baud(f_bus)
    ) tick_gen
    (
        .clk(clk & clk_tick_ena), 
        .reset(reset | (~clk_tick_ena)),
        .oTick(clk_tick)
    );

    //////////////////////////////////
    /* FSM of I2C Master Controller */
    //////////////////////////////////
    
    /* State Declaration */
    reg [7:0] state_reg, state_next;
    localparam  s_idle = 8'h0,
                s_start = 8'h1,
                s_addr = 8'h2,
                s_slv_ack1 = 8'h3,
                s_wrt = 8'h4,
                s_rd = 8'h5,
                s_slv_ack2 = 8'h6,
                s_mst_ack = 8'h7,
                s_end = 8'h9,
                s_finish = 8'hA,
                s_err = 8'hB;
    /* Variable Declaration */
    reg [7:0] data_rd_reg, data_rd_next;
    reg [7:0] data_wrt_reg, data_wrt_next;
    reg [7:0] addr_reg, addr_next;
    reg tick_edge_reg, tick_edge_next;
    reg [7:0] bit_count_reg, bit_count_next;
    reg scl_out, sda_out;

    always@(posedge clk, posedge reset) begin
        if(reset) begin
            state_reg <= s_idle;
            data_rd_reg <= {8{1'b0}};
            data_wrt_reg <= {8{1'b0}};
            tick_edge_reg <= 1'b1;
            bit_count_reg <= 8'd7;
	    addr_reg <= 0;
        end
        else begin
            state_reg <= state_next;
            data_rd_reg <= data_rd_next;
            data_wrt_reg <= data_wrt_next;
            tick_edge_reg <= tick_edge_next;
            bit_count_reg <= bit_count_next;
	    addr_reg <= addr_next;
        end
    end
    always@* begin
        state_next = state_reg;
        data_rd_next = data_rd_reg;
        data_wrt_next = data_wrt_reg;
        tick_edge_next = tick_edge_reg;
        bit_count_next = bit_count_reg;
	addr_next = addr_reg;
        busy = 1'b1;
        clk_tick_ena = 1'b1;
        err = 1'b0;
        scl_out = 1'b1;
        sda_out = 1'b1;
        /* */
        if(clk_tick) begin
            tick_edge_next = ~tick_edge_reg;
        end
        /* */
        case(state_reg) 
            s_idle: begin
                busy = 1'b0;
                clk_tick_ena = 1'b0;
                if(ena) begin /* Enable -> start */
                    state_next = s_start;
                    addr_next = {addr, rd_wrt};
                    data_wrt_next = data_wrt;
                end
            end
            s_start: begin
                sda_out = 1'b0;
                scl_out = 1'b1;
                if(clk_tick) begin /* Start bit -> Send Addr */
                    state_next = s_addr;
                    bit_count_next = 'd7;
                end
            end
            s_addr: begin
                scl_out = tick_edge_reg;
                sda_out = addr_reg[7];
                if(clk_tick & tick_edge_reg) begin
                    addr_next = {addr_reg[6:0], addr_reg[7]}; /* Rotate */
                    if(bit_count_reg==0) begin /* Complete transmitt -> slave ack */
                        state_next = s_slv_ack1;
                    end
                    else begin /* Continue to Transmitt */
                        bit_count_next = bit_count_reg - 'd1;
                    end
                end
            end
            s_slv_ack1: begin
                sda_out = 1'b1; /* High Z and read ACK */
                scl_out = tick_edge_reg;
                if(clk_tick & tick_edge_reg) begin
                    if(~sda) begin /* ACK from Slave device */
                        if(addr_reg[0]) begin /* -> Read */
                            state_next = s_rd;
                        end
                        else begin /* -> Write */
                            state_next = s_wrt;
                        end
                        bit_count_next = 'd7;
                    end
                    else begin /* NACK -> ERR */
                        state_next = s_err;
                    end
                end
            end
            s_rd: begin
                sda_out = 1'b1; /* High Z to read input */
                scl_out = tick_edge_reg;
                if(clk_tick & tick_edge_reg) begin
                    data_rd_next = {data_rd_reg[6:0], sda};
                    if(bit_count_reg==0) begin /* Read Complete -> Master ACK */
                        state_next = s_mst_ack;
                    end
                    else begin
                        bit_count_next = bit_count_reg - 'd1;
                    end
                end
            end
            s_wrt: begin
                sda_out = data_wrt_reg[7];
                scl_out = tick_edge_reg;
                if(clk_tick & tick_edge_reg) begin
                    data_wrt_next = {data_wrt_reg[6:0], 1'b0};
                    if(bit_count_reg==0) begin /* Write Complete -> Slave ACK */
                        state_next = s_slv_ack2;
                    end
                    else begin
                        bit_count_next = bit_count_reg - 'd1;
                    end
                end
            end
            s_slv_ack2: begin
                scl_out = tick_edge_reg;
                sda_out = 1'b1;
                busy = 1'b0;
                if(clk_tick & tick_edge_reg) begin
                    if((ena) && ({addr,rd_wrt} == addr_reg) && (~sda)) begin /* Continue to write to the same address */
                        state_next = s_wrt;
                        bit_count_next = 'd7;
                        data_wrt_next = data_wrt;
                    end 
                    else begin 
                        if(~sda) 
                            state_next = s_end; /* ACK -> Stop */
                        else
                            state_next = s_err; /* NACK -> ERR */
		    end
		end
	    end
            s_mst_ack: begin 
                scl_out = tick_edge_reg;
                busy = 1'b0; /* Read Data is available during one SCL cycle */
                if((ena) && ({addr,rd_wrt} == addr_reg)) begin
                    sda_out = 1'b0; /* ACK */
                end
                else begin 
                    sda_out = 1'b1; /* NACK */
                end
                if(clk_tick & tick_edge_reg) begin
                    if((ena)&&({addr, rd_wrt} == addr_reg)) begin /* -> Continue read from the same Address */
                        state_next = s_rd;
                        bit_count_next = 'd7;
                    end
                    else begin          /* -> Stop */
                        state_next = s_end;
                    end
                end
            end
            s_end: begin
                busy = 1'b0;
                scl_out = tick_edge_reg;
                sda_out = 1'b0;
                if(clk_tick & tick_edge_reg) begin
                    state_next = s_finish;
                end
            end
            s_finish: begin /* Release Bus one more SCL Clock Cycle */
                busy = 1'b0;
                scl_out = 1'b1;
                sda_out = 1'b1;
                if(clk_tick & tick_edge_reg) begin
                    state_next = s_idle;
                end
            end 
            s_err: begin
                err = 1'b1;
                scl_out = 1'b1;
                sda_out = 1'b1;
                clk_tick_ena = 1'b0;
            end
        endcase
    end

    assign data_rd = data_rd_reg;
    assign sda = (sda_out == 1'b1) ? (1'bz) : (1'b0);
    assign scl = (scl_out == 1'b1) ? (1'bz) : (1'b0);

endmodule

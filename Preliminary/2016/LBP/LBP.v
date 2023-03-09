`timescale 1ns/10ps
module LBP ( clk, reset, gray_addr, gray_req, gray_ready, gray_data, lbp_addr, lbp_valid, lbp_data, finish);
input   	    clk;
input   	    reset;
output  [13:0] 	gray_addr;
output         	gray_req;
input   	    gray_ready;
input   [7:0] 	gray_data;
output  [13:0] 	lbp_addr;
output  	    lbp_valid;
output  [7:0] 	lbp_data;
output  	    finish;
//====================================================================
reg [13:0] gray_addr;
reg        gray_req;
reg [13:0] lbp_addr;
reg        lbp_valid;
reg [7:0]  lbp_data;
reg        finish;

reg  [2:0] cur_state;
reg  [2:0] nxt_state;
reg  [6:0] x, y;
reg  [3:0] gp_cnt;
reg  [7:0] gc_data;

reg  [13:0] gc_addr;        // Address of centrol pixel
wire [13:0] gp_addr [0:7];  // Addresses of pixels around centrol pixel

assign gp_addr[0] = {y - 7'd1, x - 7'd1}; // up left
assign gp_addr[1] = {y - 7'd1, x       }; // up
assign gp_addr[2] = {y - 7'd1, x + 7'd1}; // up right

assign gp_addr[3] = {y, x - 7'd1};   // left
assign gp_addr[4] = {y, x + 7'd1};   // right

assign gp_addr[5] = {y + 7'd1, x - 7'd1}; // down left
assign gp_addr[6] = {y + 7'd1, x       }; // down
assign gp_addr[7] = {y + 7'd1, x + 7'd1}; // down right


parameter IDLE    = 3'd0;
parameter READ_GC = 3'd1;
parameter READ_GP = 3'd2;
parameter RESULT  = 3'd3;
parameter FINISH  = 3'd4;

// update state
always @(posedge clk or posedge reset) begin
    if(reset) cur_state <= IDLE;
    else      cur_state <= nxt_state;
end

// next state logic
always @(*) begin
    case(cur_state)
        IDLE:
            if(gray_ready) nxt_state <= READ_GC;
            else           nxt_state <= IDLE;
        READ_GC:
            nxt_state <= READ_GP;
        READ_GP:
            if(gp_cnt == 4'd8) nxt_state <= RESULT;
            else               nxt_state <= READ_GP;
        RESULT:
            if(gc_addr == 14'd16254) nxt_state <= FINISH;
            else                     nxt_state <= READ_GC;
        FINISH: 
            nxt_state <= FINISH;
        default: 
            nxt_state <= IDLE;
    endcase
end


// index x and y
always @(posedge clk or posedge reset) begin
    if(reset) begin
        x <= 7'd1;
        y <= 7'd1;
    end
    else if(nxt_state == RESULT && x == 7'd126) begin
        x <= 7'd1;
        y <= y + 7'd1;
    end
    else if(nxt_state == RESULT)
        x <= x + 7'd1;
end

// gp_cnt
always @(posedge clk or posedge reset) begin
    if(reset)
        gp_cnt <= 4'd0;
    else if(nxt_state == READ_GP)
        gp_cnt <= gp_cnt + 4'd1;
    else if(cur_state == RESULT)
        gp_cnt <= 4'd0;
end

// gc_addr
always@(posedge clk or posedge reset) begin
    if(reset)
        gc_addr <= 14'd129;
    else if(nxt_state == READ_GC)
        gc_addr <= {y, x};
end


//OUTPUT
// gray_addr
always @(posedge clk or posedge reset) begin
    if(reset)
        gray_addr <= 14'd0;
    else if(nxt_state == READ_GC)
        gray_addr <= {y, x};
    else if(nxt_state == READ_GP)
        case (gp_cnt)
            4'd0: gray_addr <= gp_addr[0];
            4'd1: gray_addr <= gp_addr[1];
            4'd2: gray_addr <= gp_addr[2];
            4'd3: gray_addr <= gp_addr[3];
            4'd4: gray_addr <= gp_addr[4];
            4'd5: gray_addr <= gp_addr[5];
            4'd6: gray_addr <= gp_addr[6];
            4'd7: gray_addr <= gp_addr[7];
        endcase
end

// gray_req
always @(posedge clk or posedge reset) begin
    if(reset)
        gray_req <= 1'd0;
    else if(nxt_state == READ_GC || nxt_state == READ_GP)
        gray_req <= 1'd1;
    else
        gray_req <= 1'd0;
end

// lbp_addr
always @(posedge clk or posedge reset) begin
    if(reset)
        lbp_addr <= 14'd0;
    else if(nxt_state == RESULT)
        lbp_addr <= gc_addr;
end

// lbp_valid
always @(posedge clk or posedge reset) begin
    if(reset)
        lbp_valid <= 1'd0;
    else if(nxt_state == RESULT)
        lbp_valid <= 1'd1;
    else
        lbp_valid <= 1'd0;
end

// lbp_data and gc_data
always @(posedge clk or posedge reset) begin
    if(reset) begin
        lbp_data <= 8'd0;
        gc_data  <= 8'd0;
    end
    else if(cur_state == READ_GC)
        gc_data <= gray_data;
    else if(cur_state == READ_GP) begin
        if(gray_data >= gc_data)
            lbp_data <= lbp_data + (8'd1 << gp_cnt - 4'd1);
    end
    else if(cur_state == RESULT)
        lbp_data <= 8'd0;
end

// finish
always @(posedge clk or posedge reset) begin
    if(reset)
        finish <= 1'd0;
    else if(cur_state == FINISH)
        finish <= 1'd1;
end
//====================================================================
endmodule
module LASER (
input CLK,
input RST,
input [3:0] X,
input [3:0] Y,
output reg [3:0] C1X,
output reg [3:0] C1Y,
output reg [3:0] C2X,
output reg [3:0] C2Y,
output reg DONE);

    // State
    parameter RECEIVE        = 3'd1;
    parameter FIND_MAX_COVER = 3'd2;
    parameter CHECK_1        = 3'd3;
    parameter CHECK_2        = 3'd4;
    parameter FIX_POINT      = 3'd5;
    parameter FINISH         = 3'd6;

    reg [2:0] current_state;
    reg [2:0] next_state;

    reg [5:0] receive_count;
    reg [5:0] target_idx;                                      // dot for calculating distance 

    reg [5:0] tem_cover_count;                                 // cover count of every scan of 40 targets
    reg [5:0] max_cover_count;                                 // max cover count of every scan of 256 dots
    reg [5:0] iter_cover_count;                                // max cover count of all iteration

    wire [3:0] x_target, y_target;                     // x, y of target_idx for calculating distance
    wire [3:0] dist_x1, dist_y1;                               // abs of intercept of target_idx and search_pos
    wire [3:0] dist_x2, dist_y2;                               // abs of intercept of fix_pos and search_pos
    wire [8:0] dist_sq1, dist_sq2;                             // distance square

    reg [3:0] x, y;                                            // position for searching
    reg [3:0] x_l_limit, x_r_limit, y_u_limit, y_d_limit;      // limit of position
    reg [3:0] x_fix, y_fix;                                    // fixed position
    reg [3:0] x_max, y_max;                                    // position with fixed position have max cover count
    reg [3:0] x1_iter, y1_iter;                                // position 1
    reg [3:0] x2_iter, y2_iter;                                // position 2               // position 1 and postion 2 are the two positions which can cover most targets simultaneously after one iteration
    
    reg [3:0] Xlist [0:39];
    reg [3:0] Ylist [0:39];
    reg [8:0] iteration;

    
    // State update
    always @(posedge CLK) begin
        if(RST) current_state <= RECEIVE;
        else    current_state <= next_state;
    end

    // Next state logic
    always @(*) begin
        case (current_state)
            RECEIVE: begin
                if(receive_count >= 6'd40) next_state = FIND_MAX_COVER;
                else                       next_state = RECEIVE;
            end
            FIND_MAX_COVER: begin
                if(target_idx >= 6'd39) next_state = CHECK_1;
                else                    next_state = FIND_MAX_COVER;
            end
            CHECK_1: begin
                if(x == x_r_limit && y == y_d_limit) next_state = CHECK_2;
                else                                 next_state = FIND_MAX_COVER; 
            end
            CHECK_2: begin
                if(max_cover_count == iter_cover_count 
                    && x1_iter == x_max && y1_iter == y_max 
                    && x2_iter == x_fix && y2_iter == y_fix)
                    next_state = FINISH;
                else
                    next_state = FIX_POINT;
            end
            FIX_POINT: next_state = FIND_MAX_COVER;
            FINISH: next_state = RECEIVE;
        endcase
    end

    // receive_count
    always @(posedge CLK) begin
        if(RST || DONE)
            receive_count <= 6'd0;
        else if(current_state == RECEIVE) begin
            receive_count <= receive_count + 1'd1;
        end
    end

    // Xlist, Ylist
    always @(posedge CLK) begin
        if(current_state == RECEIVE) begin
            Xlist[receive_count] <= X;
            Ylist[receive_count] <= Y;
        end
    end

    // target_idx
    always @(posedge CLK) begin
        if(RST || DONE || target_idx == 6'd39)
            target_idx <= 6'd0;
        else if(current_state == FIND_MAX_COVER)
            target_idx <= target_idx + 1'd1;
    end

    // x, y
    always @(posedge CLK) begin
        if(RST || DONE) begin
            x <= 4'd0;
            y <= 4'd0;
        end
        else if(current_state == FIX_POINT) begin
            x <= x_l_limit;
            y <= y_u_limit;
        end
        else if(current_state == CHECK_1) begin
            if(x == x_r_limit) begin
                x <= x_l_limit;
                y <= y + 1'd1;
            end
            else
                x <= x + 1'd1;
        end
    end

    // x_l_limit, x_r_limit, y_u_limit, y_d_limit
    always @(posedge CLK) begin
        if(RST || DONE) begin
            x_l_limit <= 4'd0;
            x_r_limit <= 4'd15;
            y_u_limit <= 4'd0;
            y_d_limit <= 4'd15;
        end
        else if(next_state == FIX_POINT) begin
            x_l_limit <= (x_max > 4'd7) ? (x_max - 4'd7) : 4'd0;
            x_r_limit <= (x_max < 4'd8) ? (x_max + 4'd7) : 4'd15;
            y_u_limit <= (y_max > 4'd7) ? (y_max - 4'd7) : 4'd0;
            y_d_limit <= (y_max < 4'd8) ? (y_max + 4'd7) : 4'd15;
        end
    end

    // distance square
    assign x_target = Xlist[target_idx];
    assign y_target = Ylist[target_idx];
    assign dist_x1 = (x_target >= x) ? (x_target-x) : (x-x_target);  // abs
    assign dist_y1 = (y_target >= y) ? (y_target-y) : (y-y_target);  // abs
    assign dist_sq1 = dist_x1**2 + dist_y1**2;

    assign dist_x2 = (x_target >= x_fix) ? (x_target-x_fix) : (x_fix-x_target);
    assign dist_y2 = (y_target >= y_fix) ? (y_target-y_fix) : (y_fix-y_target);
    assign dist_sq2 = dist_x2**2 + dist_y2**2;

    // tem_cover_count
    always @(posedge CLK) begin
        if(RST || DONE || current_state == CHECK_1)
            tem_cover_count <= 6'd0;
        else if(current_state == FIND_MAX_COVER) begin
            if(iteration == 0)
                tem_cover_count <= tem_cover_count + (dist_sq1 <= 9'd16);
            else
                tem_cover_count <= tem_cover_count + ((dist_sq1 <= 9'd16) || (dist_sq2 <= 9'd16));
        end
    end

    // max_cover_count
    always @(posedge CLK) begin
        if(RST || DONE || current_state == FIX_POINT)
            max_cover_count <= 6'd0;
        else if(current_state == CHECK_1) begin
            if(tem_cover_count >= max_cover_count)
                max_cover_count <= tem_cover_count;
        end
    end

    // iter_cover_count
    always @(posedge CLK) begin
        if(RST || DONE)
            iter_cover_count <= 6'd0;
        else if(current_state == CHECK_2) begin
            if(max_cover_count >= iter_cover_count)
                iter_cover_count <= max_cover_count;
        end
    end

    // iteration
    always @(posedge CLK) begin
        if(RST || DONE)
            iteration <= 4'd0;
        else if(current_state == FIX_POINT)
            iteration <= iteration + 1'd1; 
    end

    // x_max, y_max
    always @(posedge CLK) begin
        if(RST || DONE) begin
            x_max <= 4'd0;
            y_max <= 4'd0;
        end
        else if(current_state == CHECK_1) begin
            if(tem_cover_count >= max_cover_count) begin
                x_max <= x;
                y_max <= y;
            end
        end
    end

    // x_fix, y_fix
    always @(posedge CLK) begin
        if(RST || DONE) begin
            x_fix <= 4'd0;
            y_fix <= 4'd0;
        end
        else if(next_state == FIX_POINT) begin
            x_fix <= x_max;
            y_fix <= y_max;
        end
    end

    // x1_iter, y1_iter, x2_iter, y2_iter
    always @(posedge CLK) begin
        if(RST || DONE) begin
            x1_iter <= 4'd0;
            y1_iter <= 4'd0;
            x2_iter <= 4'd0;
            y2_iter <= 4'd0;
        end
        else if(current_state == CHECK_2) begin
            if(max_cover_count >= iter_cover_count) begin
                x1_iter <= x_fix;
                y1_iter <= y_fix;
                x2_iter <= x_max;
                y2_iter <= y_max;
            end
        end
    end

    // OUTPUT 
    // C1X, C1Y, C2X, C2Y
    always @(*) begin
        C1X = x1_iter;
        C1Y = y1_iter;
        C2X = x2_iter;
        C2Y = y2_iter;
    end

    // DONE
    always @(posedge CLK) begin
        if(RST || current_state == FINISH)
            DONE <= 1'd0;
        else if(next_state == FINISH)
            DONE <= 1'd1;
    end

endmodule

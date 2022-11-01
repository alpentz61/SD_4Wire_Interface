module cmd_bus(
    sclk,
    cs,
    mosi,
    send_cmd,
    cmd_bit_in,
    cmd_bit_out,
    cmd_dir_out
);
    parameter IDLE=0;
    parameter CMD_START=1;
    parameter CMD_SEND_DATA=2;
    parameter CMD_SEND_CRC=3;
    parameter CMD_SEND_STOP=4;
    parameter DONE = 5;

    (* mark_debug = "true" *) input sclk;
    (* mark_debug = "true" *) input cs;
    (* mark_debug = "true" *) input mosi;
    (* mark_debug = "true" *) input send_cmd;
    (* mark_debug = "true" *) input cmd_bit_in;
    (* mark_debug = "true" *) output cmd_bit_out;
    (* mark_debug = "true" *) output cmd_dir_out;
    
    // output data types
    reg cmd_bit_out;
    wire cmd_dir_out;  
    
    // internal state
    (* mark_debug = "true" *)  reg [5:0] bit_index;
    (* mark_debug = "true" *)  reg [3:0] state;
   
    // CRC 7 calculation
    assign crc_bit = cmd_bit_out;
    assign crc_en = (state == CMD_SEND_DATA);
    assign crc_rst = (state == IDLE);
    wire [6:0] crc_val;
    CRC7 crc7(crc_bit, crc_en, sclk, crc_rst, crc_val);
    
    // Only drive the send command logic if CS is asserted and we are in command send mode
    assign send_active = send_cmd & ~cs;
    
    // Drive the CMD output only when a send is active
    assign cmd_dir_out = send_active;
    
    // Determine the CMD output value based on the current state
    always @(send_active, state, mosi, crc_val, bit_index) begin
        if (~send_active) begin
            cmd_bit_out = 1'b1;
        end
        
        else begin
            case (state)
                IDLE: 
                    begin
                        cmd_bit_out = 1'b0;
                    end
                CMD_SEND_DATA: 
                    begin
                        cmd_bit_out = mosi;
                    end
                CMD_SEND_CRC: 
                    begin
                        cmd_bit_out = crc_val[bit_index];
                    end
                CMD_SEND_STOP:
                    begin
                        cmd_bit_out = 1'b1;
                    end
                default: 
                    begin
                        cmd_bit_out = 1'b1;
                    end
            endcase
        end
     end
    
    // Send command state machine
    always @ (negedge sclk) begin
     
        if (~send_active) begin
            state <= IDLE;
            bit_index <= 39;
        end  
           
        else begin
                
            bit_index <= bit_index - 1;
               
            case (state)
                IDLE:  begin
                    state <= CMD_SEND_DATA;
                end

                CMD_SEND_DATA: begin                        
                    if (bit_index == 0) begin
                        // We are now latching out the last bit of the command payload,
                        // so we know the next bit will be the first crc bit
                        bit_index <= 6;
                        state <= CMD_SEND_CRC;
                    end 
                end
                
                CMD_SEND_CRC: begin
                    // We are now latching out the last bit of the crc value,
                    // so the next bit is the stop bit:
                    if (bit_index == 0) begin
                        state <= CMD_SEND_STOP;
                    end 
                            
                end  
                  
                CMD_SEND_STOP: begin 
                   state <= DONE;
                end
                
                DONE: begin
                   // stay here until CS goes high 
                end
                
           endcase
         end
    end    
endmodule
    
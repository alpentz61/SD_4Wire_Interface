module CRC7(BITVAL, Enable, CLK, RST, CRC);
   input        BITVAL;
   input        Enable;
   input        CLK;                         
   input        RST;                            
   output [6:0] CRC;                             

   reg    [6:0] CRC;   
                    
   wire         inv;
   
   assign inv = BITVAL ^ CRC[6];                  
    
    always @(posedge CLK or posedge RST) begin
		if (RST) begin
			CRC = 0;   
        end
		else begin
			if (Enable==1) begin
				CRC[6] = CRC[5];
				CRC[5] = CRC[4];
				CRC[4] = CRC[3];
				CRC[3] = CRC[2] ^ inv;
				CRC[2] = CRC[1];
				CRC[1] = CRC[0];
				CRC[0] = inv;
			end
		end
     end
   
endmodule
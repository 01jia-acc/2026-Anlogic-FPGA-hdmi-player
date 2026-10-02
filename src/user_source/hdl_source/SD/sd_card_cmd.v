// SD SPI 命令/数据块协议层。
//
// 上游 sd_card_sec_read_write 只告诉本模块“发一个命令”或“读一个数据块”；
// 本模块把命令 48 位拆成 6 个 SPI 字节，等待 R1 应答，再按 SD SPI 规则等待
// 0xFE 数据 token，接收 512 字节和 2 字节 CRC。底层 spi_master 每完成一个
// SPI 字节给出 spi_wr_ack，因此本模块的状态机粒度是“一个或多个 SPI 字节”。
//
// 这是协议层而非 FAT 层：它不知道 LBA 属于 BPB、FAT 还是 BMP，只负责可靠地
// 把一个已经给定的 SD 命令送到卡，并把返回字节交给上层。
module sd_card_cmd(
	// 系统时钟和高有效异步复位。
	input                       sys_clk,
	input                       rst,
	// 当前 SPI 字节的分频参数。
	input[15:0]                 spi_clk_div,                  //SPI module clock division parameter
	// 命令请求/完成/错误握手。
	input                       cmd_req,                      //SD card command request
	output                      cmd_req_ack,                  //SD card command request response
	output reg                  cmd_req_error,                //SD card command request error
	// 48 位命令打包总线：命令号、参数和 CRC，发送时自动给命令号加 0x40。
	input[47:0]                 cmd,                          //SD card command
	input[7:0]                  cmd_r1,                       //SD card expect response
	input[15:0]                 cmd_data_len,                 //SD card command read data length
	// 数据块读请求及输出数据流。
	input                       block_read_req,               //SD card sector data read request
	output reg                  block_read_valid,             //SD card sector data read data valid
	output reg[7:0]             block_read_data,              //SD card sector data read data
	output                      block_read_req_ack,           //SD card sector data read response
	// 数据块写请求及输入数据流。
	input                       block_write_req,              //SD card sector data write request
	input[7:0]                  block_write_data,             //SD card sector data write data next clock is valid
	output                      block_write_data_rd,          //SD card sector data write data
	output                      block_write_req_ack,          //SD card sector data write response
	// 发给 SPI master 的片选、字节请求和字节数据线。
	output                      nCS_ctrl,                     //SPI module chip select control
	output reg[15:0]            clk_div,
	output reg                  spi_wr_req,                   //SPI module data sending request
	input                       spi_wr_ack,                   //SPI module data request response
	output[7:0]                 spi_data_in,                  //SPI module send data
	input[7:0]                  spi_data_out                  //SPI module data returned
);
// 这些参数是 SPI 协议层状态编码。S_WAIT 等待上游事务，S_CMD* 发送命令，
// S_READ* 接收单块，S_WRITE* 发送单块，S_END/S_ERR 给出事务结果。
parameter S_IDLE         = 0;
parameter S_WAIT         = 1;
parameter S_INIT         = 2;
parameter S_CMD_PRE      = 3;
parameter S_CMD          = 4;
parameter S_CMD_DATA     = 5;
parameter S_READ_WAIT    = 6;
parameter S_READ         = 7;
parameter S_READ_ACK     = 8;
parameter S_WRITE_TOKEN  = 9;
parameter S_WRITE_DATA_0 = 10;
parameter S_WRITE_DATA_1 = 11;
parameter S_WRITE_CRC    = 12;
parameter S_WRITE_SUC    = 13;
parameter S_WRITE_BUSY   = 14;
parameter S_WRITE_ACK    = 15;
parameter S_ERR          = 16;
parameter S_END          = 17;

// 协议状态寄存器；所有 SPI 字节发送都由它决定当前 send_data 的含义。
reg[4:0]                      state;
// 当前命令/数据阶段的字节计数。
reg                           CS_reg;
reg[15:0]                     byte_cnt;
// 送入 SPI master 的待发送字节。
reg[7:0]                      send_data;
// SPI master 返回的完整字节（组合别名）。
wire[7:0]                     data_recv;
// 写数据已经发送了多少字节；10 位覆盖 0~512。
reg[9:0]                      wr_data_cnt;

// 握手输出都是由状态解码得到的组合事件：状态机在 ACK 状态停留一个周期，
// 上游就在该周期看到完成；block_read_valid 在下面独立时序块中产生。
assign cmd_req_ack = (state == S_END);
assign block_read_req_ack = (state == S_READ_ACK);
assign block_write_req_ack= (state == S_WRITE_ACK);
assign block_write_data_rd = (state == S_WRITE_DATA_0);
assign spi_data_in = send_data;
assign data_recv = spi_data_out;
assign nCS_ctrl = CS_reg;
// 协议主状态机。spi_wr_req 是电平请求：保持 1，直到 spi_wr_ack 表示一个
// SPI 字节已经完成；这样不会因为双方时钟拍不同而漏掉字节事务。
always@(posedge sys_clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		CS_reg <= 1'b1;
		spi_wr_req <= 1'b0;
		byte_cnt <= 16'd0;
		clk_div <= 16'd0;
		send_data <= 8'hff;
		state <= S_IDLE;
		cmd_req_error <= 1'b0;
		wr_data_cnt <= 10'd0;
	end
	else
		case(state)
			S_IDLE:
			begin
				// 上电后先发送至少 74 个空闲时钟，再等待命令。
				state <= S_INIT;
				clk_div <= spi_clk_div;
				CS_reg <= 1'b1;
			end
			S_INIT:
			begin
				// 连续发送 0xFF，释放 MOSI 并给 SD 卡提供初始化时钟。
				//send 11 bytes on power(at least 74 SPI clocks)
				if(spi_wr_ack == 1'b1)
				begin
					if(byte_cnt >= 16'd10)
					begin
						byte_cnt <= 16'd0;
						spi_wr_req <= 1'b0;
						state <= S_WAIT;
					end
					begin
						byte_cnt <= byte_cnt + 16'd1;
					end
				end
				else
				begin
					spi_wr_req <= 1'b1;
					send_data <= 8'hff;
				end
			end
			S_WAIT:
			begin
				// 一个事务完成后回到这里，命令/读/写三类请求互斥选择。
				cmd_req_error <= 1'b0;
				wr_data_cnt <= 10'd0;
				//wait for  instruction
				if(cmd_req == 1'b1)
					state <= S_CMD_PRE;
				else if(block_read_req == 1'b1)
					state <= S_READ_WAIT;
				else if(block_write_req == 1'b1)
					state <= S_WRITE_TOKEN;
				clk_div <= spi_clk_div;
			end
			S_CMD_PRE:
			begin
				// 命令前额外发送 0xFF，提供片选切换后的空闲时钟。
				//before sending a command, send an byte 'ff',provide some clocks
				if(spi_wr_ack == 1'b1)
				begin
					state <= S_CMD;
					spi_wr_req <= 1'b0;
					byte_cnt <= 16'd0;
				end
				else
				begin
					spi_wr_req <= 1'b1;
					CS_reg <= 1'b1;
					send_data <= 8'hff;
				end
			end
			S_CMD:
			begin
				// 依次发 6 字节命令，并轮询 R1；byte_cnt=0~5 对应 cmd[47:0]。
				if(spi_wr_ack == 1'b1)
				begin
					if((byte_cnt == 16'hffff) || (data_recv != cmd_r1 && data_recv[7] == 1'b0))
					begin
						state <= S_ERR;
						spi_wr_req <= 1'b0;
						byte_cnt <= 16'd0;
					end
					else if(data_recv == cmd_r1)
					begin
						spi_wr_req <= 1'b0;
						if(cmd_data_len != 16'd0)
						begin
							state <= S_CMD_DATA;
							byte_cnt <= 16'd0;
						end
						else
						begin
							state <= S_END;
							byte_cnt <= 16'd0;
						end
					end
					else
						byte_cnt <=  byte_cnt + 16'd1;
				end
				else
				begin
					spi_wr_req <= 1'b1;
					CS_reg <= 1'b0;
					if(byte_cnt == 16'd0)
						send_data <= (cmd[47:40] | 8'h40);
					else if(byte_cnt == 16'd1)
						send_data <= cmd[39:32];
					else if(byte_cnt == 16'd2)
						send_data <= cmd[31:24];
					else if(byte_cnt == 16'd3)
						send_data <= cmd[23:16];
					else if(byte_cnt == 16'd4)
						send_data <= cmd[15:8];
					else if(byte_cnt == 16'd5)
						send_data <= cmd[7:0];
					else
						send_data <= 8'hff;
				end
			end
			S_CMD_DATA:
			begin
				// CMD8 的 R7 等附加响应由这里读取并丢弃，只用长度控制收完。
				if(spi_wr_ack == 1'b1)
				begin
					if(byte_cnt == cmd_data_len - 16'd1)
					begin
						state <= S_END;
						spi_wr_req <= 1'b0;
						byte_cnt <= 16'd0;
					end
					else
					begin
						byte_cnt <= byte_cnt + 16'd1;
					end
				end
				else
				begin
					spi_wr_req <= 1'b1;
					send_data <= 8'hff;
				end
			end
			S_READ_WAIT:
			begin
				// CMD17 完成后，卡可能需要若干个 0xFF 时钟才返回 0xFE token。
				if(spi_wr_ack == 1'b1 && data_recv == 8'hfe)
				begin
					spi_wr_req <= 1'b0;
					state <= S_READ;
					byte_cnt <= 16'd0;
				end
				else
				begin
					spi_wr_req <= 1'b1;
					send_data <= 8'hff;
				end
			end
			S_READ:
			begin
				// token 后读取数据区和 CRC；前 512 个返回字节作为有效数据，
				// 后两个字节只是时钟/CRC，最终在 byte_cnt=513 完成块事务。
				if(spi_wr_ack == 1'b1)
				begin
					if(byte_cnt == 16'd513)
					begin
						state <= S_READ_ACK;
						spi_wr_req <= 1'b0;
						byte_cnt <= 16'd0;
					end
					else
					begin
						byte_cnt <= byte_cnt + 16'd1;
					end
				end
				else
				begin
					spi_wr_req <= 1'b1;
					send_data <= 8'hff;
				end
			end
			S_WRITE_TOKEN:
				// 单块写以 0xFE token 开始。
				if(spi_wr_ack == 1'b1)
				begin
					state <= S_WRITE_DATA_0;
					spi_wr_req <= 1'b0;
					byte_cnt <= 16'd0;  
				end
				else
				begin
					spi_wr_req <= 1'b1;
					send_data <= 8'hfe;
				end
			S_WRITE_DATA_0:
				// 这个中间态产生 block_write_data_rd，向上游索取一个新字节。
			begin
				state <= S_WRITE_DATA_1;
				wr_data_cnt <= wr_data_cnt + 10'd1;
			end
			S_WRITE_DATA_1:
				// 等待并发送上游给出的写数据。
			begin
				if(spi_wr_ack == 1'b1 && wr_data_cnt == 10'd512)
				begin
					state <= S_WRITE_CRC;
					spi_wr_req <= 1'b0;
				end
				else if(spi_wr_ack == 1'b1)
				begin
					state <= S_WRITE_DATA_0;
					spi_wr_req <= 1'b0;
				end
				else
				begin
					spi_wr_req <= 1'b1;
					send_data <= block_write_data;
				end
			end
			S_WRITE_CRC:
				// 发送两个占位 CRC 字节；SD SPI 模式下通常允许任意 CRC。
			begin
				if(spi_wr_ack == 1'b1)
				begin
					if(byte_cnt == 16'd1)
					begin
						state <= S_WRITE_SUC;
						spi_wr_req <= 1'b0;
						byte_cnt <= 16'd0;
					end
					else
					begin
						byte_cnt <= byte_cnt + 16'd1;
					end
				end
				else
				begin
					spi_wr_req <= 1'b1;
					send_data <= 8'hff;
				end
			end
			S_WRITE_SUC :
				// 等待卡返回数据响应 token，00101 表示 accepted。
			begin
				if(spi_wr_ack == 1'b1)
				begin
					if(data_recv[4:0] == 5'b00101)
					begin
						state <= S_WRITE_BUSY;
						spi_wr_req <= 1'b0;
					end
				end
				else
				begin
					spi_wr_req <= 1'b1;
					send_data <= 8'hff;
				end
			end
			S_WRITE_BUSY :
				// 卡内部编程期间返回非 0xFF；回到 0xFF 后写事务结束。
			begin
				if(spi_wr_ack == 1'b1)
				begin
					if(data_recv == 8'hff)
					begin
						state <= S_WRITE_ACK;
						spi_wr_req <= 1'b0;
					end
				end
				else
				begin
					spi_wr_req <= 1'b1;
					send_data <= 8'hff;
				end
			end
			S_ERR:
				// 将错误锁存一个周期，再通过 S_END 告知上层请求失败。
			begin
				state <= S_END;
				cmd_req_error <= 1'b1;
			end
			S_READ_ACK,S_WRITE_ACK,S_END:
			begin
				state <= S_WAIT;
			end
			default:
				state <= S_IDLE;
		endcase
end
	// 读数据有效脉冲：只在 S_READ 且 byte_cnt<512 时把 SPI 字节交给上层。
	always@(posedge sys_clk or posedge rst)
begin
	if(rst == 1'b1)
		block_read_valid <= 1'b0;
	else if(state == S_READ && byte_cnt < 16'd512)
		block_read_valid <= spi_wr_ack;
	else
		block_read_valid <= 1'b0;
end

	// 数据寄存器在 SPI 字节完成的时钟沿锁存 MISO 结果，保证 valid 同拍稳定。
	always@(posedge sys_clk or posedge rst)
begin
	if(rst == 1'b1)
		block_read_data <= 8'd0;
	else if(state == S_READ && spi_wr_ack == 1'b1)
		block_read_data <= data_recv;
end

endmodule

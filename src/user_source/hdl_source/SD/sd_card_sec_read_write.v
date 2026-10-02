// SD 扇区事务控制器：把“初始化 SD 卡”和“读/写一个 512 字节扇区”抽象成
// 上层容易使用的接口。它不直接产生 SPI 波形，而是向 sd_card_cmd 提交命令，
// 再等待命令完成或数据块完成。
//
// 初始化主线：CMD0 -> CMD8 -> CMD55/CMD41 循环等待卡离开 idle -> CMD16(512B)。
// 读主线：sd_sec_read -> CMD17 -> block_read_req -> 512 字节 -> sd_sec_read_end。
// 本工程是 SDHC/FAT32 读卡，sd_sec_read_addr 按逻辑块地址 LBA 使用。
module sd_card_sec_read_write
#(
	parameter  SPI_LOW_SPEED_DIV = 248,         // spi clk speed = clk speed /((SPI_LOW_SPEED_DIV + 2) * 2 )
	parameter  SPI_HIGH_SPEED_DIV = 0           // spi clk speed = clk speed /((SPI_HIGH_SPEED_DIV + 2) * 2 )
)

(
	// 系统时钟/高有效异步复位。
	input            clk,
	input            rst,
	// 初始化完成电平；只有为 1 时上层才会发扇区请求。
	output reg       sd_init_done,
	// 上层单扇区读写请求及地址/写数据。
	input            sd_sec_read,
	input[31:0]      sd_sec_read_addr,
	output[7:0]      sd_sec_read_data,
	output           sd_sec_read_data_valid,
	output           sd_sec_read_end,
	input            sd_sec_write,
	input[31:0]      sd_sec_write_addr,
	input[7:0]       sd_sec_write_data,
	output           sd_sec_write_data_req,
	output           sd_sec_write_end,

	// 发给 sd_card_cmd 的命令事务接口。
	output reg[15:0] spi_clk_div,
	output reg       cmd_req,
	input            cmd_req_ack,
	input            cmd_req_error,
	output reg[47:0] cmd,
	output reg[7:0]  cmd_r1,
	output reg[15:0] cmd_data_len,
	output reg       block_read_req,
	input            block_read_valid,
	input[7:0]       block_read_data,
	input            block_read_req_ack,
	output reg       block_write_req,
	output[7:0]      block_write_data,
	input            block_write_data_rd,
	input            block_write_req_ack,

	// 调试观察口：高层 SD 事务状态和最近一次命令错误。
	output wire[4:0]  debug_state,
	output wire       debug_cmd_req,
	output wire       debug_cmd_ack,
	output wire       debug_cmd_error,
	output wire       debug_block_read_req,
	output wire       debug_block_read_ack,
	output reg        debug_cmd_error_latched
);
// 读数据暂存和历史遗留计时器；当前状态机主要通过 block_read_data 直接转发，
// 这两个寄存器在本版本中不参与主要控制，可作为后续超时保护的扩展点。
reg[7:0] read_data;
reg[31:0] timer;

// 初始化、读写和完成阶段的编码。编码本身没有“执行顺序”，只是 state
// 触发器的取值；真正的转移条件写在下面的时序 case 中。
localparam S_IDLE               = 0;
localparam S_CMD0               = 1;
localparam S_CMD8               = 2;
localparam S_CMD55              = 3;
localparam S_CMD41              = 4;
localparam S_CMD17              = 5;
localparam S_READ               = 6;
localparam S_CMD24              = 7;
localparam S_WRITE              = 8;
localparam S_ERR                = 14;
localparam S_WRITE_END          = 15;
localparam S_READ_END           = 16;
localparam S_WAIT_READ_WRITE    = 17;
localparam S_CMD16              = 18;

// 主状态寄存器，保存当前 SD 高层事务阶段。
reg[4:0]                       state;
// 当前读/写事务的扇区地址锁存器；请求输入可能只脉冲一拍，所以必须保存。
reg[31:0]                      sec_addr;
// 这些连续赋值把底层块事务翻译成上层的 valid/end/data_req 事件。
assign sd_sec_read_data_valid = (state == S_READ) && block_read_valid;
assign sd_sec_read_data = block_read_data;
assign sd_sec_read_end = (state == S_READ_END);
assign sd_sec_write_data_req = (state == S_WRITE) && block_write_data_rd;
assign block_write_data = sd_sec_write_data;
assign sd_sec_write_end = (state == S_WRITE_END);
assign debug_state = state;
assign debug_cmd_req = cmd_req;
assign debug_cmd_ack = cmd_req_ack;
assign debug_cmd_error = cmd_req_error;
assign debug_block_read_req = block_read_req;
assign debug_block_read_ack = block_read_req_ack;

// SD 高层时序状态机。cmd_req/block_read_req/block_write_req 采用电平握手：
// 保持为 1 直到下级给出 ack，再清零并进入下一阶段。
always@(posedge clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		state <= S_IDLE;
		cmd_req <= 1'b0;
		cmd_data_len <= 16'd0;
		cmd_r1 <= 8'd0;
		cmd <= 48'd0;
		spi_clk_div <= SPI_LOW_SPEED_DIV[15:0];
		block_write_req <= 1'b0;
		block_read_req <= 1'b0;
		sec_addr <= 32'd0;
		sd_init_done <= 1'b0;
		debug_cmd_error_latched <= 1'b0;
	end
	else begin
		// 命令错误通常只有一个时钟周期，锁存后供低速 UART 周期采样。
		// ACMD41 在卡尚未准备好时返回非零响应是正常轮询过程，不能把
		// 初始化阶段的暂态错误误报成运行时故障；这里只记录初始化完成后的
		// CMD17/CMD24 等业务阶段错误。
		if (cmd_req_error && sd_init_done)
			debug_cmd_error_latched <= 1'b1;
		case(state)
			S_IDLE:
			begin
				// 上电后先启动初始化，不直接接受上层读请求。
				state <= S_CMD0;
				sd_init_done <= 1'b0;
				spi_clk_div <= SPI_LOW_SPEED_DIV[15:0];
			end
			S_CMD0:
			begin
				// CMD0 让卡进入 SPI idle 状态，R1 应答为 0x01。
				if(cmd_req_ack & ~cmd_req_error)
				begin
					state <= S_CMD8;
					cmd_req <= 1'b0;
				end
				else
				begin
					cmd_req <= 1'b1;
					cmd_data_len <= 16'd0;
					cmd_r1 <= 8'h01;
					cmd <= {8'd0,8'h00,8'h00,8'h00,8'h00,8'h95};
				end
			end
			S_CMD8:
			begin
				// CMD8 检查电压范围/卡版本，并读取 4 字节回显数据。
				if(cmd_req_ack & ~cmd_req_error)
				begin
					state <= S_CMD55;
					cmd_req <= 1'b0;
				end
				else
				begin
					cmd_req <= 1'b1;
					cmd_data_len <= 16'd4;
					cmd_r1 <= 8'h01;
					cmd <= {8'd8,8'h00,8'h00,8'h01,8'haa,8'h87};
				end
			end
			S_CMD55:
			begin
				// CMD55 是 ACMD41 前缀；每次 ACMD41 轮询前都要重新发送。
				if(cmd_req_ack & ~cmd_req_error)
				begin
					state <= S_CMD41;
					cmd_req <= 1'b0;
				end
				else
				begin
					cmd_req <= 1'b1;
					cmd_data_len <= 16'd0;
					cmd_r1 <= 8'h01;
					cmd <= {8'd55,8'h00,8'h00,8'h00,8'h00,8'hff};
				end
			end
			S_CMD41:
			begin
				// ACMD41 带 HCS 位等待卡完成上电初始化；R1=0 表示 ready。
				if(cmd_req_ack & ~cmd_req_error)
				begin
					state <= S_CMD16;
					cmd_req <= 1'b0;
					// ACMD41 成功只表示卡退出 idle，CMD16 仍未完成；
					// 此时不能向上层宣告“可以读扇区”。
					sd_init_done <= 1'b0;
					spi_clk_div <= SPI_HIGH_SPEED_DIV[15:0];
				end
				else if(cmd_req_ack)
				begin
					state <= S_CMD55;
				end
				else
				begin
					cmd_req <= 1'b1;
					cmd_data_len <= 16'd0;
					cmd_r1 <= 8'h00;
					cmd <= {8'd41,8'h40,8'h00,8'h00,8'h00,8'hff};
				end
			end
			S_CMD16:
			begin
				// 固定块长为 512 字节，之后进入可服务读写请求的等待态。
				if(cmd_req_ack & ~cmd_req_error)
				begin
					state <= S_WAIT_READ_WRITE;
					cmd_req <= 1'b0;
					sd_init_done <= 1'b1;
					spi_clk_div <= SPI_HIGH_SPEED_DIV[15:0];
				end
				else if(cmd_req_ack)
				begin
					state <= S_CMD55;
				end
				else
				begin
					cmd_req <= 1'b1;
					cmd_data_len <= 16'd0;
					cmd_r1 <= 8'h00;
					cmd <= {8'd16,32'd512,8'hff};
				end
			end			
			S_WAIT_READ_WRITE:
			begin
				// 读写请求互斥；读优先级在这里由 if/else 的顺序决定。
				if(sd_sec_write ==  1'b1)
				begin
					state <= S_CMD24;
					sec_addr <= sd_sec_write_addr;
				end
				else if(sd_sec_read == 1'b1)
				begin
					state <= S_CMD17;
					sec_addr <= sd_sec_read_addr;
				end

				spi_clk_div <= SPI_HIGH_SPEED_DIV[15:0];
			end
			S_CMD24:
			begin
				// 写命令路径，当前 FAT 链路不会使用，但保留原工程能力。
				if(cmd_req_ack & ~cmd_req_error)
				begin
					state <= S_WRITE;
					cmd_req <= 1'b0;
				end
				else
				begin
					cmd_req <= 1'b1;
					cmd_data_len <= 16'd0;
					cmd_r1 <= 8'h00;
					cmd <= {8'd24,sec_addr,8'hff};

				end
			end
			S_WRITE:
			begin
				// 下级逐字节索取写数据，完整数据块发送完才产生 write_end。
				if(block_write_req_ack == 1'b1)
				begin
					block_write_req <= 1'b0;
					state <= S_WRITE_END;
				end
				else
					block_write_req <= 1'b1;
			end
			S_CMD17:
			begin
				// CMD17 读取一个单块；命令完成后进入数据块等待。
				if(cmd_req_ack & ~cmd_req_error)
				begin
					state <= S_READ;
					cmd_req <= 1'b0;
				end
				else
				begin
					cmd_req <= 1'b1;
					cmd_data_len <= 16'd0;
					cmd_r1 <= 8'h00;
					cmd <= {8'd17,sec_addr,8'hff};
				end
			end
			S_READ:
			begin
				// 保持 block_read_req，直到 sd_card_cmd 返回 block_read_req_ack。
				if(block_read_req_ack)
				begin
					state <= S_READ_END;
					block_read_req <= 1'b0;
				end
				else
				begin
					block_read_req <= 1'b1;
				end
			end
			S_WRITE_END:
			begin
				state <= S_WAIT_READ_WRITE;
			end
			S_READ_END:
			begin
				// end 信号由组合 assign 在本状态整个周期为 1；下一拍回等待态。
				state <= S_WAIT_READ_WRITE;
			end
			default:
				state <= S_IDLE;
		endcase
	end
end
endmodule

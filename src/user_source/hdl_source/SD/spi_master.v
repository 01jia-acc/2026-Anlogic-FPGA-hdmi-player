// SPI 单字节时序引擎。
//
// 上层每次把一个字节放到 data_in，并将 wr_req 拉高；本模块按 clk_div 产生
// 8 个 SPI 时钟周期（共 16 个边沿），在正确的 CPHA 边沿移出 MOSI/采样 MISO，
// 最后用一个时钟宽度的 wr_ack 报告 data_out 已经完成。sd_card_cmd 只需要
// 处理“字节请求/字节完成”，不必再关心 SPI 边沿，因此协议层和物理层解耦。
//
// 这里有两个状态机概念：state 是当前状态寄存器，next_state 是组合逻辑根据
// 当前状态/计数器推导出的下一状态；其余几个 always 块分别维护时钟、计数器、
// MOSI 移位寄存器和 MISO 移位寄存器。它们在同一个时钟沿并行更新。
module spi_master
(
	// 系统时钟与高有效异步复位。
	input                       sys_clk,
	input                       rst,
	// SPI 物理线；nCS 由上层协议状态机控制。
	output                      nCS,       //chip select (SPI mode)
	output                      DCLK,      //spi clock
	output                      MOSI,      //spi data output
	input                       MISO,      //spi input
	// CPOL/CPHA 决定空闲电平和采样/翻转边沿，本工程连接为 mode 3。
	input                       CPOL,
	input                       CPHA,
	input                       nCS_ctrl,
	// 一个 SPI 半周期的系统时钟分频计数值。
	input[15:0]                 clk_div,
	// 单字节请求和完成握手。
	input                       wr_req,
	output                      wr_ack,
	input[7:0]                  data_in,
	// MISO 采样完成后的字节。
	output[7:0]                 data_out
);
// 状态含义：IDLE 等待字节请求；DCLK_IDLE 等待半周期；DCLK_EDGE 翻转 SPI
// 时钟并按 CPHA 移位；LAST_HALF_CYCLE 补完最后半周期；ACK/ACK_WAIT 产生并
// 清除完成握手，避免 wr_req 未及时撤销时重复发送一个字节。
localparam                   IDLE            = 0;
localparam                   DCLK_EDGE       = 1;
localparam                   DCLK_IDLE       = 2;
localparam                   ACK             = 3;
localparam                   LAST_HALF_CYCLE = 4;
localparam                   ACK_WAIT        = 5;
// SPI 时钟寄存器，真正连接到 DCLK 输出。
reg                          DCLK_reg;
// MOSI 移位寄存器，最高位直接驱动引脚，每个发送边沿左移一次。
reg[7:0]                     MOSI_shift;
// MISO 移位寄存器，在采样边沿把输入逐位移入。
reg[7:0]                     MISO_shift;
// 当前/下一状态寄存器。
reg[2:0]                     state;
reg[2:0]                     next_state;
// 半周期等待计数器。
reg [15:0]                   clk_cnt;
// SPI 边沿计数器；0~15 共 16 个边沿，即 8 个完整时钟。
reg[4:0]                     clk_edge_cnt;
// 组合/连续连接：输出引脚和握手状态都由内部寄存器/状态直接解码。
assign MOSI = MOSI_shift[7];
assign DCLK = DCLK_reg;
assign data_out = MISO_shift;
assign wr_ack = (state == ACK);
assign nCS = nCS_ctrl;
// 状态寄存器：只在系统时钟沿采样 next_state。
always@(posedge sys_clk or posedge rst)
begin
	if(rst)
		state <= IDLE;
	else
		state <= next_state;
end

// 下一状态组合逻辑；没有时钟，因此只“计算将来”，不保存信息。
always@(*)
begin
	case(state)
		IDLE:
			if(wr_req == 1'b1)
				next_state <= DCLK_IDLE;
			else
				next_state <= IDLE;
		DCLK_IDLE:
			// 每经过 clk_div 个系统时钟，产生一个 SPI 边沿。
			//half a SPI clock cycle produces a clock edge
			if(clk_cnt == clk_div)
				next_state <= DCLK_EDGE;
			else
				next_state <= DCLK_IDLE;
		DCLK_EDGE:
			// 共 16 个边沿，覆盖 SPI 一个字节的 8 个时钟。
			//a SPI byte with a total of 16 clock edges
			if(clk_edge_cnt == 5'd15)
				next_state <= LAST_HALF_CYCLE;
			else
				next_state <= DCLK_IDLE;
		//this is the last data edge
		LAST_HALF_CYCLE:
			if(clk_cnt == clk_div)
				next_state <= ACK;
			else
				next_state <= LAST_HALF_CYCLE;
		//send one byte complete
		ACK:
			next_state <= ACK_WAIT;
		//wait for one clock cycle, to ensure that the cancel request signal
		ACK_WAIT:
			next_state <= IDLE;
		default:
			next_state <= IDLE;
	endcase
end

	// SPI 时钟电平寄存器；IDLE 输出 CPOL，DCLK_EDGE 翻转一次。
	always@(posedge sys_clk or posedge rst)
begin
	if(rst)
		DCLK_reg <= 1'b0;
	else if(state == IDLE)
		DCLK_reg <= CPOL;
	else if(state == DCLK_EDGE)
		DCLK_reg <= ~DCLK_reg;//SPI clock edge
end
// SPI 半周期等待计数器：只在两个等待状态中递增，其余阶段清零。
always@(posedge sys_clk or posedge rst)
begin
	if(rst)
		clk_cnt <= 16'd0;
	else if(state == DCLK_IDLE || state == LAST_HALF_CYCLE)
		clk_cnt <= clk_cnt + 16'd1;
	else
		clk_cnt <= 16'd0;
end
// SPI 边沿计数器：每次 DCLK_EDGE 加一，回到 IDLE 时准备下一字节。
always@(posedge sys_clk or posedge rst)
begin
	if(rst)
		clk_edge_cnt <= 5'd0;
	else if(state == DCLK_EDGE)
		clk_edge_cnt <= clk_edge_cnt + 5'd1;
	else if(state == IDLE)
		clk_edge_cnt <= 5'd0;
end
// SPI 数据输出：根据 CPHA 在规定边沿移位；MOSI 始终输出当前最高位。
always@(posedge sys_clk or posedge rst)
begin
	if(rst)
		MOSI_shift <= 8'd0;
	else if(state == IDLE && wr_req)
		MOSI_shift <= data_in;
	else if(state == DCLK_EDGE)
		if(CPHA == 1'b0 && clk_edge_cnt[0] == 1'b1)
			MOSI_shift <= {MOSI_shift[6:0],MOSI_shift[7]};
		else if(CPHA == 1'b1 && (clk_edge_cnt != 5'd0 && clk_edge_cnt[0] == 1'b0))
			MOSI_shift <= {MOSI_shift[6:0],MOSI_shift[7]};
end
// SPI 数据输入：在另一组 CPHA 对应边沿采样 MISO，拼成 data_out。
always@(posedge sys_clk or posedge rst)
begin
	if(rst)
		MISO_shift <= 8'd0;
	else if(state == IDLE && wr_req)
		MISO_shift <= 8'h00;
	else if(state == DCLK_EDGE)
		if(CPHA == 1'b0 && clk_edge_cnt[0] == 1'b0)
			MISO_shift <= {MISO_shift[6:0],MISO};
		else if(CPHA == 1'b1 && (clk_edge_cnt[0] == 1'b1))
			MISO_shift <= {MISO_shift[6:0],MISO};
end
endmodule

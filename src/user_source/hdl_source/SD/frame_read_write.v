`timescale 1ns/1ps
// ============================================================================
// frame_read_write：帧缓存读写控制器（写通道 + 读通道）
//
// 本模块是"像素数据流"与"SDRAM 帧缓存"之间的桥梁，内部是两条完全对称的
// 异步 FIFO 流水线：
//
//   写通道（TF/SD 卡图片 → SDRAM）：
//     上游 BMP 解析器 --write_en/write_data--> 写FIFO（跨时钟域缓冲）
//       --> frame_fifo_write 状态机 --App_wr_en/App_wr_addr/App_wr_din-->
//      SDRAM 控制器
//
//   读通道（SDRAM → 显示器）：
//     frame_fifo_read 状态机 --App_rd_en/App_rd_addr--> SDRAM 控制器
//       --Sdr_rd_en/Sdr_rd_dout--> 读FIFO（跨时钟域缓冲）
//       --read_en/read_data--> 显示端
//
// 两条通道的握手协议（写侧为例，读侧完全对称）：
//   * write_req / write_req_ack：帧级握手。上游发请求后，本模块锁存写长度、
//     基地址并清空 FIFO，然后才应答；上游收到应答才开始灌像素。
//   * FIFO 攒够一个突发（BURST_SIZE 个字）才向 SDRAM 发起一次突发写，
//     避免零散的读写频繁打断对方。
//   * write_finish / read_finish：一整帧写/读完成的脉冲。
// 读写互斥：写通道发起突发前要等 ~App_rd_busy（读通道不忙），写指令优先。
// ============================================================================
module frame_read_write
#
(
	parameter MEM_DATA_BITS          = 32,      // SDRAM 用户接口数据位宽
	parameter READ_DATA_BITS         = 32,      // 显示端读数据位宽
	parameter WRITE_DATA_BITS        = 32,      // 写端数据位宽
	parameter ADDR_BITS              = 21,      // SDRAM 用户接口地址位宽
	parameter BURST_BITS             = 9,       // 突发长度计数器的位宽（容纳 BURST_SIZE）
	parameter BURST_SIZE             = 256,     // 一次突发读/写的字数（FIFO 攒够这么多才动手）
	parameter WRITE_V_FLIP           = 1,       // =1：写 SDRAM 时做垂直翻转（BMP 文件自底向上存储，翻转后显示才正立）
	parameter FRAME_WIDTH            = 640,     // 帧宽（像素）
	parameter FRAME_HEIGHT           = 480      // 帧高（行数）
)
(
	input                            rst,                  // 高有效复位
	input                            mem_clk,              // SDRAM 控制器用户接口时钟（两个状态机所在的时钟域）
	input							 Sdr_init_done,        // SDRAM 初始化完成标志
	input							 Sdr_init_ref_vld,     // SDRAM 初始化/刷新有效（本设计未直接使用）
    input							 Sdr_busy,             // SDRAM 控制器忙标志

    // ------------------------------------------------------------------------
    // 以下为旧版"突发读写请求"接口，已被 App_xx 简化接口取代，保留作参考。
    // ------------------------------------------------------------------------
    /*
    output                           rd_burst_req,               // to external memory controller,send out a burst read request
	output[BURST_BITS - 1:0]         rd_burst_len,               // to external memory controller,data length of the burst read request, not bytes
	output[ADDR_BITS - 1:0]          rd_burst_addr,              // to external memory controller,base address of the burst read request
	input                            rd_burst_data_valid,        // from external memory controller,read data valid
	input[MEM_DATA_BITS - 1:0]       rd_burst_data,              // from external memory controller,read request data
	input                            rd_burst_finish,            // from external memory controller,burst read finish
	*/
	// ------------------------------------------------------------------------
	// 读通道与 SDRAM 控制器的接口
	// ------------------------------------------------------------------------
	output							 App_rd_en,               // 读使能：突发读期间保持有效
	output[ADDR_BITS - 1:0]			 App_rd_addr,             // 读地址（由 frame_fifo_read 生成）
	input							 Sdr_rd_en,	              // SDRAM 控制器返回：本拍 Sdr_rd_dout 有效，同时写入读 FIFO
	input[MEM_DATA_BITS - 1:0]		 Sdr_rd_dout,             // SDRAM 控制器返回的读数据

	// ------------------------------------------------------------------------
	// 读通道与显示端的接口
	// ------------------------------------------------------------------------
	input                            read_clk,                 // 显示端读时钟（本设计中为 video_clk）
	input                            read_req,                 // 帧读请求：电平信号，保持到 read_req_ack = 1
	output                           read_req_ack,             // 帧读请求应答："读 FIFO 已准备好，开始读帧"
	output                           read_finish,              // 一整帧读取完成的脉冲
	input[ADDR_BITS - 1:0]           read_addr_0,              // 帧缓存基地址 0（read_addr_index = 0 时选用）
	input[ADDR_BITS - 1:0]           read_addr_1,              // 帧缓存基地址 1（read_addr_index = 1 时选用）
	input[ADDR_BITS - 1:0]           read_addr_2,              // 帧缓存基地址 2（本设计未用，接 0）
	input[ADDR_BITS - 1:0]           read_addr_3,              // 帧缓存基地址 3（本设计未用，接 0）
	input[1:0]                       read_addr_index,          // 选择用哪个基地址（即显示缓冲区 disp_buf_idx）
	input[ADDR_BITS - 1:0]           read_len,                 // 要读的字数（一帧像素数 = 640*480）
	input                            read_en,                  // 显示端逐字取数请求，下一拍 read_data 有效
	output[READ_DATA_BITS  - 1:0]    read_data,                // 显示端读出的数据

    // ------------------------------------------------------------------------
    // 以下为旧版"突发写请求"接口，已被 App_xx 简化接口取代，保留作参考。
    // ------------------------------------------------------------------------
    /*
    output                           wr_burst_req,               // to external memory controller,send out a burst write request
	output[BURST_BITS - 1:0]         wr_burst_len,               // to external memory controller,data length of the burst write request, not bytes
	output[ADDR_BITS - 1:0]          wr_burst_addr,              // to external memory controller,base address of the burst write request
	input                            wr_burst_data_req,          // from external memory controller,write data request ,before data 1 clock
	output[MEM_DATA_BITS - 1:0]      wr_burst_data,              // to external memory controller,write data
	input                            wr_burst_finish,            // from external memory controller,burst write finish
	*/
	// ------------------------------------------------------------------------
	// 写通道与 SDRAM 控制器的接口
	// ------------------------------------------------------------------------
	output							 App_wr_en,               // 写使能：突发写期间保持有效
	output[ADDR_BITS - 1:0]			 App_wr_addr,             // 写地址（由 frame_fifo_write 生成，含垂直翻转）
	output[MEM_DATA_BITS - 1:0]	 App_wr_din,               // 写入 SDRAM 的数据（来自写 FIFO）
	output[3:0]						 App_wr_dm,               // 字节写屏蔽：=0 表示 32 位全部写入

	// ------------------------------------------------------------------------
	// 写通道与上游 BMP 解析器的接口
	// ------------------------------------------------------------------------
	input                            write_clk,                // 写端时钟（本设计中为 sd_card_clk，即像素灌入 FIFO 的节奏）
	input                            write_req,                // 帧写请求：电平信号，保持到 write_req_ack = 1
	output                           write_req_ack,            // 帧写请求应答："写 FIFO 已清空、地址长度已锁存，开始灌像素"
	output                           write_finish,             // 一整帧写完成的脉冲
	input[ADDR_BITS - 1:0]           write_addr_0,             // 帧缓存基地址 0（write_addr_index = 0 时选用）
	input[ADDR_BITS - 1:0]           write_addr_1,             // 帧缓存基地址 1（write_addr_index = 1 时选用）
	input[ADDR_BITS - 1:0]           write_addr_2,             // 帧缓存基地址 2（本设计未用，接 0）
	input[ADDR_BITS - 1:0]           write_addr_3,             // 帧缓存基地址 3（本设计未用，接 0）
	input[1:0]                       write_addr_index,         // 选择写哪个基地址（即写缓冲区 write_buf_idx，双缓冲关键）
	input[ADDR_BITS - 1:0]           write_len,                // 要写的字数（一帧像素数 = 640*480）
	input                            write_en,                 // 逐字写使能：上游每凑满一个像素拍一拍
	input[WRITE_DATA_BITS - 1:0]     write_data                // 32 位像素数据 {R,G,B,8'b0}

);
wire[BURST_BITS - 1:0]                           wrusedw;      // 读 FIFO 中已写入未读走的字数（写时钟域视角）
wire[BURST_BITS - 1:0]                           rdusedw;      // 写 FIFO 中已写入未读走的字数（读时钟域视角）

wire 								 App_rd_busy;              // 读通道忙标志：帧写模块靠它避开与读突发的冲突
wire								 App_wr_busy;              // 写通道忙标志：帧读模块靠它避开与写突发的冲突

wire                                 read_fifo_aclr;           // 读 FIFO 异步清零
wire                                 write_fifo_aclr;          // 写 FIFO 异步清零（每帧开始时排空一次）

wire O_wr_busy;
wire O_rd_busy;

assign App_wr_busy = O_wr_busy; // 写通道忙标志直通（"正在突发写或即将发起"时置 1）
assign App_rd_busy = O_rd_busy; // 读通道忙标志直通；因为写指令优先，写通道不需要关心读指令下个时钟是否到来
assign App_wr_dm = 4'b0000;     // 写字节屏蔽全 0：每个 32 位字的 4 个字节全部写入

// ----------------------------------------------------------------------------
// 写通道 FIFO（异步）：像素流缓冲。
//   写侧：write_clk 域（sd_card_clk），上游 BMP 解析器逐像素写入；
//   读侧：mem_clk 域，frame_fifo_write 攒够一个突发后整批取走写 SDRAM。
// rdusedw（读侧视角的已存字数）是 frame_fifo_write 判断"是否攒够一个突发"
// 的水位依据。深度 512 > 突发长度 256，保证一次突发期间不会被读空。
// ----------------------------------------------------------------------------
wfifo_32_32_512 write_buf
	(
	.clkr                      	(mem_clk                  ),          // 读侧时钟
	.clkw                      	(write_clk                ),          // 写侧时钟
	.rst                       	(write_fifo_aclr          ),          // 异步清零：每帧开始前排空
	.we                      	(write_en                 ),          // 写使能：每像素一拍
	.re                      	(App_wr_en        		  ),          // 读使能：突发写期间连续有效
	.di                       	(write_data               ),          // 写数据：32 位像素
	.empty_flag                 (                         ),          // 读侧空标志（未使用）
	.full_flag                  (                         ),          // 写侧满标志（未使用）
	.wrusedw                	(              	  		  ),          // 写侧视角已存字数（未使用）
	.rdusedw                	(rdusedw                  ),          // 读侧视角已存字数（水位，给 frame_fifo_write 用）
	.dout                       (App_wr_din		          )           // 读数据 → 直接送给 SDRAM 控制器
);

// ----------------------------------------------------------------------------
// 帧写状态机：frame_fifo_write
// 负责：帧级握手（write_req/ack，跨时钟域三级同步）→ 锁存基地址与写长度 →
// 清空 FIFO → 等 FIFO 攒够一个突发且读通道不忙 → 突发写 SDRAM →
// 写够 write_len 个字后输出 write_finish。写地址按 WRITE_V_FLIP 做垂直翻转。
// ----------------------------------------------------------------------------
frame_fifo_write
#
(
	.MEM_DATA_BITS              (MEM_DATA_BITS            ),
	.ADDR_BITS                  (ADDR_BITS                ),
	.BURST_BITS                 (BURST_BITS               ),
	.BURST_SIZE                 (BURST_SIZE               ),
	.WRITE_V_FLIP               (WRITE_V_FLIP             ),
	.FRAME_WIDTH                (FRAME_WIDTH              ),
	.FRAME_HEIGHT               (FRAME_HEIGHT             )
)
frame_fifo_write_m0
(
	.rst                        (rst                      ),
	.mem_clk                    (mem_clk                  ),
    .Sdr_init_done				(Sdr_init_done),
	.Sdr_init_ref_vld			(Sdr_init_ref_vld),
    .Sdr_busy					(Sdr_busy),
	.App_rd_busy				(App_rd_busy),              // 读通道忙？写突发要等它不忙
    .O_wr_busy					(O_wr_busy),                // 本模块忙标志输出

	.App_wr_en					(App_wr_en),                // 突发写使能 → SDRAM 控制器 + 写 FIFO 读使能
    .App_wr_addr				(App_wr_addr),              // 突发写地址 → SDRAM 控制器
	.write_req                  (write_req                ),
	.write_req_ack              (write_req_ack            ),
	.write_finish               (write_finish             ),
	.write_addr_0               (write_addr_0             ),
	.write_addr_1               (write_addr_1             ),
	.write_addr_2               (write_addr_2             ),
	.write_addr_3               (write_addr_3             ),
	.write_addr_index           (write_addr_index         ),
	.write_len                  (write_len                ),
	.fifo_aclr                  (write_fifo_aclr          ), // 帧开始前清空写 FIFO
	.rdusedw                 	(rdusedw                  )  // 写 FIFO 水位
);

// ----------------------------------------------------------------------------
// 读通道 FIFO（异步）：SDRAM 读出的帧数据缓冲。
//   写侧：mem_clk 域，SDRAM 控制器每返回一个字（Sdr_rd_en = 1）就写入；
//   读侧：read_clk 域（video_clk），显示端按视频时序 read_en 逐字取走。
// wrusedw（写侧视角的已存字数）是 frame_fifo_read 判断"何时发起突发读"
// 的水位依据。
// ----------------------------------------------------------------------------
rfifo_32_32_512 read_buf
	(
	.clkr                     	(read_clk                   ),          // 读侧时钟（显示端）
	.clkw                     	(mem_clk                    ),          // 写侧时钟（SDRAM 控制器）
	.rst                      	(read_fifo_aclr             ),          // 异步清零：每帧开始前排空
	.we                     	(Sdr_rd_en       			),          // 写使能：SDRAM 返回一个字就写一个字
	.re                     	(read_en                    ),          // 读使能：显示端逐字取数
	.di                      	(Sdr_rd_dout                ),          // 写数据：SDRAM 返回的读数据
	.empty_flag					(                           ),          // 读侧空标志（未使用）
	.full_flag					(                           ),          // 写侧满标志（未使用）
	.wrusedw                	(wrusedw         	  		 ),          // 写侧视角已存字数（水位，给 frame_fifo_read 用）
	.rdusedw                	(                  			 ),          // 读侧视角已存字数（未使用）
	.dout						(read_data                  )           // 读数据 → 显示端
);

// ----------------------------------------------------------------------------
// 帧读状态机：frame_fifo_read
// 负责：帧级握手（read_req/ack，跨时钟域三级同步）→ 锁存基地址与读长度 →
// 清空 FIFO → 在写通道不忙时突发读 SDRAM 填满读 FIFO → 读够 read_len 个字后
// 输出 read_finish。显示端从读 FIFO 按自己的时钟逐字取数据。
// ----------------------------------------------------------------------------
frame_fifo_read
#
(
	.MEM_DATA_BITS              (MEM_DATA_BITS            ),
	.ADDR_BITS                  (ADDR_BITS                ),
	.BURST_BITS                 (BURST_BITS               ),
	.BURST_SIZE                 (BURST_SIZE               )
)
frame_fifo_read_m0
(
	.rst                        (rst                      ),
	.mem_clk                    (mem_clk                  ),
    .Sdr_init_done				(Sdr_init_done),
	.Sdr_init_ref_vld			(Sdr_init_ref_vld),
    .Sdr_busy					(Sdr_busy),
    .Sdr_rd_en					(Sdr_rd_en),                // SDRAM 返回数据有效 → 同时写入读 FIFO
	.App_wr_busy				(App_wr_busy),              // 写通道忙？读突发要等它不忙
    .O_rd_busy					(O_rd_busy),                // 本模块忙标志输出

    .App_rd_en					(App_rd_en),                // 突发读使能 → SDRAM 控制器
    .App_rd_addr				(App_rd_addr),              // 突发读地址 → SDRAM 控制器
	.read_req                   (read_req                 ),
	.read_req_ack               (read_req_ack             ),
	.read_finish                (read_finish              ),
	.read_addr_0                (read_addr_0              ),
	.read_addr_1                (read_addr_1              ),
	.read_addr_2                (read_addr_2              ),
	.read_addr_3                (read_addr_3              ),
	.read_addr_index            (read_addr_index          ),
	.read_len                   (read_len                 ),
	.fifo_aclr                  (read_fifo_aclr           ), // 帧开始前清空读 FIFO
	.wrusedw                	(wrusedw                  )  // 读 FIFO 水位
);

endmodule

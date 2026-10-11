


`define   DATA_WIDTH                        32
`define   ADDR_WIDTH                        21
`define   DM_WIDTH                          4

`define   ROW_WIDTH                        11
`define   BA_WIDTH                        2

// Macro expansion is textual: always parenthesize arithmetic macro bodies.
// 125 MHz -> 8 ns. Without parentheses the nested division below became
// 64000000 / 1000000000 / 125000000 / 2048 = 0 (integer arithmetic),
// holding init_cnt at zero and preventing the periodic refresh request.
`define   SDR_CLK_PERIOD                    (1000000000 / 125000000)
// Preserve the controller's existing 64 ms / 2^ROW_WIDTH refresh policy.
// ROW_WIDTH=11 -> 3906. The existing counter runs 0..3906 inclusive:
// requests repeat every 3907 clocks = 31.256 us at 125 MHz.
// This is a terminal count, not a declaration that every SDRAM uses 2048 rows.
`define   SELF_REFRESH_INTERVAL             (64000000 / (`SDR_CLK_PERIOD) / (2 ** (`ROW_WIDTH)))


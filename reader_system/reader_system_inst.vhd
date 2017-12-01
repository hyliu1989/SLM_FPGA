	component reader_system is
		port (
			clk_clk                                   : in    std_logic                     := 'X';             -- clk
			jtag_uart_0_avalon_jtag_slave_chipselect  : in    std_logic                     := 'X';             -- chipselect
			jtag_uart_0_avalon_jtag_slave_address     : in    std_logic                     := 'X';             -- address
			jtag_uart_0_avalon_jtag_slave_read_n      : in    std_logic                     := 'X';             -- read_n
			jtag_uart_0_avalon_jtag_slave_readdata    : out   std_logic_vector(31 downto 0);                    -- readdata
			jtag_uart_0_avalon_jtag_slave_write_n     : in    std_logic                     := 'X';             -- write_n
			jtag_uart_0_avalon_jtag_slave_writedata   : in    std_logic_vector(31 downto 0) := (others => 'X'); -- writedata
			jtag_uart_0_avalon_jtag_slave_waitrequest : out   std_logic;                                        -- waitrequest
			reset_reset_n                             : in    std_logic                     := 'X';             -- reset_n
			sdram_controller_0_s1_address             : in    std_logic_vector(24 downto 0) := (others => 'X'); -- address
			sdram_controller_0_s1_byteenable_n        : in    std_logic_vector(1 downto 0)  := (others => 'X'); -- byteenable_n
			sdram_controller_0_s1_chipselect          : in    std_logic                     := 'X';             -- chipselect
			sdram_controller_0_s1_writedata           : in    std_logic_vector(15 downto 0) := (others => 'X'); -- writedata
			sdram_controller_0_s1_read_n              : in    std_logic                     := 'X';             -- read_n
			sdram_controller_0_s1_write_n             : in    std_logic                     := 'X';             -- write_n
			sdram_controller_0_s1_readdata            : out   std_logic_vector(15 downto 0);                    -- readdata
			sdram_controller_0_s1_readdatavalid       : out   std_logic;                                        -- readdatavalid
			sdram_controller_0_s1_waitrequest         : out   std_logic;                                        -- waitrequest
			sdram_controller_0_s1_clock_clk           : out   std_logic;                                        -- clk
			sdram_controller_0_wire_addr              : out   std_logic_vector(12 downto 0);                    -- addr
			sdram_controller_0_wire_ba                : out   std_logic_vector(1 downto 0);                     -- ba
			sdram_controller_0_wire_cas_n             : out   std_logic;                                        -- cas_n
			sdram_controller_0_wire_cke               : out   std_logic;                                        -- cke
			sdram_controller_0_wire_cs_n              : out   std_logic;                                        -- cs_n
			sdram_controller_0_wire_dq                : inout std_logic_vector(15 downto 0) := (others => 'X'); -- dq
			sdram_controller_0_wire_dqm               : out   std_logic_vector(1 downto 0);                     -- dqm
			sdram_controller_0_wire_ras_n             : out   std_logic;                                        -- ras_n
			sdram_controller_0_wire_we_n              : out   std_logic;                                        -- we_n
			sdram_controller_0_wire_clk_clk           : out   std_logic                                         -- clk
		);
	end component reader_system;

	u0 : component reader_system
		port map (
			clk_clk                                   => CONNECTED_TO_clk_clk,                                   --                           clk.clk
			jtag_uart_0_avalon_jtag_slave_chipselect  => CONNECTED_TO_jtag_uart_0_avalon_jtag_slave_chipselect,  -- jtag_uart_0_avalon_jtag_slave.chipselect
			jtag_uart_0_avalon_jtag_slave_address     => CONNECTED_TO_jtag_uart_0_avalon_jtag_slave_address,     --                              .address
			jtag_uart_0_avalon_jtag_slave_read_n      => CONNECTED_TO_jtag_uart_0_avalon_jtag_slave_read_n,      --                              .read_n
			jtag_uart_0_avalon_jtag_slave_readdata    => CONNECTED_TO_jtag_uart_0_avalon_jtag_slave_readdata,    --                              .readdata
			jtag_uart_0_avalon_jtag_slave_write_n     => CONNECTED_TO_jtag_uart_0_avalon_jtag_slave_write_n,     --                              .write_n
			jtag_uart_0_avalon_jtag_slave_writedata   => CONNECTED_TO_jtag_uart_0_avalon_jtag_slave_writedata,   --                              .writedata
			jtag_uart_0_avalon_jtag_slave_waitrequest => CONNECTED_TO_jtag_uart_0_avalon_jtag_slave_waitrequest, --                              .waitrequest
			reset_reset_n                             => CONNECTED_TO_reset_reset_n,                             --                         reset.reset_n
			sdram_controller_0_s1_address             => CONNECTED_TO_sdram_controller_0_s1_address,             --         sdram_controller_0_s1.address
			sdram_controller_0_s1_byteenable_n        => CONNECTED_TO_sdram_controller_0_s1_byteenable_n,        --                              .byteenable_n
			sdram_controller_0_s1_chipselect          => CONNECTED_TO_sdram_controller_0_s1_chipselect,          --                              .chipselect
			sdram_controller_0_s1_writedata           => CONNECTED_TO_sdram_controller_0_s1_writedata,           --                              .writedata
			sdram_controller_0_s1_read_n              => CONNECTED_TO_sdram_controller_0_s1_read_n,              --                              .read_n
			sdram_controller_0_s1_write_n             => CONNECTED_TO_sdram_controller_0_s1_write_n,             --                              .write_n
			sdram_controller_0_s1_readdata            => CONNECTED_TO_sdram_controller_0_s1_readdata,            --                              .readdata
			sdram_controller_0_s1_readdatavalid       => CONNECTED_TO_sdram_controller_0_s1_readdatavalid,       --                              .readdatavalid
			sdram_controller_0_s1_waitrequest         => CONNECTED_TO_sdram_controller_0_s1_waitrequest,         --                              .waitrequest
			sdram_controller_0_s1_clock_clk           => CONNECTED_TO_sdram_controller_0_s1_clock_clk,           --   sdram_controller_0_s1_clock.clk
			sdram_controller_0_wire_addr              => CONNECTED_TO_sdram_controller_0_wire_addr,              --       sdram_controller_0_wire.addr
			sdram_controller_0_wire_ba                => CONNECTED_TO_sdram_controller_0_wire_ba,                --                              .ba
			sdram_controller_0_wire_cas_n             => CONNECTED_TO_sdram_controller_0_wire_cas_n,             --                              .cas_n
			sdram_controller_0_wire_cke               => CONNECTED_TO_sdram_controller_0_wire_cke,               --                              .cke
			sdram_controller_0_wire_cs_n              => CONNECTED_TO_sdram_controller_0_wire_cs_n,              --                              .cs_n
			sdram_controller_0_wire_dq                => CONNECTED_TO_sdram_controller_0_wire_dq,                --                              .dq
			sdram_controller_0_wire_dqm               => CONNECTED_TO_sdram_controller_0_wire_dqm,               --                              .dqm
			sdram_controller_0_wire_ras_n             => CONNECTED_TO_sdram_controller_0_wire_ras_n,             --                              .ras_n
			sdram_controller_0_wire_we_n              => CONNECTED_TO_sdram_controller_0_wire_we_n,              --                              .we_n
			sdram_controller_0_wire_clk_clk           => CONNECTED_TO_sdram_controller_0_wire_clk_clk            --   sdram_controller_0_wire_clk.clk
		);


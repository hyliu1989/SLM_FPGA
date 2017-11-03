	component reader_system is
		port (
			altera_up_sd_card_avalon_interface_0_avalon_sdcard_slave_chipselect  : in    std_logic                     := 'X';             -- chipselect
			altera_up_sd_card_avalon_interface_0_avalon_sdcard_slave_address     : in    std_logic_vector(7 downto 0)  := (others => 'X'); -- address
			altera_up_sd_card_avalon_interface_0_avalon_sdcard_slave_read        : in    std_logic                     := 'X';             -- read
			altera_up_sd_card_avalon_interface_0_avalon_sdcard_slave_write       : in    std_logic                     := 'X';             -- write
			altera_up_sd_card_avalon_interface_0_avalon_sdcard_slave_byteenable  : in    std_logic_vector(3 downto 0)  := (others => 'X'); -- byteenable
			altera_up_sd_card_avalon_interface_0_avalon_sdcard_slave_writedata   : in    std_logic_vector(31 downto 0) := (others => 'X'); -- writedata
			altera_up_sd_card_avalon_interface_0_avalon_sdcard_slave_readdata    : out   std_logic_vector(31 downto 0);                    -- readdata
			altera_up_sd_card_avalon_interface_0_avalon_sdcard_slave_waitrequest : out   std_logic;                                        -- waitrequest
			altera_up_sd_card_avalon_interface_0_conduit_end_b_SD_cmd            : inout std_logic                     := 'X';             -- b_SD_cmd
			altera_up_sd_card_avalon_interface_0_conduit_end_b_SD_dat            : inout std_logic                     := 'X';             -- b_SD_dat
			altera_up_sd_card_avalon_interface_0_conduit_end_b_SD_dat3           : inout std_logic                     := 'X';             -- b_SD_dat3
			altera_up_sd_card_avalon_interface_0_conduit_end_o_SD_clock          : out   std_logic;                                        -- o_SD_clock
			clk_clk                                                              : in    std_logic                     := 'X';             -- clk
			sdram_controller_0_wire_addr                                         : out   std_logic_vector(12 downto 0);                    -- addr
			sdram_controller_0_wire_ba                                           : out   std_logic_vector(1 downto 0);                     -- ba
			sdram_controller_0_wire_cas_n                                        : out   std_logic;                                        -- cas_n
			sdram_controller_0_wire_cke                                          : out   std_logic;                                        -- cke
			sdram_controller_0_wire_cs_n                                         : out   std_logic;                                        -- cs_n
			sdram_controller_0_wire_dq                                           : inout std_logic_vector(15 downto 0) := (others => 'X'); -- dq
			sdram_controller_0_wire_dqm                                          : out   std_logic_vector(1 downto 0);                     -- dqm
			sdram_controller_0_wire_ras_n                                        : out   std_logic;                                        -- ras_n
			sdram_controller_0_wire_we_n                                         : out   std_logic;                                        -- we_n
			reset_reset_n                                                        : in    std_logic                     := 'X';             -- reset_n
			sdram_clock_0_clk                                                    : out   std_logic;                                        -- clk
			sdram_controller_0_s1_address                                        : in    std_logic_vector(24 downto 0) := (others => 'X'); -- address
			sdram_controller_0_s1_byteenable_n                                   : in    std_logic_vector(1 downto 0)  := (others => 'X'); -- byteenable_n
			sdram_controller_0_s1_chipselect                                     : in    std_logic                     := 'X';             -- chipselect
			sdram_controller_0_s1_writedata                                      : in    std_logic_vector(15 downto 0) := (others => 'X'); -- writedata
			sdram_controller_0_s1_read_n                                         : in    std_logic                     := 'X';             -- read_n
			sdram_controller_0_s1_write_n                                        : in    std_logic                     := 'X';             -- write_n
			sdram_controller_0_s1_readdata                                       : out   std_logic_vector(15 downto 0);                    -- readdata
			sdram_controller_0_s1_readdatavalid                                  : out   std_logic;                                        -- readdatavalid
			sdram_controller_0_s1_waitrequest                                    : out   std_logic                                         -- waitrequest
		);
	end component reader_system;

	u0 : component reader_system
		port map (
			altera_up_sd_card_avalon_interface_0_avalon_sdcard_slave_chipselect  => CONNECTED_TO_altera_up_sd_card_avalon_interface_0_avalon_sdcard_slave_chipselect,  -- altera_up_sd_card_avalon_interface_0_avalon_sdcard_slave.chipselect
			altera_up_sd_card_avalon_interface_0_avalon_sdcard_slave_address     => CONNECTED_TO_altera_up_sd_card_avalon_interface_0_avalon_sdcard_slave_address,     --                                                         .address
			altera_up_sd_card_avalon_interface_0_avalon_sdcard_slave_read        => CONNECTED_TO_altera_up_sd_card_avalon_interface_0_avalon_sdcard_slave_read,        --                                                         .read
			altera_up_sd_card_avalon_interface_0_avalon_sdcard_slave_write       => CONNECTED_TO_altera_up_sd_card_avalon_interface_0_avalon_sdcard_slave_write,       --                                                         .write
			altera_up_sd_card_avalon_interface_0_avalon_sdcard_slave_byteenable  => CONNECTED_TO_altera_up_sd_card_avalon_interface_0_avalon_sdcard_slave_byteenable,  --                                                         .byteenable
			altera_up_sd_card_avalon_interface_0_avalon_sdcard_slave_writedata   => CONNECTED_TO_altera_up_sd_card_avalon_interface_0_avalon_sdcard_slave_writedata,   --                                                         .writedata
			altera_up_sd_card_avalon_interface_0_avalon_sdcard_slave_readdata    => CONNECTED_TO_altera_up_sd_card_avalon_interface_0_avalon_sdcard_slave_readdata,    --                                                         .readdata
			altera_up_sd_card_avalon_interface_0_avalon_sdcard_slave_waitrequest => CONNECTED_TO_altera_up_sd_card_avalon_interface_0_avalon_sdcard_slave_waitrequest, --                                                         .waitrequest
			altera_up_sd_card_avalon_interface_0_conduit_end_b_SD_cmd            => CONNECTED_TO_altera_up_sd_card_avalon_interface_0_conduit_end_b_SD_cmd,            --         altera_up_sd_card_avalon_interface_0_conduit_end.b_SD_cmd
			altera_up_sd_card_avalon_interface_0_conduit_end_b_SD_dat            => CONNECTED_TO_altera_up_sd_card_avalon_interface_0_conduit_end_b_SD_dat,            --                                                         .b_SD_dat
			altera_up_sd_card_avalon_interface_0_conduit_end_b_SD_dat3           => CONNECTED_TO_altera_up_sd_card_avalon_interface_0_conduit_end_b_SD_dat3,           --                                                         .b_SD_dat3
			altera_up_sd_card_avalon_interface_0_conduit_end_o_SD_clock          => CONNECTED_TO_altera_up_sd_card_avalon_interface_0_conduit_end_o_SD_clock,          --                                                         .o_SD_clock
			clk_clk                                                              => CONNECTED_TO_clk_clk,                                                              --                                                      clk.clk
			sdram_controller_0_wire_addr                                         => CONNECTED_TO_sdram_controller_0_wire_addr,                                         --                                  sdram_controller_0_wire.addr
			sdram_controller_0_wire_ba                                           => CONNECTED_TO_sdram_controller_0_wire_ba,                                           --                                                         .ba
			sdram_controller_0_wire_cas_n                                        => CONNECTED_TO_sdram_controller_0_wire_cas_n,                                        --                                                         .cas_n
			sdram_controller_0_wire_cke                                          => CONNECTED_TO_sdram_controller_0_wire_cke,                                          --                                                         .cke
			sdram_controller_0_wire_cs_n                                         => CONNECTED_TO_sdram_controller_0_wire_cs_n,                                         --                                                         .cs_n
			sdram_controller_0_wire_dq                                           => CONNECTED_TO_sdram_controller_0_wire_dq,                                           --                                                         .dq
			sdram_controller_0_wire_dqm                                          => CONNECTED_TO_sdram_controller_0_wire_dqm,                                          --                                                         .dqm
			sdram_controller_0_wire_ras_n                                        => CONNECTED_TO_sdram_controller_0_wire_ras_n,                                        --                                                         .ras_n
			sdram_controller_0_wire_we_n                                         => CONNECTED_TO_sdram_controller_0_wire_we_n,                                         --                                                         .we_n
			reset_reset_n                                                        => CONNECTED_TO_reset_reset_n,                                                        --                                                    reset.reset_n
			sdram_clock_0_clk                                                    => CONNECTED_TO_sdram_clock_0_clk,                                                    --                                            sdram_clock_0.clk
			sdram_controller_0_s1_address                                        => CONNECTED_TO_sdram_controller_0_s1_address,                                        --                                    sdram_controller_0_s1.address
			sdram_controller_0_s1_byteenable_n                                   => CONNECTED_TO_sdram_controller_0_s1_byteenable_n,                                   --                                                         .byteenable_n
			sdram_controller_0_s1_chipselect                                     => CONNECTED_TO_sdram_controller_0_s1_chipselect,                                     --                                                         .chipselect
			sdram_controller_0_s1_writedata                                      => CONNECTED_TO_sdram_controller_0_s1_writedata,                                      --                                                         .writedata
			sdram_controller_0_s1_read_n                                         => CONNECTED_TO_sdram_controller_0_s1_read_n,                                         --                                                         .read_n
			sdram_controller_0_s1_write_n                                        => CONNECTED_TO_sdram_controller_0_s1_write_n,                                        --                                                         .write_n
			sdram_controller_0_s1_readdata                                       => CONNECTED_TO_sdram_controller_0_s1_readdata,                                       --                                                         .readdata
			sdram_controller_0_s1_readdatavalid                                  => CONNECTED_TO_sdram_controller_0_s1_readdatavalid,                                  --                                                         .readdatavalid
			sdram_controller_0_s1_waitrequest                                    => CONNECTED_TO_sdram_controller_0_s1_waitrequest                                     --                                                         .waitrequest
		);


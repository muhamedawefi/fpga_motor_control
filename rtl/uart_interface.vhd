library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
 
-- ============================================================
-- UART Motor Control Interface
-- ============================================================
-- Frame format (host → FPGA):
--   [0xAA][CMD][ADDR][DATA_H][DATA_L][XOR_CHK]  6 bytes
--
-- Frame format (FPGA → host, READ response):
--   [0x55][ADDR][DATA_H][DATA_L][XOR_CHK]        5 bytes
--
-- Frame format (FPGA → host, telemetry stream):
--   [0xFF][SPD_H][SPD_L][CUR_H][CUR_L][VCMD_H][VCMD_L][XOR] 8 bytes
--
-- Commands:
--   0x01  WRITE  write DATA to ADDR
--   0x02  READ   read ADDR, FPGA sends response
--   0x03  STREAM enable auto telemetry, DATA=period in ms
--   0x04  STOP   disable auto telemetry
--
-- Register map:
--   0x00  speed_setpoint  R/W  Q9.7
--   0x01  speed_fb        R    Q9.7
--   0x02  current_fb      R    Q4.12
--   0x03  v_cmd           R    Q6.10
--   0x04  stream_period   R/W  ms between telemetry frames
--
-- Checksum: XOR of all bytes in frame except checksum itself
-- Baud: 115200
-- ============================================================
 
entity uart_controller is
    Generic (
        CLK_FREQ  : integer := 100_000_000;
        BAUD_RATE : integer := 115200
    );
    Port (
        clk   : in  std_logic;
        reset : in  std_logic;
 
        -- Physical UART pins
        rx : in  std_logic;
        tx : out std_logic;
 
        -- Register write output to top
        reg_addr : out std_logic_vector(7 downto 0);
        reg_data : out std_logic_vector(15 downto 0);
        reg_wr   : out std_logic;
 
        -- Live signals from top (for read and telemetry)
        speed_fb_in   : in signed(15 downto 0);
        current_fb_in : in signed(15 downto 0);
        v_cmd_in      : in signed(15 downto 0);
        speed_sp_in   : in signed(15 downto 0)
    );
end uart_controller;
 
architecture Behavioral of uart_controller is
 
    constant BAUD_DIV : integer := CLK_FREQ / BAUD_RATE;
 
    ---------------------------------------------------------------
    -- RX signals
    ---------------------------------------------------------------
    type rx_state_t is (RX_IDLE, RX_START, RX_DATA, RX_STOP);
    signal rx_state    : rx_state_t := RX_IDLE;
    signal rx_baud_cnt : integer range 0 to BAUD_DIV := 0;
    signal rx_bit_cnt  : integer range 0 to 7 := 0;
    signal rx_shift    : std_logic_vector(7 downto 0) := (others => '0');
    signal rx_byte     : std_logic_vector(7 downto 0) := (others => '0');
    signal rx_valid    : std_logic := '0';
    signal rx_s1       : std_logic := '1';
    signal rx_s2       : std_logic := '1';
   signal read_cmd      : std_logic := '0'; -- Pulse from parser
   signal read_pending  : std_logic := '0'; -- Latch in sequencer
   signal saved_addr    : std_logic_vector(7 downto 0) := (others => '0'); 
    ---------------------------------------------------------------
    -- TX FIFO (32 bytes deep - fits READ=5 + TELEM=8 with headroom)
    ---------------------------------------------------------------
    type fifo_t is array (0 to 31) of std_logic_vector(7 downto 0);
    signal fifo    : fifo_t := (others => (others => '0'));
    signal wr_ptr  : integer range 0 to 31 := 0;
    signal rd_ptr  : integer range 0 to 31 := 0;
 
    -- fifo_empty is a concurrent signal assignment - NOT inside process
    -- This avoids the VHDL-1993 "construct not supported" erro
    signal fifo_empty : std_logic := '1';
 
    -- Single driver for FIFO write - only tx_sequencer writes
    signal fifo_wr    : std_logic := '0';
    signal fifo_wdata : std_logic_vector(7 downto 0) := (others => '0');
 
    ---------------------------------------------------------------
    -- TX UART signals
    ---------------------------------------------------------------
    type tx_state_t is (TX_IDLE, TX_START, TX_DATA, TX_STOP);
    signal tx_state    : tx_state_t := TX_IDLE;
    signal tx_baud_cnt : integer range 0 to BAUD_DIV := 0;
    signal tx_bit_cnt  : integer range 0 to 7 := 0;
    signal tx_shift    : std_logic_vector(7 downto 0) := (others => '1');
    signal tx_reg      : std_logic := '1';
signal do_read : std_logic := '0'; 
    ---------------------------------------------------------------
    -- Frame parser signals
    ---------------------------------------------------------------
    type frame_state_t is (
        WAIT_START, GET_CMD, GET_ADDR,
        GET_DATA_H, GET_DATA_L, GET_CHK, EXEC
    );
    signal frame_state : frame_state_t := WAIT_START;
 
    signal f_cmd      : std_logic_vector(7 downto 0) := (others => '0');
    signal f_addr     : std_logic_vector(7 downto 0) := (others => '0');
    signal f_data_h   : std_logic_vector(7 downto 0) := (others => '0');
    signal f_data_l   : std_logic_vector(7 downto 0) := (others => '0');
    signal f_chk_rx   : std_logic_vector(7 downto 0) := (others => '0');
    signal f_chk_calc : std_logic_vector(7 downto 0) := (others => '0');
 
    ---------------------------------------------------------------
    -- Register file (internal copies for readback)
    ---------------------------------------------------------------
    signal speed_sp_reg      : std_logic_vector(15 downto 0) := (others => '0');
    signal stream_period_reg : std_logic_vector(15 downto 0) := x"000A"; -- 10ms
 signal latched_period : std_logic_vector(15 downto 0) ;
    ---------------------------------------------------------------
    -- Register write outputs
    ---------------------------------------------------------------
    signal reg_addr_r : std_logic_vector(7 downto 0)  := (others => '0');
    signal reg_data_r : std_logic_vector(15 downto 0) := (others => '0');
    signal reg_wr_r   : std_logic := '0';
 
    ---------------------------------------------------------------
    -- Telemetry stream
    ---------------------------------------------------------------
    signal stream_en   : std_logic := '0';
    signal stream_trig : std_logic := '0';
 
    ---------------------------------------------------------------
    -- TX sequencer
    ---------------------------------------------------------------
    type seq_state_t is (
        SEQ_IDLE,
        SEQ_R0, SEQ_R1, SEQ_R2, SEQ_R3, SEQ_R4,
        SEQ_T0, SEQ_T1, SEQ_T2, SEQ_T3,
        SEQ_T4, SEQ_T5, SEQ_T6, SEQ_T7
    );
    signal seq_state : seq_state_t := SEQ_IDLE;
 
    -- Latched read response
    signal resp_addr : std_logic_vector(7 downto 0) := (others => '0');
    signal resp_dh   : std_logic_vector(7 downto 0) := (others => '0');
    signal resp_dl   : std_logic_vector(7 downto 0) := (others => '0');
    signal resp_chk  : std_logic_vector(7 downto 0) := (others => '0');
 
    -- Latched telemetry snapshot
    signal telem_spd_h  : std_logic_vector(7 downto 0) := (others => '0');
    signal telem_spd_l  : std_logic_vector(7 downto 0) := (others => '0');
    signal telem_cur_h  : std_logic_vector(7 downto 0) := (others => '0');
    signal telem_cur_l  : std_logic_vector(7 downto 0) := (others => '0');
    signal telem_vcmd_h : std_logic_vector(7 downto 0) := (others => '0');
    signal telem_vcmd_l : std_logic_vector(7 downto 0) := (others => '0');
    signal telem_chk    : std_logic_vector(7 downto 0) := (others => '0');
 
 -- protection from fifo overflow
 signal fifo_full : std_logic;
 
signal read_req : std_logic := '0';   -- set by frame_parser, cleared by frame_parser on ack
signal read_ack : std_logic := '0';   -- set by tx_sequencer when it latches 
    -- FIX: registered pending flag instead of 1-cycle pulse
    -- do_read was a 1-cycle pulse that tx_sequencer could miss if busy
    -- read_pending holds until sequencer acknowledges it
 
	    ---------------------------------------------------------------
    -- Helper functions
    ---------------------------------------------------------------
    function hi(s : signed(15 downto 0)) return std_logic_vector is
    begin
        return std_logic_vector(s(15 downto 8));
    end function;
 
    function lo(s : signed(15 downto 0)) return std_logic_vector is
    begin
        return std_logic_vector(s(7 downto 0));
    end function;
 
    function read_reg(
        addr     : std_logic_vector(7 downto 0);
        sp_reg   : std_logic_vector(15 downto 0);
        spd_fb   : signed(15 downto 0);
        cur_fb   : signed(15 downto 0);
        vcmd     : signed(15 downto 0);
        strm_per : std_logic_vector(15 downto 0)
    ) return std_logic_vector is
    begin
        case addr is
            when x"00"  => return sp_reg;
            when x"01"  => return std_logic_vector(spd_fb);
            when x"02"  => return std_logic_vector(cur_fb);
            when x"03"  => return std_logic_vector(vcmd);
            when x"04"  => return strm_per;
            when others => return (15 downto 0 => '0');
        end case;
    end function;
 
begin
 
    -- Output assignments
    reg_addr <= reg_addr_r;
    reg_data <= reg_data_r;
    reg_wr   <= reg_wr_r;
    tx       <= tx_reg;
 
    -- FIX: fifo_empty as concurrent assignment OUTSIDE any process
    -- Inside a clocked process, 'when/else' is not supported in VHDL-1993
    -- This concurrent assignment updates combinationally - correct behavior
    fifo_empty <= '1' when wr_ptr = rd_ptr else '0';
       fifo_full <= '1' when
    (wr_ptr = 31 and rd_ptr = 0) or
    (wr_ptr /= 31 and (wr_ptr + 1 = rd_ptr))
       else '0';
       -----------------------------------------------
    -- RX metastability synchronizer
    ---------------------------------------------------------------
    rx_sync: process(clk)
    begin
        if rising_edge(clk) then
            rx_s1 <= rx;
            rx_s2 <= rx_s1;
        end if;
    end process;
 
    ---------------------------------------------------------------
    -- UART RX state machine
    ---------------------------------------------------------------
   ---------------------------------------------------------------
   -- UART RX state machine (FIXED: samples at bit center)
   ---------------------------------------------------------------
   uart_rx_proc: process(clk, reset)
   begin
       if reset = '1' then
           rx_state    <= RX_IDLE;
           rx_baud_cnt <= 0;
           rx_bit_cnt  <= 0;
           rx_shift    <= (others => '0');
           rx_byte     <= (others => '0');
           rx_valid    <= '0';

       elsif rising_edge(clk) then
           rx_valid <= '0';

           case rx_state is
               when RX_IDLE =>
                   if rx_s2 = '0' then
                       rx_baud_cnt <= (BAUD_DIV / 2) - 2;
                       rx_state    <= RX_START;
                   end if;

               when RX_START =>
                   if rx_baud_cnt = 0 then
                       if rx_s2 = '0' then
                           rx_baud_cnt <= BAUD_DIV;
                           rx_bit_cnt  <= 0;
                           rx_state    <= RX_DATA;
                       else
                           rx_state <= RX_IDLE;
                       end if;
                   else
                       rx_baud_cnt <= rx_baud_cnt - 1;
                   end if;

               when RX_DATA =>
                   -- CRITICAL FIX: Sample at center of each bit period
                   if rx_baud_cnt = BAUD_DIV/2 then
                       rx_shift <= rx_s2 & rx_shift(7 downto 1);
                   end if;
                   if rx_baud_cnt = 0 then
                       rx_baud_cnt <= BAUD_DIV;
                       if rx_bit_cnt = 7 then
                           rx_state <= RX_STOP;
                       else
                           rx_bit_cnt <= rx_bit_cnt + 1;
                       end if;
                   else
                       rx_baud_cnt <= rx_baud_cnt - 1;
                   end if;

               when RX_STOP =>
                   if rx_baud_cnt = 0 then
                       if rx_s2 = '1' then
                           rx_byte  <= rx_shift;
                           rx_valid <= '1';
                       end if;
                       rx_state <= RX_IDLE;
                   else
                       rx_baud_cnt <= rx_baud_cnt - 1;
                   end if;
           end case;
       end if;
   end process;
   ---------------------------------------------------------------
   -- Frame Parser
   ---------------------------------------------------------------
   frame_parser: process(clk, reset)
   begin
       if reset = '1' then
           frame_state       <= WAIT_START;
           -- ... (keep other resets) ...
           read_cmd          <= '0';

       elsif rising_edge(clk) then
           reg_wr_r <= '0';
           read_cmd <= '0'; -- Default low

           if rx_valid = '1' then
               report "RX: byte=" & integer'image(to_integer(unsigned(rx_byte))) severity note;
           end if;

           case frame_state is
               when WAIT_START =>
                   if rx_valid = '1' and rx_byte = x"AA" then
                       f_chk_calc  <= x"AA";
                       frame_state <= GET_CMD;
                   end if;

               when GET_CMD =>
                   if rx_valid = '1' then
                       f_cmd       <= rx_byte;
                       f_chk_calc  <= f_chk_calc xor rx_byte;
                       frame_state <= GET_ADDR;
                   end if;

               when GET_ADDR =>
                   if rx_valid = '1' then
                       f_addr      <= rx_byte;
                       f_chk_calc  <= f_chk_calc xor rx_byte;
                       frame_state <= GET_DATA_H;
                   end if;

               when GET_DATA_H =>
                   if rx_valid = '1' then
                       f_data_h    <= rx_byte;
                       f_chk_calc  <= f_chk_calc xor rx_byte;
                       frame_state <= GET_DATA_L;
                   end if;

               when GET_DATA_L =>
                   if rx_valid = '1' then
                       f_data_l    <= rx_byte;
                       f_chk_calc  <= f_chk_calc xor rx_byte;
                       frame_state <= GET_CHK;
                   end if;

               when GET_CHK =>
                   if rx_valid = '1' then
                       f_chk_rx    <= rx_byte;
                       frame_state <= EXEC;
                   end if;

               when EXEC =>
                   frame_state <= WAIT_START;
                   if f_chk_rx = f_chk_calc then
                       if f_cmd = x"01" then
                           reg_addr_r <= f_addr;
                           reg_data_r <= f_data_h & f_data_l;
                           reg_wr_r   <= '1';
                           if f_addr = x"00" then
                               speed_sp_reg <= f_data_h & f_data_l;
                           elsif f_addr = x"04" then
                               stream_period_reg <= f_data_h & f_data_l;
                           end if;
                       elsif f_cmd = x"02" then
                            read_req <= '1'; 
                      elsif f_cmd = x"03" then
                           stream_en         <= '1';
                           stream_period_reg <= f_data_h & f_data_l;
                           report "STREAM_CMD: Enabled" severity warning;
                       elsif f_cmd = x"04" then
                           stream_en <= '0';
                       end if;
                   end if;
           end case;
       end if;
   end process;    ---------------------------------------------------------------
    -- Telemetry Stream Timer
    -- Fixed: period_v variable used locally, no dead signal
    ---------------------------------------------------------------
    stream_timer: process(clk, reset)
        variable cnt      : integer := 0;
        variable period_v : integer := 0;
    begin
        if reset = '1' then
            cnt         := 0;
            stream_trig <= '0';
 
        elsif rising_edge(clk) then
            stream_trig <= '0';
 
            -- Safe period computation - caps at 20000ms to prevent overflow
            -- 20000 * 100000 = 2,000,000,000 < 2,147,483,647 (integer max)
            if to_integer(unsigned(stream_period_reg)) > 20000 then
                period_v := 2_000_000_000;
            else
                period_v := to_integer(unsigned(stream_period_reg))
                            * (CLK_FREQ / 1000);
            end if;
 
            if stream_en = '1' then
                cnt := cnt + 1;
                if cnt >= period_v then
                    cnt         := 0;
                    stream_trig <= '1'; 
                    report "TIMER_TRIG: Stream timer elapsed" severity warning;
                end if;
            else
                cnt := 0;
            end if;
        end if;
    end process;
 
    ---------------------------------------------------------------
    -- TX Sequencer
    -- FIX: checks read_pending flag (persistent) not do_read pulse
    -- FIX: clears read_pending before processing to prevent re-trigger
    ---------------------------------------------------------------

    tx_sequencer: process(clk, reset)
  begin
      if reset = '1' then
          seq_state    <= SEQ_IDLE;
          fifo_wr      <= '0';
          fifo_wdata   <= (others => '0');
          read_pending <= '0';
          -- ... (keep other resets) ...

      elsif rising_edge(clk) then
          fifo_wr <= '0';

          case seq_state is
              when SEQ_IDLE =>
                   if read_req = '1' and read_ack = '0' then
    read_pending <= '1';
    saved_addr   <= f_addr;
    read_ack     <= '1';
end if;
if read_req = '0' then
    read_ack <= '0';   -- de-assert ack once parser cleared the request
end if;
                  -- Priority 1: Process pending READ
                  if read_pending = '1' and fifo_full = '0' then
                      report "TX: READ sequence started" severity warning;
                      resp_addr <= saved_addr;
                      resp_dh   <= hi(signed(speed_sp_reg));
                      resp_dl   <= lo(signed(speed_sp_reg));
                      resp_chk  <= x"55" xor saved_addr xor resp_dh xor resp_dl;
                      seq_state <= SEQ_R0;
                      read_pending <= '0'; -- Clear latch
                      
                  -- Priority 2: Telemetry stream
                  elsif stream_trig = '1' and fifo_full = '0' and stream_en = '1' then
                      report "TX: TELEMETRY sequence started" severity warning;
                      telem_spd_h  <= hi(speed_fb_in);
                      telem_spd_l  <= lo(speed_fb_in);
                      telem_cur_h  <= hi(current_fb_in);
                      telem_cur_l  <= lo(current_fb_in);
                      telem_vcmd_h <= hi(v_cmd_in);
                      telem_vcmd_l <= lo(v_cmd_in);
                      telem_chk <= x"FF"
                                   xor hi(speed_fb_in) xor lo(speed_fb_in)
                                   xor hi(current_fb_in) xor lo(current_fb_in)
                                   xor hi(v_cmd_in) xor lo(v_cmd_in);
                      seq_state <= SEQ_T0;
                  end if;

              -- ... (Keep your READ/TELEMETRY state cases exactly as they were) ...
              when SEQ_R0 => if fifo_full = '0' then fifo_wr <= '1'; fifo_wdata <= x"55"; seq_state <= SEQ_R1; end if;
              when SEQ_R1 => if fifo_full = '0' then fifo_wr <= '1'; fifo_wdata <= resp_addr; seq_state <= SEQ_R2; end if;
              when SEQ_R2 => if fifo_full = '0' then fifo_wr <= '1'; fifo_wdata <= resp_dh; seq_state <= SEQ_R3; end if;
              when SEQ_R3 => if fifo_full = '0' then fifo_wr <= '1'; fifo_wdata <= resp_dl; seq_state <= SEQ_R4; end if;
   when SEQ_R4 =>
       if fifo_full = '0' then
           fifo_wr <= '1';
           -- 🔑 Compute checksum directly from register (bypasses 1-cycle signal delay)
           fifo_wdata <= x"55" xor saved_addr xor hi(signed(speed_sp_reg)) xor lo(signed(speed_sp_reg));
           seq_state <= SEQ_IDLE;
           read_pending <= '0';
       end if;
              when SEQ_T0 => if fifo_full = '0' then fifo_wr <= '1'; fifo_wdata <= x"FF"; seq_state <= SEQ_T1; end if;
              when SEQ_T1 => if fifo_full = '0' then fifo_wr <= '1'; fifo_wdata <= telem_spd_h; seq_state <= SEQ_T2; end if;
              when SEQ_T2 => if fifo_full = '0' then fifo_wr <= '1'; fifo_wdata <= telem_spd_l; seq_state <= SEQ_T3; end if;
              when SEQ_T3 => if fifo_full = '0' then fifo_wr <= '1'; fifo_wdata <= telem_cur_h; seq_state <= SEQ_T4; end if;
              when SEQ_T4 => if fifo_full = '0' then fifo_wr <= '1'; fifo_wdata <= telem_cur_l; seq_state <= SEQ_T5; end if;
              when SEQ_T5 => if fifo_full = '0' then fifo_wr <= '1'; fifo_wdata <= telem_vcmd_h; seq_state <= SEQ_T6; end if;
              when SEQ_T6 => if fifo_full = '0' then fifo_wr <= '1'; fifo_wdata <= telem_vcmd_l; seq_state <= SEQ_T7; end if;
              when SEQ_T7 => if fifo_full = '0' then fifo_wr <= '1'; fifo_wdata <= telem_chk; seq_state <= SEQ_IDLE; end if;

          end case;
      end if;
  end process;
  -- TX FIFO write process
    -- Single writer: tx_sequencer
    ---------------------------------------------------------------
    tx_fifo_proc: process(clk, reset)
    begin
        if reset = '1' then
            wr_ptr <= 0;
            fifo   <= (others => (others => '0'));
        elsif rising_edge(clk) then
            if fifo_wr = '1' and fifo_full = '0' then 
                fifo(wr_ptr) <= fifo_wdata;
                --handleds the ring buffer better than the mod 
              if wr_ptr = 31 then
              wr_ptr <= 0;
              else
              wr_ptr <= wr_ptr + 1;
              end if;
            end if;
        end if;
    end process;
 
    ---------------------------------------------------------------
    -- UART TX state machine
    -- Single reader of FIFO
    ---------------------------------------------------------------
    uart_tx_proc: process(clk, reset)
    begin
        if reset = '1' then
            tx_state    <= TX_IDLE;
            tx_baud_cnt <= 0;
            tx_bit_cnt  <= 0;
            tx_shift    <= (others => '1');
            tx_reg      <= '1';
            rd_ptr      <= 0;
 
        elsif rising_edge(clk) then
            case tx_state is
 
                when TX_IDLE =>
                    tx_reg <= '1';
                    if fifo_empty = '0' then
                        tx_shift    <= fifo(rd_ptr);
                        if rd_ptr = 31 then
                        rd_ptr <= 0;
                        else
                       rd_ptr <= rd_ptr + 1;
                        end if;
                        tx_reg      <= '0';        -- start bit
                        tx_baud_cnt <= BAUD_DIV;
                        tx_state    <= TX_START;
                    end if;
 
                when TX_START =>
                    if tx_baud_cnt = 0 then
                        tx_reg      <= tx_shift(0);
                        tx_shift    <= '1' & tx_shift(7 downto 1);
                        tx_bit_cnt  <= 0;
                        tx_baud_cnt <= BAUD_DIV;
                        tx_state    <= TX_DATA;
                    else
                        tx_baud_cnt <= tx_baud_cnt - 1;
                    end if;
 
                when TX_DATA =>
                    if tx_baud_cnt = 0 then
                        if tx_bit_cnt = 7 then
                            tx_reg      <= '1';    -- stop bit
                            tx_baud_cnt <= BAUD_DIV;
                            tx_state    <= TX_STOP;
                        else
                            tx_reg      <= tx_shift(0);
                            tx_shift    <= '1' & tx_shift(7 downto 1);
                            tx_bit_cnt  <= tx_bit_cnt + 1;
                            tx_baud_cnt <= BAUD_DIV;
                        end if;
                    else
                        tx_baud_cnt <= tx_baud_cnt - 1;
                    end if;
 
                when TX_STOP =>
                    if tx_baud_cnt = 0 then
                        tx_state <= TX_IDLE;
                    else
                        tx_baud_cnt <= tx_baud_cnt - 1;
                    end if;
 
            end case;
        end if;
    end process;
-- DEBUG: Log exactly what VHDL sees on the pin every clock cycle
debug_rx_monitor: process(clk)
begin
    if rising_edge(clk) then
        if rx_s2 = '0' then
            report "PIN_LOW" severity note;
        end if;
    end if;
end process; 
end Behavioral;
 



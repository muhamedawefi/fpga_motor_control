library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
 use work.motor_constants_pkg.all;

entity top is
Generic (
        -- ⚠️ CHANGE to false for hardware
        SIM_MODE : boolean := true
    );

    Port (
        clk         : in  std_logic;
        reset       : in  std_logic;
        
           -- Encoder (used in hardware mode, tied off in simulation)
        enc_a       : in  std_logic;
        enc_b       : in  std_logic;
       speed_setpoint_debug : out signed(15 downto 0);  -- DEBUG ONLY 
         -- PWM output to motor driver
        pwm_out     : out std_logic;
        
         -- Debug: voltage command visible to testbench
        v_cmd_test  : out signed(15 downto 0);
        
        -- SIMULATION-ONLY ports
        -- ⚠️ These ports are only meaningful when SIM_MODE=true
        -- In hardware, tie speed_fb='0' and sim_current_ma='0'
        -- They will be ignored when SIM_MODE=false
        
        speed_fb    : in  signed(15 downto 0);
        sim_current_ma   : in  signed(15 downto 0);

  -- SPI ADC pins (used in hardware mode only)
        -- ⚠️ Add pin constraints for these in hardware
        -- ============================================================
        spi_cs   : out std_logic;
        spi_clk  : out std_logic;
        spi_mosi : out std_logic;
        spi_miso : in  std_logic;
        
        
        -- ⚠️ SIMULATION ONLY - parallel register interface
        -- Direct register access, no serial overhead
        -- In hardware: tie all to '0', ignored when SIM_MODE=false
        uart_addr     : in  std_logic_vector(7 downto 0);
        uart_wr       : in  std_logic;
        uart_rd       : in  std_logic;
        uart_data_in  : in  std_logic_vector(31 downto 0);
        uart_data_out : out std_logic_vector(31 downto 0);

        -- ⚠️ HARDWARE ONLY - serial UART pins
        -- In simulation: tie rx='1', ignore tx
        uart_rx : in  std_logic;
        uart_tx : out std_logic ;
        w_ref  : out signed(31 downto 0);
        w_meas : out signed(31 downto 0);
        i_meas : out signed(31 downto 0)
    );
end top;

architecture Behavioral of top is
signal current_ref_test : signed(15 downto 0) := to_signed(1000, 16);
signal speed_setpoint_sim : signed(15 downto 0) := (others => '0');
signal speed_setpoint_mux : signed(15 downto 0);



    -- ============================================================
    -- Internal signals
    -- ============================================================
signal w_ref_i  : signed(31 downto 0);
signal w_meas_i : signed(31 downto 0);
signal i_meas_i : signed(31 downto 0); 
    -- Control loop enables
    signal current_enable : std_logic := '0';
    signal speed_enable   : std_logic := '0';
 
    -- Setpoint chain
    signal speed_setpoint      : signed(15 downto 0) := (others => '0');
    signal speed_ref_filtered  : signed(15 downto 0) := (others => '0');
 
    -- Cascade connection: speed PI output → current PI input
    signal current_ref : signed(15 downto 0) := (others => '0');
 
    -- Final voltage command
    signal v_cmd : signed(15 downto 0) := (others => '0');
 
    -- Feedback signals (muxed between sim and hardware sources)
    signal speed_fb_int   : signed(15 downto 0) := (others => '0');
    signal current_fb_int : signed(15 downto 0) := (others => '0');
 
    -- ADC interface signals
    signal adc_start     : std_logic := '0';
    signal adc_ready     : std_logic := '0';
    signal adc_current   : signed(15 downto 0) := (others => '0');
 
    -- Encoder interface signals
    signal enc_speed     : signed(15 downto 0) := (others => '0');
    signal enc_direction : std_logic := '1';
 
   signal uart_reg_addr : std_logic_vector(7 downto 0)  := (others => '0');
   signal uart_reg_data : std_logic_vector(15 downto 0) := (others => '0');
   signal uart_reg_wr   : std_logic := '0';
	
 
         

begin

-- pragma translate_off
--speed_setpoint_mux <= speed_setpoint_sim;
-- pragma translate_on

-- synthesis sees only this:
-- pragma translate_off
-- (hidden from synthesis)
-- pragma translate_on
-- real path:
speed_setpoint_mux <= speed_setpoint;



uart_ctrl: entity work.uart_controller
    generic map (
        CLK_FREQ  => 100_000_000,
        BAUD_RATE => 115200
    )
    port map (
        clk           => clk,
        reset         => reset,
        rx            => uart_rx,
        tx            => uart_tx,
        reg_addr      => uart_reg_addr,
        reg_data      => uart_reg_data,
        reg_wr        => uart_reg_wr,
        speed_fb_in   => speed_fb_int,
        current_fb_in => current_fb_int,
        v_cmd_in      => v_cmd,
        speed_sp_in   => speed_setpoint
    );
    
    uart_regs: process(clk, reset)
begin
    if reset = '1' then
        speed_setpoint <= (others => '0');
        uart_data_out  <= (others => '0');
    elsif rising_edge(clk) then
        -- Hardware: uart_controller writes
        if uart_reg_wr = '1' then
            case uart_reg_addr is
                when x"00" =>
                    speed_setpoint <= signed(uart_reg_data);
                when others => null;
            end case;
        end if;
        -- Simulation: parallel interface writes
        if SIM_MODE and uart_wr = '1' then
            case uart_addr is
                when x"00" =>
                    speed_setpoint <= signed(uart_data_in(15 downto 0));
                when others => null;
            end case;
        end if;
        -- Readback for simulation
        if SIM_MODE and uart_rd = '1' then
            uart_data_out <= std_logic_vector(resize(speed_setpoint, 32));
        end if;
    end if;
end process;


   

enable_gen: process(clk, reset)
    variable fast_cnt : integer := 0;
    variable spd_cnt  : integer := 0;
begin
    if reset = '1' then
        current_enable <= '0';
        speed_enable   <= '0';
        adc_start      <= '0';
        fast_cnt := 0;
        spd_cnt  := 0;
    elsif rising_edge(clk) then
        current_enable <= '0';
        speed_enable   <= '0';
        adc_start      <= '0';

        fast_cnt := fast_cnt + 1;

        -- Trigger ADC at cycle 8500
        -- 8500 × 10ns = 85μs
        -- ADC needs 12 SPI clocks × 1μs = 12μs
        -- Finishes at ~8512 → well before 10000
        if fast_cnt = 5000 then
            adc_start <= '1';
        end if;

        -- Current loop at exactly 10kHz
        if fast_cnt = 10000 then
            current_enable <= '1';
            fast_cnt := 0;
        end if;

        spd_cnt := spd_cnt + 1;
        if spd_cnt = 100000 then
            speed_enable <= '1';
            spd_cnt := 0;
        end if;
    end if;
end process;
        ---------------------------------------------------------------

        ---------------------------------------------------------------
    -- Reference Pre-filter (first-order IIR on speed setpoint)
    -- α = 1/32 (shift 5) → τ = 32ms → 95% settling at ~96ms
    -- ⚠️ CHANGE shift amount to tune rise time:
    --   shift 4 → τ=16ms (faster, more overshoot)
    --   shift 5 → τ=32ms (current, balanced)
    --   shift 6 → τ=64ms (slower, no overshoot)
    ---------------------------------------------------------------
    ref_filter: process(clk, reset)
        variable diff : signed(15 downto 0);
    begin
        if reset = '1' then
            speed_ref_filtered <= (others => '0');
        elsif rising_edge(clk) then
            if speed_enable = '1' then
                diff := speed_setpoint_mux - speed_ref_filtered;
                speed_ref_filtered <= speed_ref_filtered +
                                      shift_right(diff, 5);  -- α=1/32
            end if;
        end if;
    end process;


    ---------------------------------------------------------------
   -- Feedback Mux
    -- Speed:
    --   SIM  → speed_fb port (Q9.7, testbench drives directly)
    --   HW   → enc_speed from encoder_interface (Q9.7)
    --
    -- Current:
    --   BOTH → adc_current (always Q4.12)
    --   adc_interface handles conversion internally:
    --     SIM path: sim_current_ma (mA) → Q4.12
    --     HW  path: spi_miso raw        → Q4.12
    ---------------------------------------------------------------

process(clk, reset)
begin
    if reset = '1' then
        speed_fb_int <= (others => '0');
    elsif rising_edge(clk) then
        if SIM_MODE then
            speed_fb_int <= speed_fb;
        else
            -- Encoder updates every speed_enable pulse
            if speed_enable = '1' then
                speed_fb_int <= enc_speed;
            end if;
        end if;
    end if;
end process;
process(clk, reset)
begin
    if reset = '1' then
        current_fb_int <= (others => '0');
    elsif rising_edge(clk) then
        if SIM_MODE then
            -- Simulation: direct from plant, always fresh
            current_fb_int <= sim_current_ma;
        else
            -- Hardware: only latch when ADC conversion complete
            -- Prevents stale or glitched samples reaching PI
            if adc_ready = '1' then
                current_fb_int <= adc_current;
            end if;
        end if;
    end if;
end process;
 
--    ---------------------------------------------------------------
--    -- ADC Interface
--    -- SIM_MODE=true:  takes sim_current_ma, converts to Q4.12
--    -- SIM_MODE=false: runs SPI state machine, scales raw ADC to Q4.12
--    -- ⚠️ Update VREF_MV and I_MAX_MA when hardware is known
--    ---------------------------------------------------------------
    adc_inst: entity work.adc_interface
        generic map (
            SIM_MODE      => SIM_MODE,
            CLK_FREQ      => 100_000_000,
            SPI_FREQ      => 1_000_000,   -- ⚠️ CHANGE: match ADC chip max SPI speed
            VREF_MV       => 3300,         -- ⚠️ CHANGE: measure your Vref
            I_MAX_MA      => 5000,         -- ⚠️ CHANGE: compute from R_shunt × gain
            CURRENT_SCALE => 4096,         -- must match current PI ERROR_FRAC=12
            ADC_BITS      => 12            -- ⚠️ CHANGE if different ADC resolution
        )
        port map (
            clk            => clk,
            reset          => reset,
            start_conv     => adc_start,
            data_ready     => adc_ready,
            current_fb     => adc_current,
            spi_cs         => spi_cs,
            spi_clk        => spi_clk,
            spi_mosi       => spi_mosi,
            spi_miso       => spi_miso,
            sim_current_ma => sim_current_ma
        );
 


    ---------------------------------------------------------------
    -- Encoder Interface (hardware mode only)
    -- In simulation, enc_a and enc_b are tied off in testbench
    -- speed_fb port is used instead
    -- ⚠️ CHANGE PPR to match your physical motor encoder spec
    ---------------------------------------------------------------
    enc_inst: entity work.encoder_interface
        generic map (
            PPR          => 1024,      -- ⚠️ CHANGE: check motor label
            SPEED_SCALE  => 128,       -- Q9.7, must match speed PI ERROR_FRAC=7
            CLK_FREQ     => 100_000_000,
            SPEED_TS_CLK => 100_000    -- 1ms at 100MHz
        )
        port map (
            clk          => clk,
            reset        => reset,
            enc_a        => enc_a,
            enc_b        => enc_b,
            speed_enable => speed_enable,
            speed_out    => enc_speed,
            direction    => enc_direction
        );
 




    -- =========================
    -- SPEED LOOP (slow)
    -- =========================
  speed_pi: entity work.pi_controller_17bit
    generic map (
        ERROR_FRAC    => 7,
        INTEGRAL_FRAC => 15,
        OUTPUT_FRAC   => 12,
        KP_RAW        => SPEED_KP_RAW,
        KI_RAW        => SPEED_KI_RAW,
        TS_RAW        => SPEED_TS_RAW,
        TT_INV_RAW    => SPEED_TT_INV_RAW,
        KP_FRAC       => 16,   -- Q0.16
        KI_FRAC       => 12,   -- Q4.12
        TS_FRAC       => 20,
        U_MIN_RAW     => SPEED_U_MIN_RAW,
        U_MAX_RAW     => SPEED_U_MAX_RAW
    )
    port map (
        clk         => clk,
        reset       => reset,
        enable      => speed_enable,
        setpoint    => speed_ref_filtered,
        measurement => speed_fb_int,
        output_cmd  => current_ref
    );
    

    
    -- =========================
    -- CURRENT LOOP (fast)
    -- =========================
    
    current_pi: entity work.pi_controller_17bit
        generic map (
            ERROR_FRAC    => 12,
            INTEGRAL_FRAC => 8,
            OUTPUT_FRAC   => 10,

            KP_RAW     => CURRENT_KP_RAW,
            KI_RAW     => CURRENT_KI_RAW,
            TS_RAW     => CURRENT_TS_RAW,
            TT_INV_RAW => CURRENT_TT_INV_RAW,

            KP_FRAC       => 14,
            KI_FRAC       => 2,
            TS_FRAC       => 20, 
       
            U_MIN_RAW  => CURRENT_U_MIN_RAW,
            U_MAX_RAW  => CURRENT_U_MAX_RAW
        )
        port map (
            clk         => clk,
            reset       => reset,
            enable      => current_enable, -- 10 kHz !!
            setpoint    => current_ref,
            measurement => current_fb_int,
            output_cmd  => v_cmd
        );

 --It still works correctly - zero voltage gives 50% duty which is correct for a H-bridge.
 -- But you're not using the full PWM range. For now it's fine.
 --  When you get to hardware just note that your effective PWM resolution is slightly reduced
pwm_gen: entity work.pwm_generator
    port map (
        clk     => clk,
        reset   => reset,
        v_cmd   => v_cmd,
        pwm_out => pwm_out
    );

    -- Debug output
    v_cmd_test <= v_cmd;
w_ref_i  <= resize(speed_setpoint, 32);
w_meas_i <= resize(speed_fb_int, 32);
i_meas_i <= resize(current_fb_int, 32);

w_ref  <= w_ref_i;
w_meas <= w_meas_i;
i_meas <= i_meas_i;
speed_setpoint_debug <= speed_setpoint;
end Behavioral;

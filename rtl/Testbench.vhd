library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use IEEE.MATH_REAL.ALL;
use work.motor_constants_pkg.all;

entity tb_top is
end tb_top;

architecture Behavioral of tb_top is

    -- Clock and reset
    signal clk   : std_logic := '0';
    signal reset : std_logic := '1';

    -- Encoder (not used in this sim, tied off)
    signal enc_a : std_logic := '0';
    signal enc_b : std_logic := '0';

    -- DUT outputs
    signal pwm_out    : std_logic;
    signal v_cmd_test : signed(15 downto 0);

    -- Feedback to DUT (driven by plant model)
    signal speed_fb_sig   : signed(15 downto 0) := (others => '0');
    signal sim_current_ma_sig : signed(15 downto 0) := (others => '0');

    -- Enable strobes - generated here, wired into DUT if exposed as ports
    -- If top generates them internally, these are just for monitoring
    signal current_enable : std_logic := '0';
    signal speed_enable   : std_logic := '0';
 

    -- UART register interface
    signal uart_addr     : std_logic_vector(7 downto 0)  := (others => '0');
    signal uart_wr       : std_logic := '0';
    signal uart_rd       : std_logic := '0';
    signal uart_data_in  : std_logic_vector(31 downto 0) := (others => '0');
    signal uart_data_out : std_logic_vector(31 downto 0);


    -- Plant model (real-valued continuous state)
    -- Nominal parameters
    constant MOTOR_R  : real := 2.5;
    constant MOTOR_L  : real := 0.0005;
    constant MOTOR_J  : real := 0.0001;
    constant MOTOR_B  : real := 0.0001;
    constant MOTOR_KT : real := 0.05;
    constant MOTOR_KE : real := 0.05;

    -- Perturbed plant (testbench uses mismatched params to stress controller)
    constant R_REAL : real := MOTOR_R * 1.15;
    constant L_REAL : real := MOTOR_L * 0.90;
    constant J_REAL : real := MOTOR_J * 1.20;
    constant B_REAL : real := MOTOR_B * 1.10;

    -- Plant timestep - must match main clock period (10 ns = 100 MHz)
    -- Current loop Ts = 0.1 ms = 10000 clock cycles
    -- Speed    loop Ts = 1   ms = 100000 clock cycles
    -- DT = one clock = 10 ns so the plant integrates at full clock rate
    constant DT : real := 10.0e-9;

    signal i_plant : real := 0.0;
    signal w_plant : real := 0.0;
    signal sim_done : boolean := false;

    -- Helper: convert fixed-point signed to real
    function signed_to_real(s : signed; scale : integer) return real is
    begin
        return real(to_integer(s)) / real(scale);
    end function;

begin

    -------------------------------------------------------------------------
    -- DUT
    -------------------------------------------------------------------------
    DUT: entity work.top
        port map (
            clk           => clk,
            reset         => reset,
            enc_a         => enc_a,
            enc_b         => enc_b,
            pwm_out       => pwm_out,
            v_cmd_test    => v_cmd_test,
            speed_fb      => speed_fb_sig,
            sim_current_ma => sim_current_ma_sig,
            -- SPI pins tied off in simulation
            spi_cs         => open,
            spi_clk        => open,
            spi_mosi       => open,
            spi_miso       => '0',
            uart_rx        => '1',   -- ← idle state for UART line
            uart_tx        => open,   -- ← ignore in simulation
            uart_addr     => uart_addr,
            uart_wr       => uart_wr,
            uart_rd       => uart_rd,
            uart_data_in  => uart_data_in,
            uart_data_out => uart_data_out 
           

        );

    -------------------------------------------------------------------------
    -- Clock: 100 MHz → period = 10 ns
    -------------------------------------------------------------------------
    clk_process: process
    begin
        while not sim_done loop
            clk <= '0'; wait for 5 ns;
            clk <= '1'; wait for 5 ns;
        end loop;
        wait;
    end process;

    -------------------------------------------------------------------------
    -- Enable strobe generator
    -- Current loop: every 10000 clocks  → 10 kHz (Ts = 0.1 ms)
    -- Speed loop:   every 100000 clocks →  1 kHz (Ts = 1 ms)
    -- NOTE: if top generates these internally, this process is
    --       only here for waveform monitoring - it does not break anything
    -------------------------------------------------------------------------
    enable_gen: process(clk, reset)
        variable fast_cnt : integer := 0;
        variable spd_cnt  : integer := 0;
    begin
        if reset = '1' then
            current_enable <= '0';
            speed_enable   <= '0';
            fast_cnt := 0;
            spd_cnt  := 0;
        elsif rising_edge(clk) then
            current_enable <= '0';
            speed_enable   <= '0';

            fast_cnt := fast_cnt + 1;
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

    -------------------------------------------------------------------------
    -- Stimulus
    -------------------------------------------------------------------------
    stimulus: process
begin
--        -- Hold reset
--        reset <= '1';
--        wait for 200 ns;
--        reset <= '0';
--        wait for 500 ns;  -- let pipeline settle after reset

--        -- Write speed setpoint = 6400 to register 0x00
--        -- 6400 decimal = 0x1900
       
--        -- Run for 50 ms - long enough to see speed loop settle
--        wait for 200 ms;
--        sim_done <= true;
--        report "Simulation completed." severity note;
--        wait;

    
    -- Reset DUT
    reset <= '1';
    wait for 20 ns;
    reset <= '0';
    wait for 20 ns;

    -- Example: write speed setpoint via UART parallel interface
    uart_addr    <= x"00";
    uart_data_in <= x"00001900";  -- 6400 decimal
    uart_wr      <= '1';
    wait for 10 ns;
    uart_wr      <= '0';

    -- Let the controller run for 50 ms
    wait for 200ms;

    sim_done <= true;
    report "Simulation completed." severity note;
    wait;
end process;
    

    -------------------------------------------------------------------------
    -- Plant model - synchronous, runs every clock cycle
    -- Integrates continuous motor equations at DT = 10 ns
    -------------------------------------------------------------------------
    plant_process: process(clk)
        variable v_cmd_real : real;
        variable di_dt      : real;
        variable dw_dt      : real;
    begin
        if rising_edge(clk) then
            if reset = '1' then
                i_plant       <= 0.0;
                w_plant       <= 0.0;
                speed_fb_sig  <= (others => '0');
                sim_current_ma_sig<= (others => '0');
            else
                -- Convert fixed-point voltage command to real
                v_cmd_real := signed_to_real(v_cmd_test, VOLTAGE_SCALE);

                -- Motor electrical: L * di/dt = V - R*i - Ke*w
                di_dt := (v_cmd_real - R_REAL * i_plant - MOTOR_KE * w_plant) / L_REAL;

                -- Motor mechanical: J * dw/dt = Kt*i - B*w
                dw_dt := (MOTOR_KT * i_plant - B_REAL * w_plant) / J_REAL;

                i_plant <= i_plant + di_dt * DT;
                w_plant <= w_plant + dw_dt * DT;

                -- Convert back to Q-format feedback for controller
                speed_fb_sig   <= to_signed(integer(w_plant * real(SPEED_SCALE)),   16);
                sim_current_ma_sig <= to_signed(integer(i_plant * real(CURRENT_SCALE)), 16);
                --sim_current_ma_sig <= to_signed(integer(i_plant * real(CURRENT_SCALE)), 16);
            end if;
        end if;
    end process;

    -------------------------------------------------------------------------
    -- Monitor - prints every 100k cycles (~1 ms real time)
    -------------------------------------------------------------------------
    monitor: process(clk)
        variable count : integer := 0;
    begin
        if rising_edge(clk) then
            if reset = '0' then
                count := count + 1;
                if count mod 10000 = 0 then
                    report "=== t=" & integer'image(count / 100000) & " ms ===" severity note;

                    -- Real plant state
                    report "  w_plant      = " & real'image(w_plant)  & " rad/s" severity note;
                    report "  i_plant      = " & real'image(i_plant)  & " A"     severity note;

                    -- Fixed-point feedback as seen by controller
                    report "  speed_fb     = " & integer'image(to_integer(speed_fb_sig))
                           severity note;
                    report "  current_ma      = " & integer'image(to_integer(sim_current_ma_sig))
                           severity note;

                    -- Voltage command from current PI (the output that drives the plant)
                    report "  v_cmd_test   = " & integer'image(to_integer(v_cmd_test))
                           severity note;

                    -- Enable strobes state
                    report "  speed_en     = " & std_logic'image(speed_enable)   severity note;
                    report "  current_en   = " & std_logic'image(current_enable) severity note;
                end if;
            end if;
        end if;
    end process;

end Behavioral;
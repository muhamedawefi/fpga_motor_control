library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- ============================================================
-- ADC Interface - SPI + Current Scaling
-- ============================================================
-- Reads 12-bit ADC value over SPI and converts to Q4.12
-- format expected by the current PI controller.
--
-- ⚠️  HARDWARE SETUP CHECKLIST (fill when you get the board):
--   1. Check ADC chip model on your board
--   2. Measure or look up Vref (likely 3.3V on Basys3)
--   3. Find current sensing circuit:
--      - Shunt resistor value (Ω)
--      - Amplifier gain (if any)
--      - I_MAX = Vref / (R_shunt × gain)
--   4. Update VREF_MV and I_MAX_MA generics
--   5. Set SIM_MODE => false for hardware
-- ============================================================

entity adc_interface is
    Generic (
        -- ⚠️ CHANGE: set to false for real hardware
        SIM_MODE   : boolean := true;

        -- ⚠️ CHANGE: match your ADC chip's SPI frequency limit
        -- Common values: 1_000_000 (1MHz safe), 10_000_000 (10MHz fast)
        SPI_FREQ   : integer := 1_000_000;

        CLK_FREQ   : integer := 100_000_000;  -- 100MHz, match your board

        -- ⚠️ CHANGE: reference voltage in millivolts
        -- Basys3 = 3300, 5V system = 5000, precision ref = 2048
        VREF_MV    : integer := 3300;

        -- ⚠️ CHANGE: maximum measurable current in milliamps
        -- Depends on your shunt resistor + amplifier gain:
        --   I_MAX = VREF / (R_shunt × amp_gain)
        -- Example: Vref=3.3V, R=0.1Ω, gain=6.6 → I_MAX=5A=5000mA
        I_MAX_MA   : integer := 5000;

        -- Q4.12 scale factor - must match PI controller ERROR_FRAC=12
        -- ⚠️ CHANGE only if you change current PI ERROR_FRAC
        CURRENT_SCALE : integer := 4096;

        -- ADC resolution - 12 bits for most common chips (MCP3201, ADS7816)
        -- ⚠️ CHANGE if your ADC has different resolution (10, 14, 16 bit)
        ADC_BITS   : integer := 12
    );
    Port (
        clk         : in  std_logic;
        reset       : in  std_logic;

        -- Trigger a new conversion (assert for 1 clock)
        start_conv  : in  std_logic;

        -- Pulses high for 1 clock when new data is ready
        data_ready  : out std_logic;

        -- Scaled current output in Q4.12 format → feeds current PI
        current_fb  : out signed(15 downto 0);

        -- SPI hardware pins
        -- ⚠️ CHANGE: wire these to actual FPGA pins in constraints file
        spi_cs      : out std_logic;
        spi_clk     : out std_logic;
        spi_mosi    : out std_logic;
        spi_miso    : in  std_logic;

        -- Simulation only: inject current value directly (in milliamps)
        -- Example: sim_current_ma = 1000 means 1.0A
        -- ⚠️ Drive this from your testbench plant model
        sim_current_ma : in signed(15 downto 0) := (others => '0')
    );
end adc_interface;

architecture Behavioral of adc_interface is

    -- State machine
    type state_type is (IDLE, SEND_CMD, READ_DATA, SCALE, DONE);

    -- SPI clock divider
    signal spi_clk_int : std_logic := '0';
    signal clk_count   : integer   := 0;

    -- clk_div: number of main clock cycles per SPI half-period
    -- SPI period = 2 × clk_div × (1/CLK_FREQ)
    -- ⚠️ Verify: 1/SPI_FREQ >= ADC chip minimum CS-to-CLK setup time
    constant clk_div : integer := CLK_FREQ / (2 * SPI_FREQ);

    -- SPI state
    signal state     : state_type                          := IDLE;
    signal shift_reg : std_logic_vector(ADC_BITS-1 downto 0) := (others => '0');
    signal bit_cnt   : integer range 0 to 15              := 0;

    -- Raw 12-bit ADC capture
    signal captured_raw : signed(ADC_BITS-1 downto 0) := (others => '0');

    -- Scaled output register
    signal current_fb_reg : signed(15 downto 0) := (others => '0');

    -- ============================================================
    -- Scaling constant (computed at elaboration - no runtime math)
    -- current_fb = ADC_raw × CURRENT_SCALE × I_MAX_MA
    --              / (2^ADC_BITS × 1000)
    --
    -- Example with defaults:
    --   ADC_raw=2048 (half scale) → current=2.5A
    --   current_fb = 2048 × 4096 × 5000 / (4096 × 1000)
    --              = 2048 × 5 = 10240
    --   10240 / 4096 = 2.5A ✓
    --
    -- ⚠️ VERIFY: after changing I_MAX_MA, check this constant
    -- in simulation by injecting known currents and verifying output
    -- ============================================================
    constant ADC_SCALE_NUM : integer := CURRENT_SCALE * I_MAX_MA;
    constant ADC_SCALE_DEN : integer := (2**ADC_BITS) * 1000;

begin

    ---------------------------------------------------------------
    -- SPI Clock Divider (hardware mode only)
    ---------------------------------------------------------------
    spi_clk_div: if not SIM_MODE generate
        process(clk, reset)
        begin
            if reset = '1' then
                clk_count   <= 0;
                spi_clk_int <= '0';
            elsif rising_edge(clk) then
                if clk_count = clk_div - 1 then
                    clk_count   <= 0;
                    spi_clk_int <= not spi_clk_int;
                else
                    clk_count <= clk_count + 1;
                end if;
            end if;
        end process;

        spi_clk <= spi_clk_int;
    end generate;

    ---------------------------------------------------------------
    -- Hardware SPI State Machine
    -- Runs on spi_clk_int edges for proper SPI timing
    --
    -- ⚠️ VERIFY timing with your specific ADC datasheet:
    --   - Some ADCs need CS low before CLK starts
    --   - Some need a null bit before data
    --   - MCP3201: 1 null bit + 12 data bits = 13 clocks total
    --   - ADS7816: 12 data bits directly
    -- ⚠️ CHANGE bit_cnt limit and SEND_CMD if your ADC
    --   needs a command word sent before reading
    ---------------------------------------------------------------
    adc_hw: if not SIM_MODE generate
        process(clk, reset)
        begin
            if reset = '1' then
                state       <= IDLE;
                data_ready  <= '0';
                shift_reg   <= (others => '0');
                bit_cnt     <= 0;
                captured_raw<= (others => '0');
                spi_mosi    <= '0';

            elsif rising_edge(spi_clk_int) then
                case state is

                    when IDLE =>
                        data_ready <= '0';
                        bit_cnt    <= 0;
                        if start_conv = '1' then
                            state <= SEND_CMD;
                        end if;

                    when SEND_CMD =>
                        -- ⚠️ CHANGE: if ADC needs a command byte, send it here
                        -- For most simple SAR ADCs, MOSI=0 is fine
                        spi_mosi <= '0';
                        state    <= READ_DATA;

                    when READ_DATA =>
                        -- MSB-first shift: new bit enters at MSB position
                        shift_reg <= spi_miso & shift_reg(ADC_BITS-1 downto 1);
                        bit_cnt   <= bit_cnt + 1;

                        -- ⚠️ CHANGE: if ADC sends null/start bits before data,
                        -- adjust this count (e.g. MCP3201 needs 13 not 12)
                        if bit_cnt = ADC_BITS - 1 then
                            captured_raw <= signed(shift_reg);
                            state        <= SCALE;
                        end if;

               when SCALE =>
    -- Safe scaling: avoid large intermediate product
    -- Equivalent to: (raw / 4095) × (I_MAX_MA/1000) × CURRENT_SCALE
    -- Precomputed: for I_MAX=5A, CURRENT_SCALE=4096
    -- multiplier = 5 × 4096 / 4095 ≈ 5
    -- ⚠️ RECALCULATE if I_MAX_MA changes
    current_fb_reg <= resize(captured_raw * to_signed(5, 16), 16);
   
                        state <= DONE;

                    when DONE =>
                        data_ready <= '1';
                        state      <= IDLE;

                    when others =>
                        state <= IDLE;

                end case;
            end if;
        end process;

        -- CS active low during conversion
        spi_cs     <= '0' when state /= IDLE else '1';
        current_fb <= current_fb_reg;

    end generate;

    ---------------------------------------------------------------
    -- Simulation Mode
    -- Bypasses SPI entirely - takes current directly in milliamps
    -- and converts to Q4.12 for the PI controller
    --
    -- In testbench, drive sim_current_ma from plant model:
    --   sim_current_ma <= to_signed(integer(i_plant * 1000.0), 16);
    --
    -- ⚠️ CHANGE: sim_current_ma units are milliamps
    -- If your plant uses different units, adjust the conversion
    -- in the testbench, not here
    ---------------------------------------------------------------
    adc_sim: if SIM_MODE generate
        process(clk, reset)
            variable scaled : signed(31 downto 0);
        begin
            if reset = '1' then
                state          <= IDLE;
                data_ready     <= '0';
                current_fb_reg <= (others => '0');

            elsif rising_edge(clk) then
                case state is
                    when IDLE =>
                        data_ready <= '0';
                        if start_conv = '1' then
                           -- Convert milliamps → Q4.12
                        -- current_fb = sim_current_ma × CURRENT_SCALE / 1000
                        -- Example: 1000mA × 4096 / 1000 = 4096 = 1A in Q4.12 ✓
                        scaled := resize(
    resize(sim_current_ma, 32) * to_signed(CURRENT_SCALE, 32),
    32
);
                        current_fb_reg <= resize(scaled / to_signed(1000, 32), 16);
                            state   <= READ_DATA;
                        end if;

                    when READ_DATA =>
                        state <= DONE;

                    when DONE =>
                        data_ready <= '1';
                        state      <= IDLE;

                    when others =>
                        state <= IDLE;
                end case;
            end if;
        end process;

        current_fb <= current_fb_reg;

        -- SPI pins idle in simulation
        spi_cs   <= '1';
        spi_clk  <= '0';
        spi_mosi <= '0';

    end generate;

end Behavioral;
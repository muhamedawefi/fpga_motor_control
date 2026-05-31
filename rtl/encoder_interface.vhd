library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- ============================================================
-- Encoder Interface - Quadrature Decoder + Speed Calculator
-- ============================================================
-- Method: pulse counting per speed loop window (1ms)
-- Resolution: 4 × PPR counts per revolution (quadrature)
--
-- Speed conversion (Q9.7 output, SPEED_SCALE=128):
--   speed_raw = count × (2π / (PPR × 4 × Ts)) × SPEED_SCALE
--   speed_raw = count × (6.2832 / (1024 × 4 × 0.001)) × 128
--   speed_raw = count × 196
--
-- At 50 rad/s: count=32, speed_raw=32×196=6272 ≈ 6400 ✓
-- Max signed 16-bit: 32767 → max speed = 167 rad/s before overflow
-- ============================================================

entity encoder_interface is
    Generic (
        PPR          : integer := 1024;    -- Pulses per revolution
        SPEED_SCALE  : integer := 128;     -- Q9.7: 2^7
        CLK_FREQ     : integer := 100_000_000;  -- 100 MHz
        SPEED_TS_CLK : integer := 100_000  -- Speed loop period in clocks (1ms)
    );
    Port (
        clk      : in  std_logic;
        reset    : in  std_logic;

        -- Raw encoder signals (asynchronous from motor)
        enc_a    : in  std_logic;
        enc_b    : in  std_logic;

        -- Speed loop strobe - latch and reset counter on this pulse
        speed_enable : in  std_logic;

        -- Output speed in Q9.7 format (matches SPEED_SCALE=128)
        speed_out    : out signed(15 downto 0);

        -- Direction: '1' = forward, '0' = reverse
        direction    : out std_logic
    );
end encoder_interface;

architecture Behavioral of encoder_interface is

    ---------------------------------------------------------------
    -- Double-register synchronizers (metastability protection)
    -- CRITICAL: enc_a and enc_b are asynchronous signals
    -- Without this, metastability can corrupt the count
    ---------------------------------------------------------------
    signal enc_a_s1, enc_a_s2, enc_a_s3 : std_logic := '0';
    signal enc_b_s1, enc_b_s2, enc_b_s3 : std_logic := '0';

    -- Previous synchronized values for edge detection
    signal enc_a_prev : std_logic := '0';
    signal enc_b_prev : std_logic := '0';

    -- Quadrature state machine
    -- State encodes [enc_a_sync, enc_b_sync]
    signal enc_state      : std_logic_vector(1 downto 0) := "00";
    signal enc_state_prev : std_logic_vector(1 downto 0) := "00";

    -- Pulse counter - counts edges within current window
    -- Signed to handle direction: positive=forward, negative=reverse
    signal pulse_count : signed(15 downto 0) := (others => '0');

    -- Latched count at speed_enable - used for speed calculation
    signal count_latched : signed(15 downto 0) := (others => '0');

    -- Speed conversion constant
    -- speed_raw = count × (2π × SPEED_SCALE) / (PPR × 4 × Ts_speed)
    -- = count × (6.2832 × 128) / (1024 × 4 × 0.001)
    -- = count × 804.25 / 4.096
    -- = count × 196.4 → use 196
    -- AFTER (hardcoded, verified manually = 196)
-- Formula: 6.2832 × 128 / (1024 × 4 × 0.001) = 196
-- ⚠️ RECALCULATE if PPR or SPEED_SCALE changes
constant CONV_NUM : integer := 196;


begin

    ---------------------------------------------------------------
    -- Stage 1: Double-register synchronizer for enc_a
    -- Two flip-flops minimum required for metastability resolution
    -- Third register used for edge detection
    ---------------------------------------------------------------
    sync_a: process(clk, reset)
    begin
        if reset = '1' then
            enc_a_s1 <= '0';
            enc_a_s2 <= '0';
            enc_a_s3 <= '0';
        elsif rising_edge(clk) then
            enc_a_s1 <= enc_a;    -- first register: captures raw signal
            enc_a_s2 <= enc_a_s1; -- second register: metastability resolved
            enc_a_s3 <= enc_a_s2; -- third register: previous value for edge detect
        end if;
    end process;

    ---------------------------------------------------------------
    -- Stage 2: Double-register synchronizer for enc_b
    ---------------------------------------------------------------
    sync_b: process(clk, reset)
    begin
        if reset = '1' then
            enc_b_s1 <= '0';
            enc_b_s2 <= '0';
            enc_b_s3 <= '0';
        elsif rising_edge(clk) then
            enc_b_s1 <= enc_b;
            enc_b_s2 <= enc_b_s1;
            enc_b_s3 <= enc_b_s2;
        end if;
    end process;

    ---------------------------------------------------------------
    -- Stage 3: Quadrature decoder
    -- Uses standard quadrature state transition table
    -- Counts ALL four edges (4x mode) for maximum resolution
    --
    -- State transitions:
    -- Forward:  00→01→11→10→00
    -- Reverse:  00→10→11→01→00
    --
    -- On each valid transition: +1 (forward) or -1 (reverse)
    -- Invalid transitions (2-bit change): glitch, ignore
    ---------------------------------------------------------------
    quad_decoder: process(clk, reset)
        variable state_now  : std_logic_vector(1 downto 0);
        variable state_prev : std_logic_vector(1 downto 0);
        variable transition : std_logic_vector(3 downto 0);
    begin
        if reset = '1' then
            enc_state      <= "00";
            enc_state_prev <= "00";
            pulse_count    <= (others => '0');
            count_latched  <= (others => '0');
            direction      <= '1';

        elsif rising_edge(clk) then

            -- Current and previous states from synchronized signals
            state_now  := enc_a_s2 & enc_b_s2;
            state_prev := enc_a_s3 & enc_b_s3;

            enc_state      <= state_now;
            enc_state_prev <= state_prev;

            -- Encode transition as 4-bit vector [prev_a, prev_b, now_a, now_b]
            transition := state_prev & state_now;

            -- Latch count and reset on speed_enable strobe
            if speed_enable = '1' then
                count_latched <= pulse_count;
                pulse_count   <= (others => '0');

            else
                -- Decode quadrature transitions
                -- Forward transitions: 00→01, 01→11, 11→10, 10→00
                if    transition = "0001" or
                      transition = "0111" or
                      transition = "1110" or
                      transition = "1000" then
                    pulse_count <= pulse_count + 1;
                    direction   <= '1';

                -- Reverse transitions: 00→10, 10→11, 11→01, 01→00
                elsif transition = "0010" or
                      transition = "1011" or
                      transition = "1101" or
                      transition = "0100" then
                    pulse_count <= pulse_count - 1;
                    direction   <= '0';

                -- No change or invalid (2-bit jump = glitch): ignore
                end if;
            end if;

        end if;
    end process;

    ---------------------------------------------------------------
    -- Stage 4: Speed calculation
    -- speed_raw = count_latched × CONV_NUM
    -- Output in Q9.7 format matching SPEED_SCALE = 128
    ---------------------------------------------------------------
    speed_calc: process(clk, reset)
        variable speed_v : signed(31 downto 0);
    begin
        if reset = '1' then
            speed_out <= (others => '0');
        elsif rising_edge(clk) then
            if speed_enable = '1' then
                -- Multiply count by conversion constant
                speed_v := count_latched * to_signed(CONV_NUM, 16);
                -- Saturate to 16-bit output
                if speed_v > 32767 then
                    speed_out <= to_signed(32767, 16);
                elsif speed_v < -32768 then
                    speed_out <= to_signed(-32768, 16);
                else
                    speed_out <= speed_v(15 downto 0);
                end if;
            end if;
        end if;
    end process;

end Behavioral;
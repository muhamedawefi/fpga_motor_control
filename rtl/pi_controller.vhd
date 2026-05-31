library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use work.fixed_point_pkg.all;
use work.motor_constants_pkg.all;

entity pi_controller_17bit is
    Generic (
        -- Q-format parameters
        ERROR_FRAC     : integer := 7;
        INTEGRAL_FRAC  : integer := 15;
        OUTPUT_FRAC    : integer := 12;

        -- Controller gains
        KP_RAW         : signed(16 downto 0) := SPEED_KP_RAW;
        KI_RAW         : signed(16 downto 0) := SPEED_KI_RAW;
        TS_RAW         : signed(19 downto 0) := SPEED_TS_RAW;
        TT_INV_RAW     : signed(15 downto 0) := SPEED_TT_INV_RAW;

        -- Saturation limits
        U_MIN_RAW      : signed(15 downto 0) := SPEED_U_MIN_RAW;
        U_MAX_RAW      : signed(15 downto 0) := SPEED_U_MAX_RAW;

        -- Gain Q-formats
        KP_FRAC        : integer := 16;
        KI_FRAC        : integer := 12;
        TS_FRAC        : integer := 20
    );

    Port (
        clk          : in  std_logic;
        reset        : in  std_logic;
        enable       : in  std_logic;
        setpoint     : in  signed(15 downto 0);
        measurement  : in  signed(15 downto 0);
        output_cmd   : out signed(15 downto 0)
    );
end pi_controller_17bit;

architecture Behavioral of pi_controller_17bit is

    signal error        : signed(15 downto 0) := (others => '0');
    signal integral     : signed(31 downto 0) := (others => '0');
    signal p_term       : signed(31 downto 0) := (others => '0');
    signal i_term       : signed(31 downto 0) := (others => '0');
    signal output_sat32 : signed(31 downto 0) := (others => '0');

begin

process(clk, reset)

    variable error_v        : signed(15 downto 0);
    variable p_term_v       : signed(31 downto 0);
    variable i_term_v       : signed(31 downto 0);

    variable ki_error       : signed(31 downto 0);
    variable delta_i        : signed(31 downto 0);
    variable integral_next  : signed(31 downto 0);

    variable sat_error      : signed(31 downto 0);
    variable correction     : signed(31 downto 0);
    variable corr_scaled    : signed(31 downto 0);

    variable output_unsat_v : signed(31 downto 0);
    variable output_sat_v   : signed(31 downto 0);

begin
    if reset = '1' then
        error        <= (others => '0');
        integral     <= (others => '0');
        p_term       <= (others => '0');
        i_term       <= (others => '0');
        output_sat32 <= (others => '0');

    elsif rising_edge(clk) then

        if enable = '1' then

            -- =========================
            -- ERROR
            -- =========================
            error_v := setpoint - measurement;
            error   <= error_v;

            -- =========================
            -- P TERM
            -- =========================
            p_term_v := resize(
                fp_mult(KP_RAW, error_v,
                        KP_FRAC, ERROR_FRAC,
                        OUTPUT_FRAC), 32);

            p_term <= p_term_v;

            -- =========================
            -- INTEGRAL UPDATE
            -- =========================
            ki_error := resize(
                fp_mult(KI_RAW, error_v,
                        KI_FRAC, ERROR_FRAC,
                        INTEGRAL_FRAC), 32);

            delta_i := resize(
                fp_mult(ki_error, TS_RAW,
                        INTEGRAL_FRAC, TS_FRAC,
                        INTEGRAL_FRAC), 32);

            integral_next := integral + delta_i;

            -- =========================
            -- I TERM SCALING
            -- =========================
            if INTEGRAL_FRAC > OUTPUT_FRAC then
                i_term_v := shift_right(
                    integral_next,
                    INTEGRAL_FRAC - OUTPUT_FRAC);

            elsif INTEGRAL_FRAC < OUTPUT_FRAC then
                i_term_v := shift_left(
                    integral_next,
                    OUTPUT_FRAC - INTEGRAL_FRAC);

            else
                i_term_v := integral_next;
            end if;

            i_term <= i_term_v;

            -- =========================
            -- OUTPUT
            -- =========================
            output_unsat_v := p_term_v + i_term_v;

            output_sat_v := saturate(
                output_unsat_v,
                resize(U_MIN_RAW,32),
                resize(U_MAX_RAW,32)
            );

           -- =========================
            -- ANTI-WINDUP
            -- =========================
            if output_sat_v /= output_unsat_v then
                sat_error := output_sat_v - output_unsat_v;

               correction := resize(
                    fp_mult(TT_INV_RAW, sat_error,
                            0, OUTPUT_FRAC,
                            OUTPUT_FRAC), 32);

                corr_scaled := resize(
                    fp_mult(correction, TS_RAW,
                            OUTPUT_FRAC, TS_FRAC,
                            INTEGRAL_FRAC), 32);

                integral_next := integral_next + corr_scaled;
            end if;

            -- =========================
            -- COMMIT INTEGRAL (FIXED)
            -- =========================
            integral <= integral_next;

            output_sat32 <= output_sat_v;

        end if;
    end if;
end process;

-- Final output
output_cmd <= resize(output_sat32, 16);

end Behavioral;

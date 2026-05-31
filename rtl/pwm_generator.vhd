library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity pwm_generator is
    Port (
        clk       : in  std_logic;
        reset     : in  std_logic;
        v_cmd     : in  signed(15 downto 0);  -- Q6.10 voltage, signed
        pwm_out   : out std_logic
    );
end pwm_generator;

architecture Behavioral of pwm_generator is
    constant PWM_MAX : integer := 1023;  -- 10-bit PWM
    signal counter   : unsigned(9 downto 0) := (others => '0');
    signal duty      : unsigned(9 downto 0) := (others => '0');
begin

    -- Map signed v_cmd [-32768..32767] to PWM [0..1023]
    process(v_cmd)
        variable v_int : integer;
        variable temp  : integer;
    begin
        v_int := to_integer(v_cmd);
        -- Scale and offset for 10-bit PWM
        temp := (v_int + 32768) * PWM_MAX / 65535;  -- now 0..1023
        -- Clamp to range just in case
        if temp < 0 then
            duty <= (others => '0');
        elsif temp > PWM_MAX then
            duty <= to_unsigned(PWM_MAX, 10);
        else
            duty <= to_unsigned(temp, 10);
        end if;
    end process;

    -- PWM counter
    process(clk, reset)
    begin
        if reset = '1' then
            counter <= (others => '0');
            pwm_out <= '0';
        elsif rising_edge(clk) then
            counter <= counter + 1;
            if counter < duty then
                pwm_out <= '1';
            else
                pwm_out <= '0';
            end if;
        end if;
    end process;

end Behavioral;
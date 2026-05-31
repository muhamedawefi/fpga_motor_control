library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity register_map is
    Port (
        clk   : in  std_logic;
        reset : in  std_logic;

        -- Interface with UART
        addr      : in  std_logic_vector(7 downto 0);
        write_en  : in  std_logic;
        read_en   : in  std_logic;
        data_in   : in  std_logic_vector(31 downto 0);
        data_out  : out std_logic_vector(31 downto 0);

        -- Interface with control system
        speed_ref_out   : out signed(15 downto 0);
        speed_meas_in   : in  signed(15 downto 0);
        current_meas_in : in  signed(15 downto 0);
        voltage_cmd_in  : in  signed(15 downto 0)
    );
end register_map;

architecture Behavioral of register_map is

    -- Registers
    signal speed_ref_reg : signed(15 downto 0) := (others => '0');
   
    signal control_reg   : std_logic_vector(31 downto 0) := (others => '0');

begin

    -- WRITE logic
    process(clk)
    begin
        if rising_edge(clk) then
            if reset = '1' then
                speed_ref_reg <= (others => '0');
                control_reg   <= (others => '0');

            elsif write_en = '1' then
                case addr is
                    when x"00" =>
                        speed_ref_reg <= signed(data_in(15 downto 0));

                    when x"10" =>
                        control_reg <= data_in;

                    when others =>
                        null;
                end case;
            end if;
        end if;
    end process;

    -- READ logic
    process(addr, read_en, speed_meas_in, current_meas_in, voltage_cmd_in)
    begin
        if read_en = '1' then
            case addr is
                when x"04" =>
                    data_out <= std_logic_vector(resize(speed_meas_in, 32));

                when x"08" =>
                    data_out <= std_logic_vector(resize(current_meas_in, 32));

                when x"0C" =>
                    data_out <= std_logic_vector(resize(voltage_cmd_in, 32));

                when others =>
                    data_out <= (others => '0');
            end case;
        else
            data_out <= (others => '0');
        end if;
    end process;

    -- Output mapping
    speed_ref_out <= speed_ref_reg;

end Behavioral;

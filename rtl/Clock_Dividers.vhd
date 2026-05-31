library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
entity clock_divider is
    Port (
        clk       : in  std_logic;
        reset     : in  std_logic;
        enable_1k : out std_logic;  -- 1 kHz
        enable_10k: out std_logic;  -- 10 kHz
        enable_pre_10k : out std_logic
    );
end clock_divider;

architecture Behavioral of clock_divider is

    signal cnt_10k : integer := 0;
    signal cnt_1k  : integer := 0;
    signal pre_trigger : std_logic := '0';
begin
    process(clk, reset)
    begin
        if reset = '1' then
            cnt_10k <= 0;
            cnt_1k  <= 0;
            enable_10k <= '0';
            enable_1k  <= '0';
            enable_pre_10k <= '0';
        elsif rising_edge(clk) then
            -- 100 MHz / 10000 = 10 kHz
            if cnt_10k = 9999 then
                cnt_10k <= 0;
                enable_10k <= '1';
            else
                cnt_10k <= cnt_10k + 1;
                enable_10k <= '0';
            end if;
            
    -- PRE-TRIGGER (NEW) to guarentee that we give the adc enough time to not screw it 
    if cnt_10k = 9000 then
        enable_pre_10k <= '1';
    else
        enable_pre_10k <= '0';
    end if;

            
            -- 10 kHz / 10 = 1 kHz
            if cnt_10k = 9999 then
                if cnt_1k = 9 then
                    cnt_1k <= 0;
                    enable_1k <= '1';
                else
                    cnt_1k <= cnt_1k + 1;
                    enable_1k <= '0';
                end if;
            end if;
        end if;
    end process;
end Behavioral;
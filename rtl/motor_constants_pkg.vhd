library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

package motor_constants_pkg is
    
    -- Q-format scale
    constant SPEED_SCALE      : integer := 128;    -- 2^7 for Q9.7
    constant CURRENT_SCALE    : integer := 4096;   -- 2^12 for Q4.12
    constant VOLTAGE_SCALE    : integer := 1024;   -- 2^10 for Q6.10
    constant SPEED_INT_SCALE  : integer := 32768;  -- 2^15 for Q9.15
    constant CURRENT_INT_SCALE: integer := 256;    -- 2^8 for Q16.8
    
    -- Speed PI controller constants (Q-format raw values)
    constant SPEED_KP_RAW     : signed(16 downto 0) := to_signed(8101, 17);   -- Q0.16
    constant SPEED_KI_RAW     : signed(16 downto 0) := to_signed(32334, 17);  -- Q4.12
    constant SPEED_TS_RAW     : signed(19 downto 0) := to_signed(1049, 20);   -- Q0.20
    constant SPEED_TT_INV_RAW : signed(15 downto 0) := to_signed(638, 16);    -- Q16.0
    
    -- Current PI controller constan
    constant CURRENT_KP_RAW     : signed(16 downto 0) := to_signed(51472, 17);  -- Q2.14
    constant CURRENT_KI_RAW     : signed(16 downto 0) := to_signed(62832, 17);  -- Q14.2
    constant CURRENT_TS_RAW     : signed(19 downto 0) := to_signed(105, 20);    -- Q0.20
    constant CURRENT_TT_INV_RAW : signed(15 downto 0) := to_signed(10000, 16);  -- Q16.0
    
    -- Output limits (saturation)
    constant SPEED_U_MIN_RAW    : signed(15 downto 0) := to_signed(-20480, 16); -- Q4.12: -5.0 A
    constant SPEED_U_MAX_RAW    : signed(15 downto 0) := to_signed(20480, 16);  -- Q4.12: +5.0 A
    constant CURRENT_U_MIN_RAW  : signed(15 downto 0) := to_signed(-24576, 16); -- Q6.10: -24.0 V
    constant CURRENT_U_MAX_RAW  : signed(15 downto 0) := to_signed(24576, 16);  -- Q6.10: +24.0 V
    
    -- Filter coefficients

    constant ALPHA_SPEED_RAW : signed(15 downto 0) := to_signed(59294, 16);  -- Q0.16
    constant ALPHA_REF_RAW   : signed(15 downto 0) := to_signed(64238, 16);  -- Q0.16

end package motor_constants_pkg;
